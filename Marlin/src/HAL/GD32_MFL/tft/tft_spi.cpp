// src/HAL/GD32_MFL/tft_spi.cpp
#include "tft_spi.h"
//#include "../../../lcd/tft_io/ST7789v.h"
#include "../../../lcd/tft_io/tft_io.h"
#include "../HAL.h"
#include "../../../inc/MarlinConfig.h"
#include "../HAL_SPI.h"
#include <Arduino.h>

#ifdef TFT_GENERIC
using namespace hal_bridge;

bool TFT_SPI::tft_busy = false;
uint16_t TFT_SPI::fill_color = 0;
uint16_t TFT_SPI::data_width = DATASIZE_8BIT;

// =========================================================
// === БАЗОВАЯ ЛОГИКА ===
// =========================================================
void TFT_SPI::set_busy(bool state) { tft_busy = state; }
bool TFT_SPI::get_busy() { return tft_busy; }

bool TFT_SPI::isBusy() {
    if (!tft_busy) return false;
    if (hal_bridge::spi_dma_is_busy()) return true;

    // Транзакция завершена. Обязательно глушим DMA-запрос SPI и возвращаем
    // 8-битный формат кадра: иначе следующая команда (CASET/RASET/RAMWR)
    // уйдёт как 16-битный кадр, дисплей её не поймёт и на экране будет шум.
    hal_bridge::spi_dma_stop();
    hal_bridge::spi_cs_control_direct(false);   // CS High
    data_width = DATASIZE_8BIT;
    set_busy(false);
    return false;
}

void TFT_SPI::backlight(bool on) {
    // Используем прямой вызов HAL для надежности или Arduino API
    digitalWrite(TFT_BACKLIGHT_PIN, on ? HIGH : LOW);
}

void TFT_SPI::abort() {
    hal_bridge::spi_dma_stop();
    set_busy(false);
    hal_bridge::spi_cs_control_direct(false);
}

// =========================================================
// === НИЗКИЙ УРОВЕНЬ (ИСПРАВЛЕНО: БЕЗ CS) ===
// =========================================================
// Эти функции вызываются ИЗНУТРИ транзакции (когда CS уже LOW).
// Они НЕ ДОЛЖНЫ трогать CS.

void TFT_SPI::writeReg(uint16_t reg) {
    // ВАЖНО: Не проверяем busy здесь, так как tft_io уже поставил busy в beginTransaction

    // 1. DC = CMD
    hal_bridge::spi_dc_control_direct(false);

    // 2. Отправляем команду в текущем формате кадра
    if (data_width == DATASIZE_16BIT)
        hal_bridge::spi_write_frame16_direct(reg);   // ILI9328/R61505: команда = 16 бит
    else
        hal_bridge::spi_write_byte_direct(reg & 0xFF);

    // 3. Ждем завершения (чтобы DC не переключился раньше времени)
    hal_bridge::spi_wait_for_tx_complete_direct();

    // 4. Возвращаем DC в DATA (стандартное состояние для tft_io)
    hal_bridge::spi_dc_control_direct(true);
}

void TFT_SPI::writeData(uint16_t data) {
    if (data_width == DATASIZE_16BIT) {
        // Один 16-битный кадр: старшие байты уходят первыми, как и ожидает контроллер
        hal_bridge::spi_write_frame16_direct(data);
    } else {
        // 8-битный режим: Марлин в tft_io.cpp сам разбивает 16-битные значения
        // (координаты окна и т.п.) на два вызова writeData — берем только байт.
        hal_bridge::spi_write_byte_direct(data & 0xFF);
    }

    // Ждем, чтобы не нарушить порядок при быстрых вызовах
    hal_bridge::spi_wait_for_tx_complete_direct();
}

// =========================================================
// === УПРАВЛЕНИЕ ТРАНЗАКЦИЕЙ (ЗДЕСЬ ЖИВЕТ CS) ===
// =========================================================

void TFT_SPI::dataTransferBegin(uint16_t dataWidth) {
    // Предыдущая DMA-транзакция должна гарантированно завершиться:
    // она оставляет SPI в 16-битном режиме с включённым DMATEN.
    while (isBusy()) { /* ждём */ }

    set_busy(true);

    // 1. Формат кадра под текущую транзакцию
    //    (DATASIZE_8BIT — ST7789/ILI9341 команды, DATASIZE_16BIT — ILI9328/R61505)
    data_width = dataWidth;
    hal_bridge::spi_set_frame_format_direct(dataWidth == DATASIZE_16BIT);

    // 2. Опускаем CS (Начало общения)
    hal_bridge::spi_cs_control_direct(true);

    // 3. По умолчанию DC = DATA
    hal_bridge::spi_dc_control_direct(true);

    // Errata Fix: Шина должна быть свободна перед стартом
    hal_bridge::spi_wait_for_tx_complete_direct();
}

void TFT_SPI::dataTransferEnd() {
    // 1. Ждем, пока всё улетит
    hal_bridge::spi_wait_for_tx_complete_direct();

    // 2. Поднимаем CS (Конец общения)
    hal_bridge::spi_cs_control_direct(false);

    // 3. Возврат в 8-битный режим — командная часть всегда начинается с него
    hal_bridge::spi_set_frame_format_direct(false);
    data_width = DATASIZE_8BIT;

    set_busy(false);
}

// =========================================================
// === МАССОВАЯ ЗАПИСЬ (САМОСТОЯТЕЛЬНОЕ УПРАВЛЕНИЕ) ===
// =========================================================
// Эти функции Марлин вызывает "как есть", поэтому здесь МЫ управляем CS.

void TFT_SPI::writeSequence(const uint16_t *data, uint32_t count) {
    if (!data || count == 0) return;
    while (isBusy()) { /* ждём завершения предыдущей транзакции */ }
    set_busy(true);

    hal_bridge::spi_cs_control_direct(true); // CS Low
    hal_bridge::spi_dc_control_direct(true); // DATA
    hal_bridge::spi_set_frame_format_direct(false);
    data_width = DATASIZE_8BIT;

    // Порядок байтов критичен: ST7789 принимает RGB565 как D15..D8, затем D7..D0.
    // Marlin хранит цвет в uint16_t как есть (ENDIAN_COLOR для SPI = identity),
    // а ARM — little-endian, поэтому байты из памяти идут LSB->MSB и их надо
    // отправлять в обратном порядке. Прежний побайтовый проход по buffer
    // переставлял байты местами и давал «цветной шум» вместо картинки.
    while (count--) {
        const uint16_t color = *data++;
        hal_bridge::spi_write_byte_direct((uint8_t)(color >> 8));   // MSB first
        hal_bridge::spi_write_byte_direct((uint8_t)(color & 0xFF)); // LSB second
    }

    hal_bridge::spi_wait_for_tx_complete_direct();
    hal_bridge::spi_cs_control_direct(false); // CS High
    set_busy(false);
}

void TFT_SPI::writeMultiple(uint16_t color, uint32_t count) {
    if (count == 0) return;
    while (isBusy()) { /* ждём завершения предыдущей транзакции */ }
    set_busy(true);

    hal_bridge::spi_cs_control_direct(true); // CS Low
    hal_bridge::spi_dc_control_direct(true); // DATA
    hal_bridge::spi_set_frame_format_direct(false);
    data_width = DATASIZE_8BIT;

    // Старший байт цвета отправляем первым (требование ST7789/ILI9341)
    const uint8_t byte_msb = (uint8_t)(color >> 8);
    const uint8_t byte_lsb = (uint8_t)(color & 0xFF);

    while (count--) {
        hal_bridge::spi_write_byte_direct(byte_msb);
        hal_bridge::spi_write_byte_direct(byte_lsb);
    }

    hal_bridge::spi_wait_for_tx_complete_direct();
    hal_bridge::spi_cs_control_direct(false); // CS High
    set_busy(false);
}

// =========================================================
// === ИНИЦИАЛИЗАЦИЯ ===
// =========================================================
void TFT_SPI::init() {
    // 1. Аппаратная инициализация (только мост)
    spi_init_direct();
    set_busy(false);

    // 2. Исходное состояние шины: CS отпущен, DC = DATA, формат — 8 бит
    hal_bridge::spi_cs_control_direct(false);
    hal_bridge::spi_dc_control_direct(true);
    hal_bridge::spi_set_frame_format_direct(false);
    data_width = DATASIZE_8BIT;

    // ВАЖНО: Вся логика Reset и отправки команд (commandList) 
    // находится в tft_io.cpp -> TFT_IO::initTFT().
    // Нам не нужно её дублировать, иначе будет двойной сброс и глюки.
    
    // Просто убеждаемся, что подсветка настроена (на случай если tft_io пропустит)
    pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
}

// =========================================================
// === ЗАГЛУШКИ (Используется логика tft_io.cpp) ===
// =========================================================

uint32_t TFT_SPI::getID() { return 0x8552; } // ID для ST7789

// =========================================================
// === DMA-РЕАЛИЗАЦИЯ (16-bit, неблокирующая, для TFT_COLOR_UI) ===
// =========================================================
// SPI переключается в 16-битный режим на время DMA-передачи.
// DMA width = 16-bit: каждый перенос = 1 пиксель (uint16_t).
// SPI аппаратно отправляет MSB first (LF=0) → правильный порядок для ST7789.
//
// writeSequence_DMA: count = пиксели, DMA читает buffer с инкрементом.
// writeMultiple_DMA: count = пиксели, DMA читает &fill_color без инкремента.
//
// Порядок вызовов важен: SPI/DMA настраиваются при поднятом CS (служебных
// фронтов SCK при перенастройте не возникает, потому что в момент
// перенастройки в SPI_DATA нет данных), затем поднимается DC=DATA и CS Low,
// и только после этого запускается сам перенос. Иначе первый кадр ушёл бы
// при поднятом CS, и весь ряд сдвинулся бы на пиксель.

void TFT_SPI::writeSequence_DMA(uint16_t *data, uint32_t count) {
    if (!data || count == 0) return;
    while (isBusy()) { /* ждём завершения предыдущей транзакции */ }

    // Счётчик канала — 16 бит, и 16-битные переносы требуют выравнивания.
    // Всё, что не вписывается, отдаём блокирующим путём.
    if (count > DMA_MAX_WORDS || ((uintptr_t)data & 1U)) {
        writeSequence(data, count);
        return;
    }

    set_busy(true);
    data_width = DATASIZE_16BIT;

    // 1. SPI -> 16-битный кадр + конфигурирование канала DMA (CHEN остаётся 0)
    if (!hal_bridge::spi_dma_prepare((uint32_t)(uintptr_t)data, count, true)) {
        writeSequence(data, count);   // не удалось — работаем побайтно
        return;
    }

    // 2. Режим данных и выборка — ДО старта переноса
    hal_bridge::spi_dc_control_direct(true);  // DATA mode
    hal_bridge::spi_cs_control_direct(true);  // CS Low

    // 3. Пускаем перенос: CHEN=1, затем DMATEN=1
    hal_bridge::spi_dma_trigger();
}

void TFT_SPI::writeMultiple_DMA(uint16_t color, uint32_t count) {
    if (count == 0) return;
    while (isBusy()) { /* ждём завершения предыдущей транзакции */ }

    if (count > DMA_MAX_WORDS) {
        writeMultiple(color, count);
        return;
    }

    set_busy(true);
    data_width = DATASIZE_16BIT;
    fill_color = color;

    if (!hal_bridge::spi_dma_prepare((uint32_t)(uintptr_t)&fill_color, count, false)) {
        writeMultiple(color, count);
        return;
    }

    hal_bridge::spi_dc_control_direct(true);  // DATA mode
    hal_bridge::spi_cs_control_direct(true);  // CS Low

    hal_bridge::spi_dma_trigger();
}

#endif // TFT_GENERIC