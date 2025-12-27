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

// =========================================================
// === БАЗОВАЯ ЛОГИКА ===
// =========================================================
void TFT_SPI::set_busy(bool state) { tft_busy = state; }
bool TFT_SPI::get_busy() { return tft_busy; }
bool TFT_SPI::isBusy() { return get_busy(); }

void TFT_SPI::backlight(bool on) {
    // Используем прямой вызов HAL для надежности или Arduino API
    digitalWrite(TFT_BACKLIGHT_PIN, on ? HIGH : LOW);
}

void TFT_SPI::abort() {
    set_busy(false);
    spi_cs_control_direct(false);
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
    
    // 2. Отправляем байт (команды всегда 8 бит)
    hal_bridge::spi_write_byte_direct(reg & 0xFF);
    
    // 3. Ждем завершения (чтобы DC не переключился раньше времени)
    hal_bridge::spi_wait_for_tx_complete_direct();
    
    // 4. Возвращаем DC в DATA (стандартное состояние для tft_io)
    hal_bridge::spi_dc_control_direct(true);
}

void TFT_SPI::writeData(uint16_t data) {
    // 1. Отправляем ТОЛЬКО МЛАДШИЙ БАЙТ (8 бит)
    // Марлин в tft_io.cpp сам разбивает 16-битные данные на два вызова writeData.
    hal_bridge::spi_write_byte_direct(data & 0xFF);

    // 2. Ждем (чтобы не нарушить порядок при быстрых вызовах)
    hal_bridge::spi_wait_for_tx_complete_direct();
} 

// =========================================================
// === УПРАВЛЕНИЕ ТРАНЗАКЦИЕЙ (ЗДЕСЬ ЖИВЕТ CS) ===
// =========================================================

void TFT_SPI::dataTransferBegin(uint16_t dataWidth) {
    if (get_busy()) return;
    set_busy(true);

    // 1. Опускаем CS (Начало общения)
    hal_bridge::spi_cs_control_direct(true);
    
    // 2. По умолчанию DC = DATA
    hal_bridge::spi_dc_control_direct(true);
    
    // Errata Fix: Чистим шину перед стартом
    hal_bridge::spi_wait_for_tx_complete_direct();
    
    (void)dataWidth;
}

void TFT_SPI::dataTransferEnd() {
    // 1. Ждем, пока всё улетит
    hal_bridge::spi_wait_for_tx_complete_direct();
    
    // 2. Поднимаем CS (Конец общения)
    hal_bridge::spi_cs_control_direct(false);
    
    set_busy(false);
}

// =========================================================
// === МАССОВАЯ ЗАПИСЬ (САМОСТОЯТЕЛЬНОЕ УПРАВЛЕНИЕ) ===
// =========================================================
// Эти функции Марлин вызывает "как есть", поэтому здесь МЫ управляем CS.

void TFT_SPI::writeSequence(const uint16_t *data, uint32_t count) {
    if (!data || count == 0) return;
    if (get_busy()) return;
    set_busy(true);

    hal_bridge::spi_cs_control_direct(true); // CS Low
    hal_bridge::spi_dc_control_direct(true); // DATA

    // Приводим данные к 8-битному виду для побайтовой отправки
    const uint8_t *byte_stream = (const uint8_t *)data;

    // Считаем общее количество байт (2 байта на 1 слово цвета)
    uint32_t bytes_total = count * 2;

    while (bytes_total--) {
        // Берем текущий байт из потока и сдвигаемся к следующему
        hal_bridge::spi_write_byte_direct(*byte_stream++); 
    }

    hal_bridge::spi_wait_for_tx_complete_direct();
    hal_bridge::spi_cs_control_direct(false); // CS High
    set_busy(false);
}

void TFT_SPI::writeMultiple(uint16_t color, uint32_t count) {
    if (count == 0) return;
    if (get_busy()) return;
    set_busy(true);

    hal_bridge::spi_cs_control_direct(true); // CS Low
    hal_bridge::spi_dc_control_direct(true); // DATA

    // 1. Получаем адрес переменной color и смотрим на неё как на массив байтов.
    // Так как Марлин уже развернул байты, в памяти они лежат в нужном порядке:
    // [0] -> Старший байт (MSB), [1] -> Младший байт (LSB)
    const uint8_t *color_bytes = (const uint8_t *)&color;
    
    // 2. Кэшируем байты в локальные переменные, чтобы не обращаться 
    // к стеку/памяти на каждой итерации цикла.
    uint8_t byte_1 = color_bytes[0];
    uint8_t byte_2 = color_bytes[1];

    while (count--) {
        hal_bridge::spi_write_byte_direct(byte_1);
        hal_bridge::spi_write_byte_direct(byte_2);
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

void TFT_SPI::writeSequence_DMA(uint16_t *data, uint32_t count) {
    writeSequence(data, count);
}
void TFT_SPI::writeMultiple_DMA(uint16_t color, uint32_t count) {
    writeMultiple(color, count);
}

#endif // TFT_GENERIC