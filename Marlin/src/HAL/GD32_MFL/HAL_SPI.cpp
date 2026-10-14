//File hal-bridge HAL_SPI.cpp
#include "../platforms.h"
#ifdef ARDUINO_ARCH_MFL

#include "HAL_SPI.h"
#include "../../inc/MarlinConfig.h"
#include "Arduino.h"

// ==========================================================
// === ПРЯМЫЕ АДРЕСА РЕГИСТРОВ (Hardcoded) ===
// ==========================================================

// RCU
#define MFL_RCU_BASE        0x40021000U
#define MFL_RCU_APB2EN      (*(volatile uint32_t *)(MFL_RCU_BASE + 0x18U))

// GPIO A
#define MFL_GPIOA_BASE      0x40010800U
#define MFL_GPIOA_CTL0      (*(volatile uint32_t *)(MFL_GPIOA_BASE + 0x00U))
#define MFL_GPIOA_BOP       (*(volatile uint32_t *)(MFL_GPIOA_BASE + 0x10U))
#define MFL_GPIOA_BC        (*(volatile uint32_t *)(MFL_GPIOA_BASE + 0x14U))

// SPI0
#define MFL_SPI0_BASE       0x40013000U
#define MFL_SPI0_CTL0       (*(volatile uint32_t *)(MFL_SPI0_BASE + 0x00U))
#define MFL_SPI0_STAT       (*(volatile uint32_t *)(MFL_SPI0_BASE + 0x08U))
#define MFL_SPI0_DATA       (*(volatile uint32_t *)(MFL_SPI0_BASE + 0x0CU))

// ------------------------------------------------------------
// RCU: тактирование DMA0 находится в AHB-регистре, а НЕ в APB1!
// gd32f30x_rcu.h: RCU_AHBEN = base+0x14, RCU_AHBEN_DMA0EN = BIT(0)
// ------------------------------------------------------------
#define MFL_RCU_AHBEN       (*(volatile uint32_t *)(MFL_RCU_BASE + 0x14U))
#define M_RCU_DMA0EN        (1U << 0)

// ------------------------------------------------------------
// DMA0 (base 0x40020000). По даташиту GD32F303: SPI0_TX -> DMA0 Channel 2.
// Смещения каналов: CHxCTL = 0x08 + 0x14*x  ->  CH2CTL=0x30, CH2CNT=0x34,
// CH2PADDR=0x38, CH2MADDR=0x3C (совпадает с gd32f30x_dma.h).
// ------------------------------------------------------------
#define MFL_DMA0_BASE       0x40020000U
#define MFL_DMA0_INTF       (*(volatile uint32_t *)(MFL_DMA0_BASE + 0x00U))
#define MFL_DMA0_INTC       (*(volatile uint32_t *)(MFL_DMA0_BASE + 0x04U))
#define MFL_DMA0_CH2_CTL    (*(volatile uint32_t *)(MFL_DMA0_BASE + 0x30U))
#define MFL_DMA0_CH2_CNT    (*(volatile uint32_t *)(MFL_DMA0_BASE + 0x34U))
#define MFL_DMA0_CH2_PADDR  (*(volatile uint32_t *)(MFL_DMA0_BASE + 0x38U))
#define MFL_DMA0_CH2_MADDR  (*(volatile uint32_t *)(MFL_DMA0_BASE + 0x3CU))

// Флаги канала: на каждый канал приходится 4 бита (GIF, FTFIF, HTFIF, ERRIF),
// поэтому для канала 2 это биты 8..11 регистра DMA_INTF / DMA_INTC.
#define M_DMA_GIF2          (1U << 8)
#define M_DMA_FTFIF2        (1U << 9)
#define M_DMA_HTFIF2        (1U << 10)
#define M_DMA_ERRIF2        (1U << 11)
#define M_DMA_CH2_ALL       (M_DMA_GIF2 | M_DMA_FTFIF2 | M_DMA_HTFIF2 | M_DMA_ERRIF2)

// Биты DMA_CHxCTL (gd32f30x_dma.h)
#define M_DMA_CHEN          (1U << 0)
#define M_DMA_FTFIE         (1U << 1)
#define M_DMA_HTFIE         (1U << 2)
#define M_DMA_ERRIE         (1U << 3)
#define M_DMA_DIR_M2P       (1U << 4)   // 1 = memory -> peripheral
#define M_DMA_CMEN          (1U << 5)   // circular mode (здесь не используется)
#define M_DMA_PNAGA         (1U << 6)   // 0 = адрес периферии постоянен (SPI_DATA)
#define M_DMA_MNAGA         (1U << 7)   // инкремент адреса памяти
#define M_DMA_PWIDTH_16     (1U << 8)   // PWIDTH = 01 -> 16-bit
#define M_DMA_MWIDTH_16     (1U << 10)  // MWIDTH = 01 -> 16-bit
#define M_DMA_PRIO_HIGH     (2U << 12)
#define M_DMA_M2M           (1U << 14)  // memory-to-memory (не используется, = 0)

// CNT[15:0] — 16-битный счётчик; ноль означает «транзакций нет».
#define M_DMA_CNT_MAX       0xFFFFU

// Watchdog: счётчик не двигается дольше этого времени -> запросов от SPI нет.
#define DMA_STALL_TIMEOUT_US      5000UL
#define DMA_TRANSFER_TIMEOUT_US   200000UL
#define DMA_SPI_FLAG_TIMEOUT_US   100000UL

// ==========================================================
// === МАСКИ ===
// ==========================================================
#define M_SPI_CKPH    (1U << 0U)
#define M_SPI_CKPL    (1U << 1U)
#define M_SPI_MSTMOD  (1U << 2U)
#define M_SPI_PSC_2   (0U << 3U)  // Делитель 2  → APB2/2
#define M_SPI_PSC_4   (1U << 3U)  // Делитель 4  → APB2/4
#define M_SPI_PSC_8   (2U << 3U)  // Делитель 8  → APB2/8
#define M_SPI_PSC_16  (3U << 3U)  // Делитель 16 → APB2/16

// Делитель тактирования SPI0 по умолчанию.
// При APB2 = 120 МГц: PSC_8 = 15 МГц, PSC_4 = 30 МГц.
#ifndef TFT_SPI_PSC
  #define TFT_SPI_PSC M_SPI_PSC_4
#endif
#define M_SPI_SPIEN   (1U << 6U)
#define M_SPI_LF      (1U << 7U)   // 1 = LSB first; держим 0 (MSB first — ST7789)
#define M_SPI_SWNSS   (1U << 8U)
#define M_SPI_SWNSSEN (1U << 9U)
#define M_SPI_FF16    (1U << 11U)  // 16-битный формат кадра в CTL0
#define M_SPI_TBE     (1U << 1U)
#define M_SPI_TRANS   (1U << 7U)

// SPI0 CTL1 (DMA enable)
#define MFL_SPI0_CTL1 (*(volatile uint32_t *)(MFL_SPI0_BASE + 0x04U))
#define M_SPI_DMAREN  (1U << 0U)   // RX DMA request
#define M_SPI_DMATEN  (1U << 1U)   // TX DMA request

// Bidirectional Mode
#define M_SPI_BDOEN   (1U << 14U) 
#define M_SPI_BDEN    (1U << 15U) 

namespace hal_bridge {

// ==========================================================
// === ИНИЦИАЛИЗАЦИЯ ===
// ==========================================================
bool spi_init_direct() {
    // 1. Clocks: APB2 -> AF(bit0) + GPIOA(bit2) + SPI0(bit12)
    MFL_RCU_APB2EN |= (1U << 0U) | (1U << 2U) | (1U << 12U);

    // Тактирование DMA0 включаем заранее: DMA0 висит на AHB (RCU_AHBEN bit0),
    // без тактов обращение к регистрам DMA подвешивает шину.
    MFL_RCU_AHBEN |= M_RCU_DMA0EN;
    
    // Барьер синхронизации памяти
    __DSB(); 
    
    // 2. GPIO Config
    uint32_t temp_ctl0 = MFL_GPIOA_CTL0;
    temp_ctl0 &= 0x0000FFFFU; 
    
    // PA4(CS)=Out, PA5(SCK)=Alt, PA6(DC)=Out, PA7(MOSI)=Alt
    temp_ctl0 |= (0x3U << 16) | (0xBU << 20) | (0x3U << 24) | (0xBU << 28);
    
    MFL_GPIOA_CTL0 = temp_ctl0;
    MFL_GPIOA_BOP = (1U << 4) | (1U << 6);
    __DSB(); 
    
    // 3. SPI Config
    MFL_SPI0_CTL0 &= ~M_SPI_SPIEN; 
    
    // Сбрасываем возможные флаги ошибок (Dummy read)
    volatile uint32_t dummy = MFL_SPI0_STAT;
    dummy = MFL_SPI0_DATA;
    (void)dummy;
    
    uint32_t spi_cfg = 0;
    spi_cfg |= M_SPI_MSTMOD;         
    spi_cfg |= M_SPI_SWNSS | M_SPI_SWNSSEN; 
    spi_cfg |= M_SPI_CKPL | M_SPI_CKPH;     
    spi_cfg |= TFT_SPI_PSC;          
    spi_cfg |= M_SPI_BDEN | M_SPI_BDOEN;
    
    MFL_SPI0_CTL0 = spi_cfg;
    __DSB();
    
    MFL_SPI0_CTL0 |= M_SPI_SPIEN;

    return true;
}

// ==========================================================
// === ФОРМАТ ===
// ==========================================================
// FF16 в CTL0 можно менять только при выключенном SPI (SPIEN=0) -
// иначе содержимое сдвигового регистра/буферов «доедет» в новом формате.
void spi_set_frame_format_direct(bool is_16bit) {
    const uint32_t want = is_16bit ? M_SPI_FF16 : 0U;
    if ((MFL_SPI0_CTL0 & M_SPI_FF16) == want) return;   // уже в нужном режиме

    MFL_SPI0_CTL0 &= ~M_SPI_SPIEN;                       // SPI выключен
    __DSB();

    if (is_16bit) MFL_SPI0_CTL0 |= M_SPI_FF16;           // FF16 = 1 → 16-битный кадр
    else          MFL_SPI0_CTL0 &= ~M_SPI_FF16;          // FF16 = 0 → 8-битный кадр

    // Сброс буферов (errata fix)
    volatile uint32_t dummy = MFL_SPI0_STAT;
    dummy = MFL_SPI0_DATA;
    (void)dummy;
    __DSB();

    MFL_SPI0_CTL0 |= M_SPI_SPIEN;                        // SPI включен обратно
    __DSB();
}

// ==========================================================
// === ПИНЫ ===
// ==========================================================
void spi_cs_control_direct(bool select) {
    if (select) MFL_GPIOA_BC  = (1U << 4);
    else        MFL_GPIOA_BOP = (1U << 4);
}

void spi_dc_control_direct(bool data) {
    if (data) MFL_GPIOA_BOP = (1U << 6);
    else      MFL_GPIOA_BC  = (1U << 6);
}

// ==========================================================
// === ПЕРЕДАЧА ДАННЫХ ===
// ==========================================================

void spi_write_byte_direct(uint8_t data) {
    uint32_t guard = 0;
    while (!(MFL_SPI0_STAT & M_SPI_TBE))
        if (++guard > 1000000UL) return;   // защита от зависания
    MFL_SPI0_DATA = data;
}

void spi_write_word_direct(uint16_t data) {
    // 1. MSB
    uint32_t guard = 0;
    while (!(MFL_SPI0_STAT & M_SPI_TBE))
        if (++guard > 1000000UL) return;
    MFL_SPI0_DATA = (uint8_t)(data >> 8);

    // 2. LSB
    guard = 0;
    while (!(MFL_SPI0_STAT & M_SPI_TBE))
        if (++guard > 1000000UL) return;
    MFL_SPI0_DATA = (uint8_t)(data & 0xFF);
}

// Одиночный 16-битный кадр (требуется FF16=1, т.е. режим DATASIZE_16BIT).
// SPI отдаёт старшие биты вперёд.
void spi_write_frame16_direct(uint16_t data) {
    uint32_t guard = 0;
    while (!(MFL_SPI0_STAT & M_SPI_TBE))
        if (++guard > 1000000UL) return;
    MFL_SPI0_DATA = (uint32_t)data;
}

void spi_write_buf_direct(const uint8_t* buf, size_t len) {
    while (len--) {
        uint32_t guard = 0;
        while (!(MFL_SPI0_STAT & M_SPI_TBE))
            if (++guard > 1000000UL) return;
        MFL_SPI0_DATA = *buf++;
    }
}

void spi_wait_for_tx_complete_direct() {
    uint32_t guard = 0;
    while (!(MFL_SPI0_STAT & M_SPI_TBE))
        if (++guard > 1000000UL) return;
    guard = 0;
    while (MFL_SPI0_STAT & M_SPI_TRANS)
        if (++guard > 1000000UL) return;
}

// ==========================================================
// === DMA для SPI0 TX (DMA0 Channel 2 на GD32F303) ===
// ==========================================================
// По даташиту GD32F303 запрос SPI0_TX аппаратно подключён к DMA0 Channel 2
//
// Порядок, требуемый даташитом:
//   SPI:  FF16 -> CKPL/CKPH -> LF -> режимы -> SPIEN (последним!)
//   DMA:  убедиться что CHEN=0 -> CTL (M2M, DIR, CMEN, PRIO, ширины,
//         генерация адресов, прерывания) -> PADDR -> MADDR -> CNT -> CHEN=1
//   SPI:  DMATEN включается ПОСЛЕ того, как канал DMA настроен и включён.
//   Errata GD32: менять FF16 (и любой переключать тактирование SPI) можно
//   только при SPIEN=0, после переключения — сбросить буферы и включить SPI.
//
// ==========================================================

// ==========================================================
// === ПОДСВЕТКА ===
// ==========================================================
void tft_backlight_set(pin_size_t bl_pin, bool on) {
    if (bl_pin != NO_PIN) {
        pinMode(bl_pin, OUTPUT);
        digitalWrite(bl_pin, on ? HIGH : LOW);
    }
}

// ==========================================================
// === DMA: SPI0 TX через DMA0 CH2 ===
// ==========================================================
// Транзакция разделена на два шага, чтобы CS/DC можно было установить
// МЕЖДУ ними: как только DMATEN=1, SPI (TBE=1) сразу же запрашивает данные
// у DMA и первый кадр стартует мгновенно. Если к этому моменту CS ещё
// поднят, первый пиксель теряется и весь ряд сдвигается.
//
//   spi_dma_prepare()  — SPI переведён в 16-битный режим безопасно (errata),
//                        канал DMA сконфигурирован, но CHEN ещё = 0
//   ...вызов поднимает DC/CS...
//   spi_dma_trigger() — CHEN=1, затем DMATEN=1 (канал ждёт запрос первым)
//
// Watchdog: если счётчик канала не двигается дольше DMA_STALL_TIMEOUT_US,
// транзакция считается сорванной (иначе Marlin навсегда посчитал бы TFT занятым).
static uint32_t dma_last_count = 0;
static uint32_t dma_progress_us = 0;
static uint32_t dma_deadline_us = 0;

// Ожидание флага SPI с таймаутом (без него любой сбой превращается в вечный цикл)
static bool spi_wait_tbe() {
    const uint32_t t0 = micros();
    while (!(MFL_SPI0_STAT & M_SPI_TBE))
        if (micros() - t0 > DMA_SPI_FLAG_TIMEOUT_US) return false;
    return true;
}
static bool spi_wait_tx_idle() {
    const uint32_t t0 = micros();
    while (MFL_SPI0_STAT & M_SPI_TRANS)
        if (micros() - t0 > DMA_SPI_FLAG_TIMEOUT_US) return false;
    return true;
}

// Быстрая проверка: SPI гарантированно свободен (TBE=1, TRANS=0).
// Используется после spi_dma_is_busy()==false, когда TRANS уже проверен.
static inline bool spi_is_idle_fast() {
    return (MFL_SPI0_STAT & (M_SPI_TBE | M_SPI_TRANS)) == M_SPI_TBE;
}

// Сброс буферов SPI (чтение STAT + DATA) — errata GD32
static void spi_flush_buffers() {
    volatile uint32_t dummy = MFL_SPI0_STAT;
    dummy = MFL_SPI0_DATA;
    (void)dummy;
    __DSB();
}

static void dma_channel_halt() {
    MFL_SPI0_CTL1 &= ~M_SPI_DMATEN;      // SPI больше не просит данные
    MFL_DMA0_CH2_CTL &= ~M_DMA_CHEN;     // глушим канал
    MFL_DMA0_INTC = M_DMA_CH2_ALL;       // сброс GIF/FTFIF/HTFIF/ERRIF
    __DSB();
}

// Минимальная остановка: глушим DMA-канал и DMATEN, НЕ трогаем формат кадра.
// Вызывается когда spi_dma_is_busy() уже подтвердил TRANS=0 (SPI свободен).
// Экономит spi_wait_tbe + spi_wait_tx_idle + spi_restore_8bit (~50-80 циклов).
void spi_dma_halt_only() {
    // Safety net: если по какой-то причине SPI ещё не idle (аппаратный сбой),
    // дождёмся флагов. В нормальном пути spi_dma_is_busy() уже проверил TRANS=0,
    // поэтому spi_is_idle_fast() вернёт true за 1 цикл.
    if (!spi_is_idle_fast()) {
        spi_wait_tbe();
        spi_wait_tx_idle();
    }
    MFL_SPI0_CTL1 &= ~M_SPI_DMATEN;
    MFL_DMA0_CH2_CTL &= ~M_DMA_CHEN;
    MFL_DMA0_INTC = M_DMA_CH2_ALL;
    __DSB();
}

// Возврат SPI в 8-битный режим. Смена FF16 — только при SPIEN=0 (errata).
static void spi_restore_8bit() {
    MFL_SPI0_CTL0 &= ~M_SPI_SPIEN;
    __DSB();

    MFL_SPI0_CTL0 &= ~M_SPI_FF16;
    spi_flush_buffers();

    MFL_SPI0_CTL0 |= M_SPI_SPIEN;
    __DSB();
}

// Подготовка транзакции: SPI -> 16 бит, канал DMA сконфигурирован, CHEN=0.
// count_words = число 16-битных переносов (пикселей).
bool spi_dma_prepare(uint32_t mem_addr, uint32_t count_words, bool mem_increment) {
    // CNT[15:0]: 0 означает «нет транзакций», > 0xFFFF физически невозможен
    if (count_words == 0 || count_words > M_DMA_CNT_MAX) return false;
    // 16-битные переносы: адрес памяти должен быть выровнен (аппаратно
    // младшие биты отбрасываются, и данные тихо «сдвигаются»)
    if (mem_addr & 1U) return false;

    // 1. Останавливаем канал ТОЛЬКО если он активен (экономия ~10 циклов
    //    при последовательных DMA-транзакциях, когда канал уже погашен).
    if (MFL_DMA0_CH2_CTL & M_DMA_CHEN) {
        dma_channel_halt();
    }

    // 2. SPI disabling sequence: дождаться TBE=1, затем TRANS=0.
    //    При последовательных DMA (isBusy уже подтвердил idle) эти проверки
    //    проходят за 1-2 цикла (флаги уже установлены).
    if (!spi_wait_tbe()) return false;
    if (!spi_wait_tx_idle()) return false;

    // 3. Даташит: PADDR/MADDR/CNT/CTL нельзя писать, пока CHEN=1.
    //    Убеждаемся, что канал действительно выключен.
    if (MFL_DMA0_CH2_CTL & M_DMA_CHEN) {
        MFL_DMA0_CH2_CTL &= ~M_DMA_CHEN;
        uint32_t guard = 0;
        while ((MFL_DMA0_CH2_CTL & M_DMA_CHEN) && ++guard < 1000000UL) {}
        if (MFL_DMA0_CH2_CTL & M_DMA_CHEN) return false;   // канал не гаснет
    }
    MFL_DMA0_INTC = M_DMA_CH2_ALL;
    __DSB();

    // 4. Errata: FF16 меняется только при выключенном SPI, буферы сбрасываются.
    //    Если SPI УЖЕ в 16-битном режиме (последовательные DMA-транзакции) —
    //    пропускаем полный цикл SPI off→on, экономя ~40-60 циклов на строку.
    if (!(MFL_SPI0_CTL0 & M_SPI_FF16)) {
        MFL_SPI0_CTL0 &= ~M_SPI_SPIEN;
        __DSB();
        MFL_SPI0_CTL0 |=  M_SPI_FF16;     // 16-битный кадр
        MFL_SPI0_CTL0 &= ~M_SPI_LF;       // LF=0 -> MSB first, как требует ST7789
        spi_flush_buffers();
    }

    // 5. Конфигурирование канала (CHEN всё ещё 0)
    MFL_DMA0_CH2_PADDR = (uint32_t)(MFL_SPI0_BASE + 0x0CU);  // SPI0 DATA
    MFL_DMA0_CH2_MADDR = mem_addr;
    MFL_DMA0_CH2_CNT   = count_words;

    // M2M=0, CMEN=0, PNAGA=0 (SPI_DATA — постоянный адрес), прерывания
    // канала не используются (опрос флагов), поэтому FTFIE/HTFIE/ERRIE=0.
    uint32_t ctl = M_DMA_DIR_M2P | M_DMA_PWIDTH_16 | M_DMA_MWIDTH_16 | M_DMA_PRIO_HIGH;
    if (mem_increment) ctl |= M_DMA_MNAGA;
    MFL_DMA0_CH2_CTL = ctl;
    __DSB();

    // 6. SPI включается последним шагом своей инициализации
    MFL_SPI0_CTL0 |= M_SPI_SPIEN;
    __DSB();

    // 7. Настройка watchdog'а транзакции
    const uint32_t now = micros();
    dma_last_count = count_words;
    dma_progress_us = now;
    dma_deadline_us = now + DMA_TRANSFER_TIMEOUT_US;
    return true;
}

// Запуск переноса — вызывать сразу после того, как CS/DC установлены.
void spi_dma_trigger() {
    MFL_DMA0_CH2_CTL |= M_DMA_CHEN;   // канал ждёт запрос от SPI0_TX
    __DSB();
    MFL_SPI0_CTL1 |= M_SPI_DMATEN;    // только теперь SPI начнёт просить данные
    __DSB();
}


// Остановка транзакции. Порядок — «SPI disabling sequence» из даташита:
// снять DMA-запросы, дождаться TBE=1, затем TRANS=0, и только после этого
// выключать SPI (SPIEN=0) и менять формат кадра.
void spi_dma_stop() {
    // 1. SPI больше не генерирует запросы, канал выключен, флаги сняты
    dma_channel_halt();

    // 2. Досылаем то, что уже загружено в сдвиговый регистр
    spi_wait_tbe();
    spi_wait_tx_idle();

    // 3. Возврат в 8-битный формат — только на выключенном SPI (errata)
    spi_restore_8bit();
}

bool spi_dma_is_busy() {
    // Канал ещё передаёт данные в SPI DATA?
    if (MFL_DMA0_CH2_CTL & M_DMA_CHEN) {
        // Ошибка канала — транзакцию считаем завершённой, иначе UI встанет намертво
        if (MFL_DMA0_INTF & M_DMA_ERRIF2) {
            dma_channel_halt();
            return false;
        }

        if (MFL_DMA0_INTF & M_DMA_FTFIF2) {
            // Полный перенос завершён (в normal-режиме аппаратно сам снимает CHEN)
            MFL_DMA0_INTC = M_DMA_CH2_ALL;
            MFL_DMA0_CH2_CTL &= ~M_DMA_CHEN;
            __DSB();
        }
        else {
            const uint32_t count = MFL_DMA0_CH2_CNT;
            const uint32_t now = micros();

            if (count != dma_last_count) {                 // прогресс есть
                dma_last_count = count;
                dma_progress_us = now;
            }
            else if (now - dma_progress_us > DMA_STALL_TIMEOUT_US) {
                dma_channel_halt();                        // запросов от SPI нет
                return false;
            }

            if (now > dma_deadline_us) {                   // суммарный предел
                dma_channel_halt();
                return false;
            }

            return true;
        }
    }

    // DMA закончил, но SPI ещё сдвигает последний кадр
    if (MFL_SPI0_STAT & M_SPI_TRANS) return true;
    return false;
}

} // namespace hal_bridge

#endif // ARDUINO_ARCH_MFL