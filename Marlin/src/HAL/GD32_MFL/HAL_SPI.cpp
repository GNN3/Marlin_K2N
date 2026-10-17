// hal-bridge HAL_SPI.cpp — GD32_MFL
// SPI0 и DMA0(CH2) реализованы через SDK-классы MFL (spi::SPI, dma::DMA, gpio::GPIO)
// вместо прямого обращения к регистрам. Публичный API hal_bridge и поведение
// (state-machine, watchdog, errata, двухфазный DMA) сохранены полностью.
#include "../platforms.h"
#ifdef ARDUINO_ARCH_MFL

#include "HAL_SPI.h"
#include "../../inc/MarlinConfig.h"
#include "Arduino.h"

#include <SPI.hpp>     // spi::SPI
#include <DMA.hpp>     // dma::DMA
#include <GPIO.hpp>    // gpio::GPIO + fast_write_pin
#include <variant.h>   // port_pin_map, PortPinPair

// Делитель тактирования SPI0. При APB2 = 120 МГц: PCLK_4 = 30 МГц, PCLK_8 = 15 МГц.
#ifndef TFT_SPI_PCLK_DIVIDER
  #define TFT_SPI_PCLK_DIVIDER spi::PCLK_Divider::PCLK_4
#endif

// Watchdog: счётчик не двигается дольше этого времени -> запросов от SPI нет.
#define DMA_STALL_TIMEOUT_US      5000UL
#define DMA_TRANSFER_TIMEOUT_US   200000UL
#define DMA_SPI_FLAG_TIMEOUT_US   100000UL

// CNT[15:0] — 16-битный счётчик; ноль означает «транзакций нет».
#define M_DMA_CNT_MAX       0xFFFFU

namespace hal_bridge {

// ==========================================================
// === ЛЕНИВЫЕ ОДНОЗНАЧНЫЕ ПЕРИФИЕРИЙ (SDK MFL) ===
// ==========================================================
// Конструкторы spi::SPI / dma::DMA / gpio::GPIO сами включают тактирование
// через RCU, поэтому ручные записи в RCU_APB2EN / RCU_AHBEN больше не нужны.
static spi::SPI& SPIx() {
    static spi::SPI& s = spi::SPI::get_instance(spi::SPI_Base::SPI0_BASE).value();
    return s;
}
static dma::DMA& DMAx() {
    // GD32F303: запрос SPI0_TX аппаратно подключён к DMA0 Channel 2.
    static dma::DMA& d = dma::DMA::get_instance(dma::DMA_Base::DMA0_BASE,
                                                dma::DMA_Channel::CHANNEL2).value();
    return d;
}

// Биты для быстрой проверки состояния (значения взяты из enum-ов SDK).
static inline volatile uint32_t* dma_chxctl() { return DMAx().reg_address(dma::DMA_Regs::CH2CTL); }
static const uint32_t DMA_CHEN_BIT = 1U << static_cast<uint32_t>(dma::CHXCTL_Bits::CHEN);
static const uint32_t SPI_FF16_BIT = 1U << static_cast<uint32_t>(spi::CTL0_Bits::FF16);

// Forward declarations (внутренние помощники определены ниже).
static void spi_flush_buffers();
static void dma_channel_halt();
static bool spi_wait_tbe();
static bool spi_wait_tx_idle();
static inline bool spi_is_idle_fast();
static void spi_restore_8bit();

// ==========================================================
// === STATE MACHINE (guard layer) ===
// ==========================================================
static SpiState spi_state = SpiState::Idle;
SpiState spi_get_state() { return spi_state; }
void spi_set_state(SpiState s) { spi_state = s; }

// ==========================================================
// === ИНИЦИАЛИЗАЦИЯ ===
// ==========================================================
bool spi_init_direct() {
    const PortPinPair cs   = port_pin_map[TFT_CS_PIN];
    const PortPinPair dc   = port_pin_map[TFT_DC_PIN];
    const PortPinPair sck  = port_pin_map[TFT_SCK_PIN];
    const PortPinPair mosi = port_pin_map[TFT_MOSI_PIN];

    // 1. GPIO: CS=Out, DC=Out, SCK=AF, MOSI=AF (тактирование GPIO включает конструктор GPIO)
    gpio::GPIO::get_instance(cs.port).value().set_pin_mode(cs.pin,    gpio::Pin_Mode::OUTPUT_PUSHPULL);
    gpio::GPIO::get_instance(dc.port).value().set_pin_mode(dc.pin,    gpio::Pin_Mode::OUTPUT_PUSHPULL);
    gpio::GPIO::get_instance(sck.port).value().set_pin_mode(sck.pin,  gpio::Pin_Mode::ALT_PUSHPULL);
    gpio::GPIO::get_instance(mosi.port).value().set_pin_mode(mosi.pin, gpio::Pin_Mode::ALT_PUSHPULL);

    // Уровни покоя: CS High, DC = DATA (High)
    gpio::fast_write_pin(cs.port, cs.pin, true);
    gpio::fast_write_pin(dc.port, dc.pin, true);
    __DSB();

    // 2. SPI0: master bidirectional-TX (аналог BDEN|BDOEN), software NSS,
    //    режим 3 (CKPL=1, CKPH=1), MSB first, делитель по умолчанию.
    auto& spi = SPIx();
    spi.init(spi::SPI_Config{
        .operational_mode = spi::Operational_Mode::MTB_MODE,
        .frame_format     = spi::Frame_Format::FF_8BIT,
        .nss_type         = spi::NSS_Type::SOFTWARE_NSS,
        .pclk_divider     = TFT_SPI_PCLK_DIVIDER,
        .msbf             = spi::Endian_Type::MSBF,
        .polarity_pull    = spi::Clock_Polarity::PULL_HIGH,
        .clock_phase      = spi::Clock_Phase::PHASE_SECOND_EDGE,
    });
    spi.nss_internal_high();   // SWNSS=1 (не выбран) — паритет с прежней spi_cfg
    spi.set_enable(true);
    __DSB();
    return true;
}

// ==========================================================
// === ФОРМАТ (errata: FF16 менять только при SPIEN=0) ===
// ==========================================================
void spi_set_frame_format_direct(bool is_16bit) {
    auto& spi = SPIx();
    volatile uint32_t* ctl0 = spi.reg_address(spi::SPI_Regs::CTL0);
    const uint32_t want = is_16bit ? SPI_FF16_BIT : 0U;
    if ((*ctl0 & SPI_FF16_BIT) == want) return;   // уже в нужном режиме

    spi.set_enable(false);                         // SPI выключен
    __DSB();
    spi.data_frame_format_config(is_16bit ? spi::Frame_Format::FF_16BIT
                                          : spi::Frame_Format::FF_8BIT);
    spi_flush_buffers();                           // errata fix: сброс буферов
    __DSB();
    spi.set_enable(true);                          // SPI включён обратно
    __DSB();
}

// ==========================================================
// === ПИНЫ (CS/DC через port_pin_map платы) ===
// ==========================================================
void spi_cs_control_direct(bool select) {
    const PortPinPair& cs = port_pin_map[TFT_CS_PIN];
    gpio::fast_write_pin(cs.port, cs.pin, !select);   // select=true -> CS Low
}
void spi_dc_control_direct(bool data) {
    const PortPinPair& dc = port_pin_map[TFT_DC_PIN];
    gpio::fast_write_pin(dc.port, dc.pin, data);      // data=true -> DC High
}

// ==========================================================
// === ПЕРЕДАЧА ДАННЫХ ===
// ==========================================================
void spi_write_byte_direct(uint8_t data) {
    auto& spi = SPIx();
    uint32_t guard = 0;
    while (!spi.get_flag(spi::Status_Flags::FLAG_TBE))
        if (++guard > 1000000UL) return;
    spi.data_transmit(data);
}
void spi_write_word_direct(uint16_t data) {
    auto& spi = SPIx();
    uint32_t guard = 0;
    while (!spi.get_flag(spi::Status_Flags::FLAG_TBE)) if (++guard > 1000000UL) return;
    spi.data_transmit((uint16_t)(data >> 8));       // MSB
    guard = 0;
    while (!spi.get_flag(spi::Status_Flags::FLAG_TBE)) if (++guard > 1000000UL) return;
    spi.data_transmit((uint16_t)(data & 0xFF));     // LSB
}
void spi_write_frame16_direct(uint16_t data) {
    auto& spi = SPIx();
    uint32_t guard = 0;
    while (!spi.get_flag(spi::Status_Flags::FLAG_TBE)) if (++guard > 1000000UL) return;
    spi.data_transmit(data);
}
void spi_write_buf_direct(const uint8_t* buf, size_t len) {
    auto& spi = SPIx();
    while (len--) {
        uint32_t guard = 0;
        while (!spi.get_flag(spi::Status_Flags::FLAG_TBE)) if (++guard > 1000000UL) return;
        spi.data_transmit(*buf++);
    }
}
void spi_wait_for_tx_complete_direct() {
    auto& spi = SPIx();
    uint32_t guard = 0;
    while (!spi.get_flag(spi::Status_Flags::FLAG_TBE)) if (++guard > 1000000UL) return;
    guard = 0;
    while (spi.get_flag(spi::Status_Flags::FLAG_TRANS)) if (++guard > 1000000UL) return;
}

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
// === ВСПОМОГАТЕЛЬНЫЕ (SPI idle / flush / halt) ===
// ==========================================================
static void spi_flush_buffers() {
    auto& spi = SPIx();
    volatile uint32_t dummy = *spi.reg_address(spi::SPI_Regs::STAT);
    dummy = spi.data_receive();
    (void)dummy;
}
static bool spi_wait_tbe() {
    auto& spi = SPIx();
    if (spi.get_flag(spi::Status_Flags::FLAG_TBE)) return true;          // fast-path
    const uint32_t t0 = micros();
    while (!spi.get_flag(spi::Status_Flags::FLAG_TBE))
        if (micros() - t0 > DMA_SPI_FLAG_TIMEOUT_US) return false;
    return true;
}
static bool spi_wait_tx_idle() {
    auto& spi = SPIx();
    if (!spi.get_flag(spi::Status_Flags::FLAG_TRANS)) return true;       // fast-path
    const uint32_t t0 = micros();
    while (spi.get_flag(spi::Status_Flags::FLAG_TRANS))
        if (micros() - t0 > DMA_SPI_FLAG_TIMEOUT_US) return false;
    return true;
}
static inline bool spi_is_idle_fast() {
    auto& spi = SPIx();
    return spi.get_flag(spi::Status_Flags::FLAG_TBE) && !spi.get_flag(spi::Status_Flags::FLAG_TRANS);
}
static void dma_channel_halt() {
    auto& spi = SPIx(); auto& dma = DMAx();
    spi.set_dma_enable(spi::DMA_Direction::DMA_TX, false);   // SPI больше не просит данные
    dma.set_channel_enable(false);                           // глушим канал
    dma.clear_flag(dma::Status_Flags::FLAG_GIF);
    dma.clear_flag(dma::Status_Flags::FLAG_FTFIF);
    dma.clear_flag(dma::Status_Flags::FLAG_HTFIF);
    dma.clear_flag(dma::Status_Flags::FLAG_ERRIF);
    __DSB();
}
static void spi_restore_8bit() {
    auto& spi = SPIx();
    spi.set_enable(false); __DSB();
    spi.data_frame_format_config(spi::Frame_Format::FF_8BIT);
    spi_flush_buffers();
    spi.set_enable(true); __DSB();
}

// Watchdog-состояние транзакции
static uint32_t dma_last_count = 0;
static uint32_t dma_progress_us = 0;
static uint32_t dma_start_us = 0;

// ==========================================================
// === DMA: SPI0 TX через DMA0 CH2 (двухфазно) ===
// ==========================================================
bool spi_dma_prepare(uint32_t mem_addr, uint32_t count_words, bool mem_increment) {
    if (count_words == 0 || count_words > M_DMA_CNT_MAX) return false;
    if (mem_addr & 1U) return false;   // 16-битные переносы: адрес выровнен

    auto& spi = SPIx(); auto& dma = DMAx();
    volatile uint32_t* chctl = dma_chxctl();
    spi_state = SpiState::DmaPrepared;

    // 1. Останавливаем канал ТОЛЬКО если он активен
    if (*chctl & DMA_CHEN_BIT) dma_channel_halt();

    // 2. SPI disabling sequence: TBE=1, затем TRANS=0
    if (!spi_wait_tbe()) return false;
    if (!spi_wait_tx_idle()) return false;

    // 3. Убеждаемся, что CHEN=0 (PADDR/MADDR/CNT нельзя писать при CHEN=1)
    if (*chctl & DMA_CHEN_BIT) {
        dma.set_channel_enable(false);
        uint32_t guard = 0;
        while ((*chctl & DMA_CHEN_BIT) && ++guard < 1000000UL) {}
        if (*chctl & DMA_CHEN_BIT) return false;
    }
    dma.clear_flag(dma::Status_Flags::FLAG_GIF);
    dma.clear_flag(dma::Status_Flags::FLAG_FTFIF);
    dma.clear_flag(dma::Status_Flags::FLAG_HTFIF);
    dma.clear_flag(dma::Status_Flags::FLAG_ERRIF);
    __DSB();

    // 4. Errata: FF16 только при выключенном SPI; буферы сбрасываются.
    //    Если уже 16-битный режим (последовательные DMA) — пропускаем цикл.
    volatile uint32_t* ctl0 = spi.reg_address(spi::SPI_Regs::CTL0);
    if (!(*ctl0 & SPI_FF16_BIT)) {
        spi.set_enable(false); __DSB();
        spi.data_frame_format_config(spi::Frame_Format::FF_16BIT);  // LF=0 (MSB) уже в конфиге
        spi_flush_buffers();
    }

    // 5. Конфигурирование канала (init() сам держит CHEN=0)
    dma.init(dma::DMA_Config{
        .count              = count_words,
        .memory_address     = mem_addr,
        .peripheral_address = (uint32_t)(uintptr_t)spi.reg_address(spi::SPI_Regs::DATA),
        .peripheral_bit_width = dma::Bit_Width::WIDTH_16BIT,
        .memory_bit_width     = dma::Bit_Width::WIDTH_16BIT,
        .peripheral_increase  = dma::Increase_Mode::INCREASE_DISABLE,
        .memory_increase      = mem_increment ? dma::Increase_Mode::INCREASE_ENABLE
                                             : dma::Increase_Mode::INCREASE_DISABLE,
        .channel_priority     = dma::Channel_Priority::HIGH_PRIORITY,
        .direction            = dma::Transfer_Direction::M2P,
        .circular_mode        = false,
        .memory_to_memory     = false,
    });
    // Прерывания канала не используются (опрос флагов) — гарантированно выключены.
    dma.set_interrupt_enable(dma::Interrupt_Type::INTR_FTFIE, false);
    dma.set_interrupt_enable(dma::Interrupt_Type::INTR_HTFIE, false);
    dma.set_interrupt_enable(dma::Interrupt_Type::INTR_ERRIE, false);
    __DSB();

    // 6. SPI включается последним шагом (no-op, если уже включён)
    spi.set_enable(true); __DSB();

    // 7. Watchdog
    const uint32_t now = micros();
    dma_last_count = count_words;
    dma_progress_us = now;
    dma_start_us = now;
    return true;
}


void spi_dma_trigger() {
    auto& spi = SPIx(); auto& dma = DMAx();
    spi_state = SpiState::DmaActive;
    dma.set_channel_enable(true); __DSB();                 // канал ждёт запрос от SPI0_TX
    spi.set_dma_enable(spi::DMA_Direction::DMA_TX, true); __DSB();
}

void spi_dma_stop() {
    spi_state = SpiState::Idle;
    dma_channel_halt();
    spi_wait_tbe();
    spi_wait_tx_idle();
    spi_restore_8bit();
}

void spi_dma_halt_only() {
    auto& spi = SPIx(); auto& dma = DMAx();
    spi_state = SpiState::Idle;
    if (!spi_is_idle_fast()) { spi_wait_tbe(); spi_wait_tx_idle(); }
    spi.set_dma_enable(spi::DMA_Direction::DMA_TX, false);
    dma.set_channel_enable(false);
    dma.clear_flag(dma::Status_Flags::FLAG_GIF);
    dma.clear_flag(dma::Status_Flags::FLAG_FTFIF);
    dma.clear_flag(dma::Status_Flags::FLAG_HTFIF);
    dma.clear_flag(dma::Status_Flags::FLAG_ERRIF);
    __DSB();
}

DmaStatus spi_dma_poll() {
    auto& spi = SPIx(); auto& dma = DMAx();
    volatile uint32_t* chctl = dma_chxctl();

    if (*chctl & DMA_CHEN_BIT) {
        if (dma.get_flag(dma::Status_Flags::FLAG_ERRIF)) {
            dma_channel_halt();
            spi_state = SpiState::Error;
            return DmaStatus::ERROR;
        }
        if (dma.get_flag(dma::Status_Flags::FLAG_FTFIF)) {
            dma.clear_flag(dma::Status_Flags::FLAG_GIF);
            dma.clear_flag(dma::Status_Flags::FLAG_FTFIF);
            dma.clear_flag(dma::Status_Flags::FLAG_HTFIF);
            dma.clear_flag(dma::Status_Flags::FLAG_ERRIF);
            dma.set_channel_enable(false);
            __DSB();
        }
        else {
            const uint32_t count = dma.get_transfer_count();
            const uint32_t now = micros();
            if (count != dma_last_count) {                 // прогресс есть
                dma_last_count = count;
                dma_progress_us = now;
            }
            else if (now - dma_progress_us > DMA_STALL_TIMEOUT_US) {
                dma_channel_halt();                        // запросов от SPI нет
                return DmaStatus::ABORT;
            }
            if (now - dma_start_us > DMA_TRANSFER_TIMEOUT_US) {  // суммарный предел
                dma_channel_halt();
                return DmaStatus::ABORT;
            }
            return DmaStatus::BUSY;
        }
    }

    // DMA закончил, но SPI ещё сдвигает последний кадр
    if (spi.get_flag(spi::Status_Flags::FLAG_TRANS)) return DmaStatus::BUSY;
    return DmaStatus::DONE;
}

} // namespace hal_bridge

#endif // ARDUINO_ARCH_MFL

