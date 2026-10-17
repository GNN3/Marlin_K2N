#pragma once

#include <cstdint>
#include <cstddef>
#include "../../inc/MarlinConfig.h"
#include "Arduino.h"

#define DATASIZE_8BIT  0
#define DATASIZE_16BIT 1

#ifndef NO_PIN
  #define NO_PIN ((pin_size_t)(-1))
#endif

// Max pixels per single DMA transfer (DMA counter register is 16-bit).
// Вариант платы может задать своё значение — тогда используем его.
#ifndef DMA_MAX_WORDS
  #define DMA_MAX_WORDS  0x7FFFU
#endif

namespace hal_bridge {

    // Initialization
    bool spi_init_direct();

    // Transfer
    void spi_write_byte_direct(uint8_t data);
    void spi_write_word_direct(uint16_t data);          // 2 байта в 8-битном режиме (MSB first)
    void spi_write_frame16_direct(uint16_t data);       // 1 кадр в 16-битном режиме (FF16=1)
    void spi_write_buf_direct(const uint8_t* buf, size_t len);

    // Pins
    void spi_cs_control_direct(bool select);
    void spi_dc_control_direct(bool data);

    // Settings
    void spi_set_frame_format_direct(bool is_16bit);
    void spi_wait_for_tx_complete_direct();

    // Backlight
    void tft_backlight_set(pin_size_t bl_pin, bool on);

    // --- DMA for SPI0 TX (DMA0 Channel 2 on GD32F303 = SPI0_TX request) ---
    // count_words = number of 16-bit transfers (pixels)
    // 1) spi_dma_prepare() — SPI -> 16-bit frame, DMA channel configured, CHEN=0
    // 2) caller asserts DC/CS
    // 3) spi_dma_trigger() — CHEN=1, then SPI DMATEN=1

    enum class DmaStatus : uint8_t {
        BUSY,       // Транзакция ещё идёт
        DONE,       // Полный перенос завершён (FTFIF)
        ERROR,      // Аппаратная ошибка DMA (ERRIF) — данные неполные
        ABORT,      // Stall / deadline — данные неполные
    };

    // --- State machine (guard layer, NOT a replacement for hardware reads) ---
    // Tracks the EXPECTED protocol state. Hardware flags remain authoritative
    // for "is DMA done?" decisions. The state machine catches programming errors
    // (e.g., trigger() without prepare(), double-trigger, wrong teardown path).
    enum class SpiState : uint8_t {
        Idle,           // No transaction. CS high. SPI in 8-bit mode.
        Tx8,            // Software 8-bit transfer. CS low. CPU writing bytes.
        Tx16,           // Software 16-bit transfer. CS low. CPU writing frames.
        DmaPrepared,    // DMA configured (CHEN=0, DMATEN=0). CS/DC not yet set.
        DmaActive,      // DMA transferring (CHEN=1, DMATEN=1). CS low.
        Error,          // Unrecoverable fault. Requires re-init.
    };

    bool spi_dma_prepare(uint32_t mem_addr, uint32_t count_words, bool mem_increment);
    void spi_dma_trigger();
    void spi_dma_stop();
    void spi_dma_halt_only();   // Минимальная остановка без смены формата кадра
    DmaStatus spi_dma_poll();   // Опрос состояния (заменяет spi_dma_is_busy)

    // State machine accessors (for TFT_SPI layer guard checks)
    SpiState spi_get_state();
    void spi_set_state(SpiState s);

    // --- Wrappers ---
    inline bool spi_init(pin_size_t sck_pin, pin_size_t mosi_pin, pin_size_t miso_pin = NO_PIN) { 
        return spi_init_direct(); 
    }
    inline void spi_write_byte(uint8_t out) { 
        spi_write_byte_direct(out); 
    }
    inline void spi_cs_control(bool select) { 
        spi_cs_control_direct(select); 
    }
    inline void spi_dc_control(bool data) { 
        spi_dc_control_direct(data); 
    }
    inline void spi_wait_for_tx_complete() { 
        spi_wait_for_tx_complete_direct(); 
    }
    inline void spi_write_word(uint16_t data) {
        spi_write_word_direct(data);
    }

} // namespace hal_bridge