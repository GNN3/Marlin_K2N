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

namespace hal_bridge {

    // Инициализация
    bool spi_init_direct();
    
    // Передача
    void spi_write_byte_direct(uint8_t data);
    void spi_write_word_direct(uint16_t data);
    void spi_write_buf_direct(const uint8_t* buf, size_t len);
    
    // Пины
    void spi_cs_control_direct(bool select);
    void spi_dc_control_direct(bool data);
    
    // Настройки
    // (Функция оставлена для совместимости, но внутри будет пустышкой или безопасной)
    void spi_set_frame_format_direct(bool is_16bit);
    void spi_wait_for_tx_complete_direct();

    // Подсветка
    void tft_backlight_set(pin_size_t bl_pin, bool on);

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