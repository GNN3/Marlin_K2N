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

// ==========================================================
// === МАСКИ ===
// ==========================================================
#define M_SPI_CKPH    (1U << 0U)
#define M_SPI_CKPL    (1U << 1U)
#define M_SPI_MSTMOD  (1U << 2U)
#define M_SPI_PSC_8   (2U << 3U)
#define M_SPI_SPIEN   (1U << 6U)
#define M_SPI_SWNSS   (1U << 8U)
#define M_SPI_SWNSSEN (1U << 9U)
#define M_SPI_TBE     (1U << 1U)
#define M_SPI_TRANS   (1U << 7U)

// Bidirectional Mode
#define M_SPI_BDOEN   (1U << 14U) 
#define M_SPI_BDEN    (1U << 15U) 

namespace hal_bridge {

// ==========================================================
// === ИНИЦИАЛИЗАЦИЯ (ФИНАЛЬНЫЙ FIX СИНХРОНИЗАЦИИ) ===
// ==========================================================
bool spi_init_direct() {
    // 1. Clocks
    MFL_RCU_APB2EN |= (1U << 0U) | (1U << 2U) | (1U << 12U);
    
    // !!! КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ ЗАВИСАНИЯ !!!
    // Барьер синхронизации шины: читаем регистр обратно, чтобы убедиться,
    // что клок SPI/GPIO был активирован перед тем, как мы начнем писать в CTL0.
    // Это предотвращает зависание процессора на первой же записи в периферию.
    (void)MFL_RCU_APB2EN; 
    __asm__ __volatile__ ("nop"); 
    
    // 2. GPIO Config
    uint32_t temp_ctl0 = MFL_GPIOA_CTL0;
    temp_ctl0 &= 0x0000FFFFU; 
    
    // PA4(CS)=Out, PA5(SCK)=Alt, PA6(DC)=Out, PA7(MOSI)=Alt
    temp_ctl0 |= (0x3U << 16) | (0xBU << 20) | (0x3U << 24) | (0xBU << 28);
    
    MFL_GPIOA_CTL0 = temp_ctl0;
    MFL_GPIOA_BOP = (1U << 4) | (1U << 6); 
    
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
    spi_cfg |= M_SPI_PSC_8;          
    spi_cfg |= M_SPI_BDEN | M_SPI_BDOEN;
    
    MFL_SPI0_CTL0 = spi_cfg;
    
    __asm("nop"); __asm("nop");
    
    MFL_SPI0_CTL0 |= M_SPI_SPIEN;

    return true;
}

// ==========================================================
// === ФОРМАТ (ЗАГЛУШКА) ===
// ==========================================================
void spi_set_frame_format_direct(bool is_16bit) {
    (void)is_16bit; 
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
    while (!(MFL_SPI0_STAT & M_SPI_TBE));
    MFL_SPI0_DATA = data;
}

void spi_write_word_direct(uint16_t data) {
    // 1. MSB
    while (!(MFL_SPI0_STAT & M_SPI_TBE));
    MFL_SPI0_DATA = (uint8_t)(data >> 8);
    
    // 2. LSB
    while (!(MFL_SPI0_STAT & M_SPI_TBE));
    MFL_SPI0_DATA = (uint8_t)(data & 0xFF);
}

void spi_write_buf_direct(const uint8_t* buf, size_t len) {
    while (len--) {
        while (!(MFL_SPI0_STAT & M_SPI_TBE));
        MFL_SPI0_DATA = *buf++;
    }
}

void spi_wait_for_tx_complete_direct() {
    while (!(MFL_SPI0_STAT & M_SPI_TBE));
    while (MFL_SPI0_STAT & M_SPI_TRANS);
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

} // namespace hal_bridge

#endif // ARDUINO_ARCH_MFL