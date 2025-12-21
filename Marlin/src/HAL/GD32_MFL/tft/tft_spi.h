// src/HAL/GD32_MFL/tft_spi.h
#pragma once

#include "../../../inc/MarlinConfig.h"
#include "../../platforms.h"
#ifdef ARDUINO_ARCH_MFL
#include "../HAL_SPI.h"

#ifdef TFT_GENERIC

class TFT_SPI {
    static bool tft_busy;

public:
    // Инициализация и состояние
    static void init();
    static void set_busy(bool state);
    static bool get_busy();
    static bool isBusy();
    static void abort();
    
    // Идентификация и управление
    static uint32_t getID();
    static void backlight(bool on);
    
    // Геометрия
    static void setDisplayAddress(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    static void setWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    static void clearScreen(uint16_t color);
    
    // Управление транзакцией
    static void dataTransferBegin(uint16_t dataWidth = 16);
    static void dataTransferEnd();
    
    // Низкоуровневая запись
    // writeReg: uint8_t, так как команды дисплея всегда 8-битные
    static void writeReg(uint16_t reg);
    
    // writeData: uint16_t, чтобы принимать цвета и координаты (MSB->LSB)
    // и чтобы компилятор не ругался на переполнение при 0xFFFF
    static void writeData(uint16_t data);
    
    // Парсер скриптов инициализации
    static void commandList(const uint16_t *list);
    
    // Массовая передача (счетчики uint32_t для экранов > 256x256)
    static void writeSequence(const uint16_t *data, uint32_t count);
    static void writeMultiple(uint16_t color, uint32_t count);
    
    // DMA-совместимые обертки (в нашей реализации вызывают обычные методы)
    static void writeSequence_DMA(uint16_t *data, uint32_t count);
    static void writeMultiple_DMA(uint16_t color, uint32_t count);
};
typedef TFT_SPI TFT_IO_DRIVER;
#endif // TFT_GENERIC
#endif // ARDUINO_ARCH_MFL

