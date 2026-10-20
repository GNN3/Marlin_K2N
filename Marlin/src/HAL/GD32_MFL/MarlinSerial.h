/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

#include <UsartSerial.hpp>

#include "../../core/serial_hook.h"

#define SERIAL_INDEX_MIN 0
#define SERIAL_INDEX_MAX 4

// serial_ports.h выполняет #undef SERIAL_INDEX_MIN/MAX в конце, поэтому
// количество портов вычисляется здесь и дальше используется как constexpr.
namespace gd32_mfl_serial { inline constexpr uint8_t count = SERIAL_INDEX_MAX - SERIAL_INDEX_MIN + 1; }
#define GD32_MFL_SERIAL_COUNT gd32_mfl_serial::count

#include "../shared/serial_ports.h"

#if defined(LCD_SERIAL_PORT) && ANY(HAS_DGUS_LCD, EXTENSIBLE_UI)
  #define LCD_SERIAL_TX_BUFFER_FREE() LCD_SERIAL.availableForWrite()
#endif

// Буферы RX/TX задаёт ядро MFL: 128 и 64 байта (UsartSerial.hpp).
// Marlin-настройки RX_BUFFER_SIZE / TX_BUFFER_SIZE на этой платформе не
// применяются, а переопределять -DUSART_RX_BUFFER_SIZE нельзя: ядро вычисляет
// head DMA-приёма по жёстко зашитому USART_RX_BUFFER_SIZE = 128.

// Приоритет прерываний U(S)ART. Ядро выставляет 1 (UsartSerial.cpp), что выше
// приоритета шагового ISR (STEP_TIMER_IRQ_PRIORITY = 2 в HAL/GD32_MFL/timers.cpp).
// TX обслуживается по TBEIE на каждый байт, поэтому более высокий приоритет UART
// вытесняет генерацию шагов и даёт джитр шагов. Держим UART ниже шагового ISR.
#ifndef UART_IRQ_PRIORITY
  #define UART_IRQ_PRIORITY 3
#endif

using namespace arduino;

struct MarlinSerial : public UsartSerial {
  // Фабрика (статический метод создания/получения инстанса)
  static auto get_instance(usart::USART_Base Base, pin_size_t rxPin = NO_PIN, pin_size_t txPin = NO_PIN) -> MarlinSerial&;

  // Инициализация
  void begin(unsigned long baudrate, uint16_t config);
  inline void begin(unsigned long baudrate) { begin(baudrate, SERIAL_8N1); }

  // Перехватчик для режима DMA (Spy Mode)
  // Обязательно override, так как это виртуальный метод базового класса
  void updateRxDmaBuffer() override;

  // Обработчики для режима IRQ (No DMA).
  // Должны быть public, чтобы статические функции-трамплины в .cpp могли их вызвать
  #if DISABLED(SERIAL_DMA)
    // Восстановление после ORERR / NERR / FERR. Без него драйвер навсегда
    // остаётся в состоянии ERROR и связь с хостом теряется.
    void handle_usart_error();

    #if ENABLED(EMERGENCY_PARSER)
      void emergency_isr();
    #endif
  #endif

  // Заглушка, требуемая ядром Marlin (M111, DGUS). Определяется всегда,
  // иначе сборка ломается при SERIAL_STATS_RX_BUFFER_OVERRUNS / SERIAL_DMA.
  FORCE_INLINE static uint8_t buffer_overruns() { return 0; }

protected:
  // Наследуем конструкторы UsartSerial
  using UsartSerial::UsartSerial;
};

// Объявления глобальных объектов (чтобы линковщик их видел)
typedef Serial1Class<MarlinSerial> MSerialT;
extern MSerialT MSerial0;
extern MSerialT MSerial1;
extern MSerialT MSerial2;
extern MSerialT MSerial3;
extern MSerialT MSerial4;