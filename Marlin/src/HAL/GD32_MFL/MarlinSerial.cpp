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

#include "../platforms.h"

#ifdef ARDUINO_ARCH_MFL

#include "../../inc/MarlinConfig.h"
#include "MarlinSerial.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
  // Храним состояние здесь, так как нельзя расширять размер класса при reinterpret_cast
  static EmergencyParser::State local_emergency_state = EmergencyParser::State::EP_RESET;
#endif

using namespace arduino;

auto MarlinSerial::get_instance(usart::USART_Base Base, pin_size_t rxPin, pin_size_t txPin) -> MarlinSerial& {
  auto& serial = UsartSerial::get_instance(Base, rxPin, txPin);
  // Безопасно, так как мы не добавили новых полей данных в класс MarlinSerial
  return *reinterpret_cast<MarlinSerial*>(&serial);
}

#if USING_HW_SERIAL0
  MSerialT MSerial0(true, MarlinSerial::get_instance(usart::USART_Base::USART0_BASE, NO_PIN, NO_PIN));
#endif
#if USING_HW_SERIAL1
  MSerialT MSerial1(true, MarlinSerial::get_instance(usart::USART_Base::USART1_BASE, NO_PIN, NO_PIN));
#endif
#if USING_HW_SERIAL2
  MSerialT MSerial2(true, MarlinSerial::get_instance(usart::USART_Base::USART2_BASE, NO_PIN, NO_PIN));
#endif
#if USING_HW_SERIAL3
  MSerialT MSerial3(true, MarlinSerial::get_instance(usart::USART_Base::UART3_BASE, NO_PIN, NO_PIN));
#endif
#if USING_HW_SERIAL4
  MSerialT MSerial4(true, MarlinSerial::get_instance(usart::USART_Base::UART4_BASE, NO_PIN, NO_PIN));
#endif

void MarlinSerial::begin(unsigned long baudrate, uint16_t config) {
  // Просто запускаем штатный UART
  UsartSerial::begin(baudrate, config, ENABLED(SERIAL_DMA));
  
  #if ENABLED(EMERGENCY_PARSER)
    // Сброс состояния при рестарте порта
    local_emergency_state = EmergencyParser::State::EP_RESET;
  #endif
}

// ГЛАВНОЕ ИСПРАВЛЕНИЕ:
// Читаем байт штатным методом. Если он есть — скармливаем копию парсеру и возвращаем байт Марлину.
// Никакие данные не теряются, прерывания не конфликтуют.
int MarlinSerial::read() {
  int c = UsartSerial::read();
  
  #if ENABLED(EMERGENCY_PARSER)
    if (c >= 0) {
      emergency_parser.update(local_emergency_state, (uint8_t)c);
    }
  #endif
  
  return c;
}

void MarlinSerial::updateRxDmaBuffer() {
  // Только штатная логика MFL по переброске данных из DMA в RingBuffer
  UsartSerial::updateRxDmaBuffer();
}

#endif // ARDUINO_ARCH_MFL
