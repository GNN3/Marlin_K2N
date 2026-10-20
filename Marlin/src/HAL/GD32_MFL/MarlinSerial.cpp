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

// Количество Marlin-портов должно совпадать с числом U(S)ART-блоков ядра
static_assert(GD32_MFL_SERIAL_COUNT == static_cast<int>(static_cast<size_t>(usart::USART_Base::INVALID)),
              "Serial port count does not match the number of U(S)ART blocks in the core.");

// ---------------------------------------------------------------------------
// Индекс Marlin-порта (0..4) для каждого U(S)ART-блока.
// Вычисляется ОДИН раз в begin(), поэтому горячий путь (available/read/peek,
// т.е. каждый принятый байт) делает только O(1) обращение к таблице, а не
// switch на каждый байт. Неинициализированные ячейки = -1 -> 0xFF -> ранний выход.
// ---------------------------------------------------------------------------
static int8_t port_index_of_base[static_cast<size_t>(usart::USART_Base::INVALID)] = { -1, -1, -1, -1, -1 };

static constexpr int8_t compute_serial_index(usart::USART_Base base) {
  // Компилятор сводит это к таблице переходов; вызывается один раз на порт
  switch (base) {
    case usart::USART_Base::USART0_BASE: return 0;
    case usart::USART_Base::USART1_BASE: return 1;
    case usart::USART_Base::USART2_BASE: return 2;
    case usart::USART_Base::UART3_BASE:  return 3;
    case usart::USART_Base::UART4_BASE:  return 4;
    default:                             return -1;   // нераспознанный базовый адрес
  }
}

static inline uint8_t get_serial_index(usart::USART_Base base) {
  const size_t i = static_cast<size_t>(base);
  if (i >= GD32_MFL_SERIAL_COUNT) return 0xFF;
  const int8_t idx = port_index_of_base[i];
  return idx < 0 ? 0xFF : static_cast<uint8_t>(idx);
}

static inline IRQn_Type usart_irqn_of(usart::USART_Base base) {
  switch (base) {
    case usart::USART_Base::USART0_BASE: return USART0_IRQn;
    case usart::USART_Base::USART1_BASE: return USART1_IRQn;
    case usart::USART_Base::USART2_BASE: return USART2_IRQn;
    case usart::USART_Base::UART3_BASE:  return UART3_IRQn;
    case usart::USART_Base::UART4_BASE:  return UART4_IRQn;
    default:                             return INVALID_IRQ;
  }
}

// ---------------------------------------------------------------------------
// EMERGENCY_PARSER
// ---------------------------------------------------------------------------
#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"

  // Состояния парсеров для каждого порта (0..4)
  static EmergencyParser::State e_states[GD32_MFL_SERIAL_COUNT];

  // Режим DMA: «теневая» позиция, до которой EMERGENCY_PARSER уже просмотрен
  #if ENABLED(SERIAL_DMA)
    static size_t dma_shadow_heads[GD32_MFL_SERIAL_COUNT];
  #endif
#endif

// ---------------------------------------------------------------------------
// Режим IRQ (без DMA): трамплины прерываний
// ---------------------------------------------------------------------------
#if DISABLED(SERIAL_DMA)
  // Указатели на экземпляры нужны и EMERGENCY_PARSER, и обработчику ошибок
  static MarlinSerial* instances[GD32_MFL_SERIAL_COUNT];

  #if ENABLED(EMERGENCY_PARSER)
    static void emergency_isr_0() { if (instances[0]) instances[0]->emergency_isr(); }
    static void emergency_isr_1() { if (instances[1]) instances[1]->emergency_isr(); }
    static void emergency_isr_2() { if (instances[2]) instances[2]->emergency_isr(); }
    static void emergency_isr_3() { if (instances[3]) instances[3]->emergency_isr(); }
    static void emergency_isr_4() { if (instances[4]) instances[4]->emergency_isr(); }

    static constexpr void (*isr_handlers[GD32_MFL_SERIAL_COUNT])() = {
      emergency_isr_0, emergency_isr_1, emergency_isr_2,
      emergency_isr_3, emergency_isr_4
    };
  #endif

  static void usart_error_0() { if (instances[0]) instances[0]->handle_usart_error(); }
  static void usart_error_1() { if (instances[1]) instances[1]->handle_usart_error(); }
  static void usart_error_2() { if (instances[2]) instances[2]->handle_usart_error(); }
  static void usart_error_3() { if (instances[3]) instances[3]->handle_usart_error(); }
  static void usart_error_4() { if (instances[4]) instances[4]->handle_usart_error(); }

  static constexpr void (*err_handlers[GD32_MFL_SERIAL_COUNT])() = {
    usart_error_0, usart_error_1, usart_error_2,
    usart_error_3, usart_error_4
  };
#endif

using namespace arduino;

// Фабрика - простое приведение типа
MarlinSerial& MarlinSerial::get_instance(usart::USART_Base Base, pin_size_t rxPin, pin_size_t txPin) {
  return static_cast<MarlinSerial&>(UsartSerial::get_instance(Base, rxPin, txPin));
}

// Инициализация портов
#if USING_HW_SERIAL0
  MSerialT MSerial0(true, MarlinSerial::get_instance(usart::USART_Base::USART0_BASE, NO_PIN, NO_PIN));
  arduino::UsartSerial& Serial = MSerial0;
#endif

#if USING_HW_SERIAL1
  MSerialT MSerial1(true, MarlinSerial::get_instance(usart::USART_Base::USART1_BASE, NO_PIN, NO_PIN));
  arduino::UsartSerial& Serial1 = MSerial1;
#endif

#if USING_HW_SERIAL2
  MSerialT MSerial2(true, MarlinSerial::get_instance(usart::USART_Base::USART2_BASE, NO_PIN, NO_PIN));
  arduino::UsartSerial& Serial2 = MSerial2;
#endif

#if USING_HW_SERIAL3
  MSerialT MSerial3(true, MarlinSerial::get_instance(usart::USART_Base::UART3_BASE, NO_PIN, NO_PIN));
  arduino::UsartSerial& Serial3 = MSerial3;
#endif

#if USING_HW_SERIAL4
  MSerialT MSerial4(true, MarlinSerial::get_instance(usart::USART_Base::UART4_BASE, NO_PIN, NO_PIN));
  arduino::UsartSerial& Serial4 = MSerial4;
#endif

// Инициализация порта
void MarlinSerial::begin(unsigned long baudrate, uint16_t config) {
  // Инициализируем базовый драйвер
  UsartSerial::begin(baudrate, config, ENABLED(SERIAL_DMA));

  const usart::USART_Base base = usart_.get_base();

  // Индекс порта вычисляется один раз и дальше используется как таблица
  const size_t bi = static_cast<size_t>(base);
  if (bi < GD32_MFL_SERIAL_COUNT) port_index_of_base[bi] = compute_serial_index(base);

  const uint8_t idx = get_serial_index(base);
  if (idx == 0xFF) return;

  // Ядро выставляет приоритет 1, т.е. ВЫШЕ шагового ISR (STEP_TIMER = 2).
  // TX обслуживается прерыванием на каждый байт и вытесняет генерацию шагов,
  // поэтому опускаем UART ниже шагового прерывания.
  const IRQn_Type irq = usart_irqn_of(base);
  if (irq != INVALID_IRQ) NVIC_SetPriority(irq, UART_IRQ_PRIORITY);

  #if ENABLED(EMERGENCY_PARSER)
    e_states[idx] = EmergencyParser::State::EP_RESET;
  #endif

  #if DISABLED(SERIAL_DMA)
    instances[idx] = this;

    // Без обработки ошибок драйвер навсегда остаётся в состоянии ERROR:
    // DATA не читается, RBNE не сбрасывается, прерывание зацикливается,
    // а available() возвращает 0 -> связь с хостом теряется до перезагрузки.
    usart_.register_interrupt_callback(usart::Interrupt_Type::INTR_ERRIE, err_handlers[idx]);

    #if ENABLED(EMERGENCY_PARSER)
      usart_.register_interrupt_callback(usart::Interrupt_Type::INTR_RBNEIE, isr_handlers[idx]);
    #endif
  #endif
}

// Обновление буфера для DMA режима
void MarlinSerial::updateRxDmaBuffer() {
  // Обновляем данные из DMA
  UsartSerial::updateRxDmaBuffer();

  #if ENABLED(EMERGENCY_PARSER) && ENABLED(SERIAL_DMA)
    const uint8_t idx = get_serial_index(usart_.get_base());
    if (idx == 0xFF) return;

    auto& ring = usart_.get_rx_buffer();
    const size_t capacity = ring.capacity();

    // Быстрый выход если нет данных
    if (capacity == 0) return;

    // Ядро вычисляет head как (размер буфера - CNDTR). В момент, когда круговой
    // DMA дописал цикл (CNDTR == 0), head == capacity, что выходит за пределы
    // индексов буфера и даёт до capacity «мусорных» байт для чтения.
    size_t current_head = ring.getHead();
    if (current_head >= capacity) {
      current_head %= capacity;
      usart_.modify_rx_buffer_head(static_cast<uint8_t>(current_head));
    }

    const uint8_t* buffer = ring.data();
    size_t& shadow = dma_shadow_heads[idx];
    if (shadow >= capacity) shadow = 0;

    // Ровно то количество байт, которое DMA дописал с прошлого опроса,
    // и никогда больше размера буфера.
    size_t pending = (current_head + capacity - shadow) % capacity;
    if (pending > capacity) pending = capacity;

    while (pending--) {
      emergency_parser.update(e_states[idx], buffer[shadow]);
      if (++shadow == capacity) shadow = 0;
    }
  #endif
}

// Обработчик прерывания для IRQ режима
#if DISABLED(SERIAL_DMA)

  void MarlinSerial::emergency_isr() {
    #if ENABLED(EMERGENCY_PARSER)
      const uint8_t idx = get_serial_index(usart_.get_base());
      if (idx == 0xFF) return;

      // Байт уже прочитан ядром (USART::handle_rx_interrupt) и лежит в rx-буфере.
      // Повторное чтение DATA при RBNE = 0 даёт неопределённое/дублирующее
      // значение, из-за чего конечный автомат не распознавал M112 / M410.
      emergency_parser.update(e_states[idx], usart_.get_last_data());
    #endif
  }

  // Восстановление работоспособности после ORERR / NERR / FERR
  void MarlinSerial::handle_usart_error() {
    // В ветке ошибок ядро DATA не читает, поэтому RBNE остаётся установленным
    // и прерывание генерируется снова. Чтение DATA сбрасывает RBNE и флаги ошибок.
    (void)usart_.receive_data8();
    usart_.handle_errors();
    usart_.get_config().state = usart::USART_State::IDLE;
    usart_.flush_buffer(true);

    #if ENABLED(EMERGENCY_PARSER)
      const uint8_t idx = get_serial_index(usart_.get_base());
      if (idx != 0xFF) e_states[idx] = EmergencyParser::State::EP_RESET;
    #endif

    usart_.set_interrupt_enable(usart::Interrupt_Type::INTR_RBNEIE, true);
  }

#endif // !SERIAL_DMA

#endif // ARDUINO_ARCH_MFL

