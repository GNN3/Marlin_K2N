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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../platforms.h"

#ifdef ARDUINO_ARCH_MFL

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"

#include <ADC.hpp>
#include <Analog.h>

uint16_t MarlinHAL::adc_result;
bool MarlinHAL::adc_conv_active = false;

#if ENABLED(POSTMORTEM_DEBUGGING)
  extern void install_min_serial();
#endif

#if ENABLED(MARLIN_DEV_MODE)
  // Dump the clock frequencies of the system, AHB, APB1, APB2, and F_CPU.
  static inline void HAL_clock_frequencies_dump() {
    auto& rcuInstance = rcu::RCU::get_instance();
    uint32_t freq = rcuInstance.get_clock_frequency(rcu::Clock_Frequency::CK_SYS);
    SERIAL_ECHOPGM("\nSYSTEM_CLOCK=", freq);
    freq = rcuInstance.get_clock_frequency(rcu::Clock_Frequency::CK_AHB);
    SERIAL_ECHOPGM("\nABH_CLOCK=", freq);
    freq = rcuInstance.get_clock_frequency(rcu::Clock_Frequency::CK_APB1);
    SERIAL_ECHOPGM("\nAPB1_CLOCK=", freq);
    freq = rcuInstance.get_clock_frequency(rcu::Clock_Frequency::CK_APB2);
    SERIAL_ECHOPGM("\nAPB2_CLOCK=", freq,
    "\nF_CPU=", F_CPU);
    // Done
    SERIAL_ECHOPGM("\n--\n");
  }
#endif  // MARLIN_DEV_MODE

// Initializes the Marlin HAL
void MarlinHAL::init() {
  // Ensure F_CPU is a constant expression.
  // If the compiler breaks here, it means that delay code that should compute at compile time will not work.
  // So better safe than sorry here.
  constexpr unsigned int cpuFreq = F_CPU;
  UNUSED(cpuFreq);

  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    __set_FPSCR(__get_FPSCR() | (1 << 24));
  #endif
  
  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  SetTimerInterruptPriorities();

  // Print clock frequencies to host serial
  TERN_(MARLIN_DEV_MODE, HAL_clock_frequencies_dump());

  // Register min serial
  TERN_(POSTMORTEM_DEBUGGING, install_min_serial());
}

// ADC: non-blocking single-conversion with hardware oversampling.
// adc_init() performs the one-time hardware setup; adc_start() only triggers a
// conversion; adc_ready()/adc_value() collect the result on a later ISR tick.
void MarlinHAL::adc_init() {
  analogReadResolution(HAL_ADC_RESOLUTION);

  auto res = adc::ADC::get_instance(adc::ADC_Base::ADC0_BASE);
  if (res.error() != adc::ADC_Error_Type::OK) return;
  auto &adc = res.value();

  // Single regular conversion, software trigger, right alignment.
  adc.set_data_alignment(adc::Data_Alignment::DATA_ALIGN_RIGHT);
  adc.set_channel_length(adc::Channel_Group_Type::REGULAR_CHANNEL, 0);
  adc.set_mode(adc::Sync_Mode::SM_FREE);
  adc.set_external_trigger_enable(adc::Channel_Group_Type::REGULAR_CHANNEL, true);
  adc.set_external_group_source(adc::Channel_Group_Type::REGULAR_CHANNEL,
                                adc::External_Trigger_Source::ADC0_1_REGULAR_SOFTWARE);

  #ifdef GD32F303RE
    // 128x hardware oversampling, shift back to HAL resolution, 12-bit output.
    // 128 samples × 68 cycles / 120 MHz = 72.5 µs — spans >2 full periods of
    // the 31.4 kHz fan PWM, so PWM-coupled noise averages out completely.
    adc.set_oversampling_configuration(adc::Oversampling_Conversion::OVERSAMPLING_CONVERT_ALL,
                                       adc::Oversampling_Shift::OVERSAMPLING_SHIFT_7BIT,
                                       adc::Oversampling_Ratio::OVERSAMPLING_RATIO_MUL128);
    adc.set_oversampling_enable(true);
  #endif

  adc.enable();
  adc.set_temperature_voltage_reference_enable(true);

  // Errata 2.4.1 delay for VDDA stabilization.
  for (uint32_t i = 0; i < 40000; ++i) { __asm__ volatile ("nop"); }

  // Reset + run calibration once.
  adc.calibration_enable();

  adc_conv_active = false;
}

void MarlinHAL::adc_start(const pin_t pin) {
  auto res = adc::ADC::get_instance(adc::ADC_Base::ADC0_BASE);
  if (res.error() != adc::ADC_Error_Type::OK) { adc_conv_active = false; return; }
  auto &adc = res.value();

  const adc::ADC_Channel channel = getAdcChannelFromPin(pin);
  if (channel == adc::ADC_Channel::INVALID) { adc_conv_active = false; return; }

  // Long sample time is required for the high-impedance thermistor divider.
  adc.set_regular_channel_sequence(0, channel, adc::ADC_Sample_Time::SAMPLETIME_55_5_CYCLES);

  // Trigger the (oversampled) conversion and return without waiting for EOC.
  adc.set_software_trigger_group(adc::Channel_Group_Type::REGULAR_CHANNEL);
  adc_conv_active = true;
}

bool MarlinHAL::adc_ready() {
  if (!adc_conv_active) return false;
  auto res = adc::ADC::get_instance(adc::ADC_Base::ADC0_BASE);
  if (res.error() != adc::ADC_Error_Type::OK) return false;
  return res.value().get_flag(adc::Status_Flags::FLAG_EOC);
}

uint16_t MarlinHAL::adc_value() {
  auto res = adc::ADC::get_instance(adc::ADC_Base::ADC0_BASE);
  if (res.error() != adc::ADC_Error_Type::OK) return 0;
  auto &adc = res.value();
  const uint32_t data = adc.get_regular_data();
  adc.clear_flag(adc::Status_Flags::FLAG_EOC);
  adc_conv_active = false;
  adc_result = static_cast<uint16_t>(data);
  return adc_result;
}

// Returns the reset source based on the flags set in the RCU module
uint8_t MarlinHAL::get_reset_source() {
  return
    (RCU_I.get_flag(rcu::Status_Flags::FLAG_FWDGTRST)) ? RST_WATCHDOG :
    (RCU_I.get_flag(rcu::Status_Flags::FLAG_SWRST)) ? RST_SOFTWARE :
    (RCU_I.get_flag(rcu::Status_Flags::FLAG_EPRST)) ? RST_EXTERNAL :
    (RCU_I.get_flag(rcu::Status_Flags::FLAG_PORRST)) ? RST_POWER_ON :
    (RCU_I.get_flag(rcu::Status_Flags::FLAG_LPRST)) ? RST_BROWN_OUT :
    0;
}

// Returns the amount of free memory available in bytes
int MarlinHAL::freeMemory() {
  volatile char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

// Watchdog Timer
#if ENABLED(USE_WATCHDOG)
  #define WDT_TIMEOUT_US  TERN(WATCHDOG_DURATION_8S, 8000000, 4000000)  // 4 or 8 second timeout

  #include <FWatchdogTimer.h>

  FWatchdogTimer& watchdogTimer = FWatchdogTimer::get_instance();

  // Initializes the watchdog timer
  void MarlinHAL::watchdog_init() {
    IF_DISABLED(DISABLE_WATCHDOG_INIT, watchdogTimer.begin(WDT_TIMEOUT_US));
  }

  // Refreshes the watchdog timer to prevent system reset
  void MarlinHAL::watchdog_refresh() {
    watchdogTimer.reload();
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN);  // Heartbeat indicator
    #endif
  }
#endif

extern "C" {
  extern unsigned int _ebss;  // End of bss section
}

// Resets the system to initiate a firmware flash.
WEAK void flashFirmware(const int16_t) {
  hal.reboot();
}

#endif // ARDUINO_ARCH_MFL
