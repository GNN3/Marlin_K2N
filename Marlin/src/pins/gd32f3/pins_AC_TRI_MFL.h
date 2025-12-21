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
#pragma once

/**
 * Anycubic Kobra 2 NEO (GD32F303RE) board pin assignments
 */
#define ALLOW_GD32F3

#define BOARD_INFO_NAME      "AC_TRI_GD32_MFL"
#define DEFAULT_MACHINE_NAME "Anycubic Kobra2 NEO"

#define BOARD_NO_NATIVE_USB
//#define LEVEING_CALIBRATION_MODULE
#define DISABLE_DEBUG

//
// U(S)ART
//
#define UART1_TX_PIN                      PA2
#define UART1_RX_PIN                      PA3

//
// EEPROM
//
#define FLASH_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE                    0x1000U           // 4KB


//
// Endstops
//
#define ONBOARD_ENDSTOPPULLUPS

#define X_MIN_PIN                           PB11
#define Y_MIN_PIN                           PC13
#define Z_MIN_PIN                           PA1
#define Z_MAX_PIN                           PA0
#define E0_MIN_PIN                          -1

//
// Steppers
//
#define X_ENABLE_PIN                        PA15
#define X_STEP_PIN                          PA12
#define X_DIR_PIN                           PA11

#define Y_ENABLE_PIN                        PA15
#define Y_STEP_PIN                          PA9
#define Y_DIR_PIN                           PA8

#define Z_ENABLE_PIN                        PA15
#define Z_STEP_PIN                          PB0
#define Z_DIR_PIN                           PB1

#define E0_ENABLE_PIN                       PA15
#define E0_STEP_PIN                         PB15
#define E0_DIR_PIN                          PB14

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC3
#define TEMP_BED_PIN                        PC1

//
// Heaters
//
#define HEATER_0_PIN                        PB8
#define HEATER_BED_PIN                      PB9
#define SUICIDE_PIN                         PB6
#define SUICIDE_PIN_STATE                   LOW

//
// Fans
//
#define FAN0_PIN                            PB5
#define FAN1_PIN                            PB13
#define FAN2_PIN                            PB12
#define CONTROLLER_FAN_PIN                  FAN2_PIN

//
// Misc
//
#define BEEPER_PIN                          -1
#define FIL_RUNOUT_PIN                      PC15
#define POWER_LOSS_PIN                      PC2
#define POWER_MONITOR_VOLTAGE_PIN           PC2
#define AUTO_LEVEL_RX_PIN                   PB8
//
// SD Card
//
#define ONBOARD_SDIO
#define SD_DETECT_PIN                       PA10

#ifdef ONBOARD_SDIO
  #define SDIO_D0_PIN                       PC8
  #define SDIO_D1_PIN                       PC9
  #define SDIO_D2_PIN                       PC10
  #define SDIO_D3_PIN                       PC11
  #define SDIO_CK_PIN                       PC12
  #define SDIO_CMD_PIN                      PD2
#else
  #define SOFTWARE_SPI
  #define SS_PIN        PC11
  #define SDSS          PC11
  #define SCK_PIN       PC12
  #define MISO_PIN      PC8
  #define MOSI_PIN      PD2
#endif

//
// TFT Display
//
#define TFT_CS_PIN                        PA4
#define TFT_A0_PIN                        PA6
#define TFT_DC_PIN                        TFT_A0_PIN 
#define TFT_SCK_PIN                       PA5
#define TFT_MOSI_PIN                      PA7
#define TFT_MISO_PIN                      TFT_MOSI_PIN
#define TFT_BACKLIGHT_PIN                 PC0
#define TFT_RESET_PIN                     -1

#define BTN_ENC                           PB4
#define BTN_EN1                           PB10
#define BTN_EN2                           PB3
