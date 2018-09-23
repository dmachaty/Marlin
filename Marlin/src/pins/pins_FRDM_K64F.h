/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (C) 2017 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Preliminary FRDM-K64F Pin assignments
 */

#ifndef TARGET_MK64FN1M0
  #error "Oops! PlatformIO environment should be set to FRDM_K64F"
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME "FRDM_K64F"
  #define DEFAULT_WEBSITE_URL "https://www.nxp.com/FRDM-K64F"
  #define BOARD_LINK_SCH    "https://www.nxp.com/downloads/en/schematics/FRDM-K64F_SCH.pdf"
  #define CHIP_LINK_DSHEET  "https://www.nxp.com/docs/en/data-sheet/K64P144M120SF5.pdf"
  #define CHIP_LINK_REFMAN  "https://www.nxp.com/docs/en/reference-manual/K64P144M120SF5RM.pdf"
#endif

#undef F_CPU
#define F_CPU 120000000

//
// Servos
//
// TODO - check requirements for pin
//#define SERVO0_PIN          P1_23

//
// Limit Switches (Header J1)
//
#define X_MIN_PIN           PTB18
#define X_MAX_PIN           PTC16
#define Y_MIN_PIN           PTB19
#define Y_MAX_PIN           PTC17
#define Z_MIN_PIN           PTC1
#define Z_MAX_PIN           PTB9

//
// Steppers (Headers J1+J2)
//
#define X_STEP_PIN          PTA1
#define X_DIR_PIN           PTC9
#define X_ENABLE_PIN        PTB23

#define Y_STEP_PIN          PTC0
#define Y_DIR_PIN           PTA2
#define Y_ENABLE_PIN        PTC7

#define Z_STEP_PIN          PTC2
#define Z_DIR_PIN           PTC5
#define Z_ENABLE_PIN        PTC3

#define E0_STEP_PIN         PTE26
#define E0_DIR_PIN          PTC12
#define E0_ENABLE_PIN       PTC4

#define E1_STEP_PIN         PTC11
#define E1_DIR_PIN          PTC10
#define E1_ENABLE_PIN       PTB20


//
// Temperature Sensors (Header J4)
// 3.3V max when defined as an analog input
//
#define TEMP_0_PIN          PTB2  // ADC0_SE12
#define TEMP_BED_PIN        PTB3  // ADC0_SE13 
#define TEMP_1_PIN          PTB10 // ADC1_SE14 
#define TEMP_2_PIN          PTB11 // ADC1_SE15

//
// Heaters / Fans (Header J6)
//
#define HEATER_BED_PIN      PTC18
#define HEATER_0_PIN        PTD4
#define HEATER_1_PIN        PTD5
#ifndef FAN_PIN
  #define FAN_PIN           PTD6
#endif
#define FAN1_PIN            PTD7

//
// Display
//
/* TODO - check if necessarry
#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #define BEEPER_PIN        P1_31
  //#define DOGLCD_A0       P2_06
  #define DOGLCD_CS         P0_16

  #define BTN_EN1           P3_25
  #define BTN_EN2           P3_26
  #define BTN_ENC           P2_11

  #define SD_DETECT_PIN     P1_18
  #define SDSS              P1_21

  #define STAT_LED_RED_PIN  P1_19
  #define STAT_LED_BLUE_PIN P1_20
#endif
*/
