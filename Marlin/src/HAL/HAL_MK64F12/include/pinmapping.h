/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/*
 *  Daniel: WHAT KIND OF F***ING SORCERY IS THIS?!!!!!!!!!!
 */


#ifndef _PINMAPPING_H_
#define _PINMAPPING_H_

#include <stdint.h>
#include "../../../core/macros.h"

typedef int16_t pin_t;

#define PORT_0  000
#define PORT_1  001
#define PORT_2  010
#define PORT_3  011
#define PORT_4  100

#define PORT_(p)  PORT_##p
#define PORT(p)   PORT_(p)

#define PIN_0  00000
#define PIN_1  00001
#define PIN_2  00010
#define PIN_3  00011
#define PIN_4  00100
#define PIN_5  00101
#define PIN_6  00110
#define PIN_7  00111
#define PIN_8  01000
#define PIN_9  01001
#define PIN_10 01010
#define PIN_11 01011
#define PIN_12 01100
#define PIN_13 01101
#define PIN_14 01110
#define PIN_15 01111
#define PIN_16 10000
#define PIN_17 10001
#define PIN_18 10010
#define PIN_19 10011
#define PIN_20 10100
#define PIN_21 10101
#define PIN_22 10110
#define PIN_23 10111
#define PIN_24 11000
#define PIN_25 11001
#define PIN_26 11010
#define PIN_27 11011
#define PIN_28 11100
#define PIN_29 11101
#define PIN_30 11110
#define PIN_31 11111

#define PIN_(p) PIN_##p
#define PIN(p)  PIN_(p)

#define ADC_NONE    0000
#define ADC_CHAN_0  0001
#define ADC_CHAN_1  0010
#define ADC_CHAN_2  0011
#define ADC_CHAN_3  0100
#define ADC_CHAN_4  0101
#define ADC_CHAN_5  0110
#define ADC_CHAN_6  0111
#define ADC_CHAN_7  1000

#define ADC_CHAN_(c)  ADC_CHAN_##c
#define ADC_CHAN(p)   ADC_CHAN_(p)

#define BOOL_0 0
#define BOOL_1 1
#define BOOL_(b)      BOOL_##b

#define INTERRUPT(b)  BOOL_(b)
#define PWM(b)        BOOL_(b)

// Combine elements into pin bits: 0b00AAAAWIPPPNNNNN
#define MK64F12_PIN_(port, pin, int, pwm, adc)  0b00##adc##pwm##int##port##pin
#define MK64F12_PIN(port, pin, int, pwm, adc)   MK64F12_PIN_(port, pin, int, pwm, adc)

constexpr uint8_t MK64F12_PIN_PORT(const pin_t pin) { return ((uint8_t)((pin >> 5) & 0b111)); }
constexpr uint8_t MK64F12_PIN_PIN(const pin_t pin) { return ((uint8_t)(pin & 0b11111)); }
constexpr bool MK64F12_PIN_INTERRUPT(const pin_t pin) { return (((pin >> 8) & 0b1) != 0); }
constexpr bool MK64F12_PIN_PWM(const pin_t pin) { return (((pin >> 9) & 0b1) != 0); }
constexpr int8_t MK64F12_PIN_ADC(const pin_t pin) { return (int8_t)((pin >> 10) & 0b1111) - 1; }

// ******************
// Runtime pinmapping
// ******************
#define P_NC -1

#define PTA0  MK64F12_PIN(PORT(0), PIN( 0), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA1  MK64F12_PIN(PORT(0), PIN( 1), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA2  MK64F12_PIN(PORT(0), PIN( 2), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA3  MK64F12_PIN(PORT(0), PIN( 3), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA4  MK64F12_PIN(PORT(0), PIN( 4), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA5  MK64F12_PIN(PORT(0), PIN( 5), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA12 MK64F12_PIN(PORT(0), PIN(12), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA13 MK64F12_PIN(PORT(0), PIN(13), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA14 MK64F12_PIN(PORT(0), PIN(14), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA15 MK64F12_PIN(PORT(0), PIN(15), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA16 MK64F12_PIN(PORT(0), PIN(16), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA17 MK64F12_PIN(PORT(0), PIN(17), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA18 MK64F12_PIN(PORT(0), PIN(18), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTA19 MK64F12_PIN(PORT(0), PIN(19), INTERRUPT(0), PWM(0), ADC_NONE)

#define PTB0  MK64F12_PIN(PORT(1), PIN( 0), INTERRUPT(0), PWM(0), ADC_CHAN_0)   // ADC TBD
#define PTB1  MK64F12_PIN(PORT(1), PIN( 1), INTERRUPT(0), PWM(0), ADC_CHAN_1)
#define PTB2  MK64F12_PIN(PORT(1), PIN( 2), INTERRUPT(0), PWM(0), ADC_CHAN_2)
#define PTB3  MK64F12_PIN(PORT(1), PIN( 3), INTERRUPT(0), PWM(0), ADC_CHAN_3)
#define PTB9  MK64F12_PIN(PORT(1), PIN( 9), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB10 MK64F12_PIN(PORT(1), PIN(10), INTERRUPT(0), PWM(0), ADC_CHAN_4)
#define PTB11 MK64F12_PIN(PORT(1), PIN(11), INTERRUPT(0), PWM(0), ADC_CHAN_5)
#define PTB16 MK64F12_PIN(PORT(1), PIN(16), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB17 MK64F12_PIN(PORT(1), PIN(17), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB18 MK64F12_PIN(PORT(1), PIN(18), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB19 MK64F12_PIN(PORT(1), PIN(19), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB20 MK64F12_PIN(PORT(1), PIN(20), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB21 MK64F12_PIN(PORT(1), PIN(21), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB22 MK64F12_PIN(PORT(1), PIN(22), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTB23 MK64F12_PIN(PORT(1), PIN(23), INTERRUPT(0), PWM(0), ADC_NONE)

#define PTC0  MK64F12_PIN(PORT(2), PIN( 0), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC1  MK64F12_PIN(PORT(2), PIN( 1), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC2  MK64F12_PIN(PORT(2), PIN( 2), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC3  MK64F12_PIN(PORT(2), PIN( 3), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC4  MK64F12_PIN(PORT(2), PIN( 4), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC5  MK64F12_PIN(PORT(2), PIN( 5), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC6  MK64F12_PIN(PORT(2), PIN( 6), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC7  MK64F12_PIN(PORT(2), PIN( 7), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC8  MK64F12_PIN(PORT(2), PIN( 8), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC9  MK64F12_PIN(PORT(2), PIN( 9), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC10 MK64F12_PIN(PORT(2), PIN(10), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC11 MK64F12_PIN(PORT(2), PIN(11), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC12 MK64F12_PIN(PORT(2), PIN(12), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC13 MK64F12_PIN(PORT(2), PIN(13), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC14 MK64F12_PIN(PORT(2), PIN(14), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC15 MK64F12_PIN(PORT(2), PIN(15), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC16 MK64F12_PIN(PORT(2), PIN(16), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC17 MK64F12_PIN(PORT(2), PIN(17), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTC18 MK64F12_PIN(PORT(2), PIN(18), INTERRUPT(0), PWM(0), ADC_NONE)

#define PTD0  MK64F12_PIN(PORT(3), PIN( 0), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTD1  MK64F12_PIN(PORT(3), PIN( 1), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTD2  MK64F12_PIN(PORT(3), PIN( 2), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTD3  MK64F12_PIN(PORT(3), PIN( 3), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTD4  MK64F12_PIN(PORT(3), PIN( 4), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTD5  MK64F12_PIN(PORT(3), PIN( 5), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTD6  MK64F12_PIN(PORT(3), PIN( 6), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTD7  MK64F12_PIN(PORT(3), PIN( 7), INTERRUPT(0), PWM(0), ADC_NONE)

#define PTE0  MK64F12_PIN(PORT(4), PIN( 0), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE1  MK64F12_PIN(PORT(4), PIN( 1), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE2  MK64F12_PIN(PORT(4), PIN( 2), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE3  MK64F12_PIN(PORT(4), PIN( 3), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE4  MK64F12_PIN(PORT(4), PIN( 4), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE5  MK64F12_PIN(PORT(4), PIN( 5), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE6  MK64F12_PIN(PORT(4), PIN( 6), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE24 MK64F12_PIN(PORT(4), PIN(24), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE25 MK64F12_PIN(PORT(4), PIN(25), INTERRUPT(0), PWM(0), ADC_NONE)
#define PTE26 MK64F12_PIN(PORT(4), PIN(26), INTERRUPT(0), PWM(0), ADC_NONE)

// COPY TO CREATE NEW GATE:
//#define PT0  MK64F12_PIN(PORT(5), PIN( 0), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT1  MK64F12_PIN(PORT(5), PIN( 1), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT2  MK64F12_PIN(PORT(5), PIN( 2), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT3  MK64F12_PIN(PORT(5), PIN( 3), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT4  MK64F12_PIN(PORT(5), PIN( 4), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT5  MK64F12_PIN(PORT(5), PIN( 5), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT6  MK64F12_PIN(PORT(5), PIN( 6), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT7  MK64F12_PIN(PORT(5), PIN( 7), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT8  MK64F12_PIN(PORT(5), PIN( 8), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT9  MK64F12_PIN(PORT(5), PIN( 9), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT10 MK64F12_PIN(PORT(5), PIN(10), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT11 MK64F12_PIN(PORT(5), PIN(11), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT12 MK64F12_PIN(PORT(5), PIN(12), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT13 MK64F12_PIN(PORT(5), PIN(13), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT14 MK64F12_PIN(PORT(5), PIN(14), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT15 MK64F12_PIN(PORT(5), PIN(15), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT16 MK64F12_PIN(PORT(5), PIN(16), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT17 MK64F12_PIN(PORT(5), PIN(17), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT18 MK64F12_PIN(PORT(5), PIN(18), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT19 MK64F12_PIN(PORT(5), PIN(19), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT20 MK64F12_PIN(PORT(5), PIN(20), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT21 MK64F12_PIN(PORT(5), PIN(21), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT22 MK64F12_PIN(PORT(5), PIN(22), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT23 MK64F12_PIN(PORT(5), PIN(23), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT24 MK64F12_PIN(PORT(5), PIN(24), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT25 MK64F12_PIN(PORT(5), PIN(25), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT26 MK64F12_PIN(PORT(5), PIN(26), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT27 MK64F12_PIN(PORT(5), PIN(27), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT28 MK64F12_PIN(PORT(5), PIN(28), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT29 MK64F12_PIN(PORT(5), PIN(29), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT30 MK64F12_PIN(PORT(5), PIN(30), INTERRUPT(0), PWM(0), ADC_NONE)
//#define PT31 MK64F12_PIN(PORT(5), PIN(31), INTERRUPT(0), PWM(0), ADC_NONE)

// Pin index for M43 and M226
constexpr pin_t pin_map[] = {
    PTA0,   PTA1,   PTA2,   PTA3,   PTA4,   PTA5,   P_NC,   P_NC,
    P_NC,   P_NC,   P_NC,   P_NC,   PTA12,  PTA13,  PTA14,  PTA15,
    PTA16,  PTA17,  PTA18,  PTA19,  P_NC,   P_NC,   P_NC,   P_NC,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,

    PTB0,   PTB1,   PTB2,   PTB3,   P_NC,   P_NC,   P_NC,   P_NC,
    P_NC,   PTB9,   PTB10,  PTB11,  P_NC,   P_NC,   P_NC,   P_NC,
    PTB16,  PTB17,  PTB18,  PTB19,  PTB20,  PTB21,  PTB22,  PTB23,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,

    PTC0,   PTC1,   PTC2,   PTC3,   PTC4,   PTC5,   PTC6,   PTC7,
    PTC8,   PTC9,   PTC10,  PTC11,  PTC12,  PTC13,  PTC14,  PTC15,
    PTC16,  PTC17,  PTC18,  P_NC,   P_NC,   P_NC,   P_NC,   P_NC,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,

    PTD0,   PTD1,   PTD2,   PTD3,   PTD4,   PTD5,   PTD6,   PTD7,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,

    PTE0,   PTE1,   PTE2,   PTE3,   PTE4,   PTE5,   PTE6,   P_NC,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,
    P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,   P_NC,
    PTE24,  PTE25,  PTE26,  P_NC,   P_NC,   P_NC,   P_NC,   P_NC
};

constexpr uint8_t NUM_DIGITAL_PINS = COUNT(pin_map);

constexpr pin_t adc_pin_table[] = {
  PTB0, PTB1, PTB2, PTB3, PTB10, PTB11
};

#define NUM_ANALOG_INPUTS 6

// PTE0..6 are for the onboard SD card
#define HAL_SENSITIVE_PINS PTE0, PTE1, PTE2, PTE3, PTE4, PTE5, PTE6

// Get the digital pin for an analog index
pin_t analogInputToDigitalPin(const int8_t p);
#define digitalPinToInterrupt(pin) (pin)
// Return the index of a pin number
// The pin number given here is in the form ppp:nnnnn
int16_t GET_PIN_MAP_INDEX(const pin_t pin);

// Test whether the pin is valid
bool VALID_PIN(const pin_t p);

// Get the analog index for a digital pin
int8_t DIGITAL_PIN_TO_ANALOG_PIN(const pin_t p);

// Test whether the pin is PWM
bool PWM_PIN(const pin_t p);

// Test whether the pin is interruptable
bool INTERRUPT_PIN(const pin_t p);
#define LPC1768_PIN_INTERRUPT_M(pin) (((pin >> 8) & 0b1) != 0)

// Get the pin number at the given index
pin_t GET_PIN_MAP_PIN(const int16_t ind);

// Parse a G-code word into a pin index
int16_t PARSED_PIN_INDEX(const char code, const int16_t dval);

#endif // _PINMAPPING_H_