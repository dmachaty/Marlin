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

#ifndef _HAL_IO_H
#define _HAL_IO_H

#include <MK64F12.h>
#include <Arduino.h>
#include <pinmapping.h>
#include <fsl_gpio_hal.h>

//bool useable_hardware_PWM(pin_t pin);
//#define USEABLE_HARDWARE_PWM(pin) useable_hardware_PWM(pin)

#define PIN_BIT(pin)                (1UL << pin) // CANNOT BE MK64F12_PIN -> exists

#define MK64F12_GPIO_OFFSET         (0x0020)    // Between two ports
#define MK64F12_GPIO(port)          ((volatile GPIO_Type *)(PTA_BASE + MK64F12_GPIO_OFFSET * port))

#define MK64F12_PORT_OFFSET         (0x1000)    // Between two ports
#define MK64F12_PORT(port)          ((volatile PORT_Type *)(PORTA_BASE + MK64F12_PORT_OFFSET * port))

#define SET_DIR_INPUT(IO)       (MK64F12_GPIO(MK64F12_PIN_PORT(IO))->PDDR &= ~MK64F12_PIN(MK64F12_PIN_PIN(IO)))
#define SET_DIR_OUTPUT(IO)      (MK64F12_GPIO(MK64F12_PIN_PORT(IO))->PDDR |=  MK64F12_PIN(MK64F12_PIN_PIN(IO)))


//#define SET_MODE(IO, mode)      (pin_mode((LPC1768_PIN_PORT(IO), LPC1768_PIN_PIN(IO)), mode))

#define WRITE_PIN_SET(IO)       (MK64F12_GPIO(MK64F12_PIN_PORT(IO))->PSOR = MK64F12_PIN(MK64F12_PIN_PIN(IO)))
#define WRITE_PIN_CLR(IO)       (MK64F12_GPIO(MK64F12_PIN_PORT(IO))->PCOR = MK64F12_PIN(MK64F12_PIN_PIN(IO)))

#define READ_PIN(IO)            ((MK64F12_GPIO(MK64F12_PIN_PORT(IO))->PDIR & MK64F12_PIN(MK64F12_PIN_PIN(IO))) ? 1 : 0)
#define WRITE_PIN(IO,V)         ((V) ? WRITE_PIN_SET(IO) : WRITE_PIN_CLR(IO))


/**
 * Magic I/O routines
 *
 * Now you can simply SET_OUTPUT(STEP); WRITE(STEP, HIGH); WRITE(STEP, LOW);
 *
 * Why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
 */

/// Read a pin
#define _READ(IO)         READ_PIN(IO)

/// Write to a pin
#define _WRITE_VAR(IO,V)  digitalWrite(IO,V)

#define _WRITE(IO,V)      WRITE_PIN(IO,V)

/// toggle a pin
#define _TOGGLE(IO)       _WRITE(IO, !READ(IO))

/// set pin as input
#define _SET_INPUT(IO)    SET_DIR_INPUT(IO)

/// set pin as output
#define _SET_OUTPUT(IO)   SET_DIR_OUTPUT(IO)

/// set pin as input with pullup mode
#define _PULLUP(IO,V)     pinMode(IO, (V) ? INPUT_PULLUP : INPUT)

/// set pin as input with pulldown mode
#define _PULLDOWN(IO,V)   pinMode(IO, (V) ? INPUT_PULLDOWN : INPUT)

/// check if pin is an input
#define _GET_INPUT(IO)    (MK64F12_GPIO(MK64F12_PIN_PORT(IO))->PDDR & MK64F12_PIN(MK64F12_PIN_PIN(IO)) != 0)

/// check if pin is an output
#define _GET_OUTPUT(IO)   (MK64F12_GPIO(MK64F12_PIN_PORT(IO))->PDDR & MK64F12_PIN(MK64F12_PIN_PIN(IO)) == 0)

/// check if pin is a timer
/// all gpio pins are pwm capable, either interrupt or hardware pwm controlled
#define _GET_TIMER(IO)    TRUE

/// Read a pin wrapper
#define READ(IO)          _READ(IO)

/// Write to a pin wrapper
#define WRITE_VAR(IO,V)   _WRITE_VAR(IO,V)
#define WRITE(IO,V)       _WRITE(IO,V)

/// toggle a pin wrapper
#define TOGGLE(IO)        _TOGGLE(IO)

/// set pin as input wrapper
#define SET_INPUT(IO)     _SET_INPUT(IO)
/// set pin as input with pullup wrapper
#define SET_INPUT_PULLUP(IO)    do{ _SET_INPUT(IO); _PULLUP(IO, HIGH); }while(0)
/// set pin as input with pulldown wrapper
#define SET_INPUT_PULLDOWN(IO)  do{ _SET_INPUT(IO); _PULLDOWN(IO, HIGH); }while(0)
/// set pin as output wrapper  -  reads the pin and sets the output to that value
#define SET_OUTPUT(IO)          do{ _WRITE(IO, _READ(IO)); _SET_OUTPUT(IO); }while(0)

/// check if pin is an input wrapper
#define GET_INPUT(IO)     _GET_INPUT(IO)
/// check if pin is an output wrapper
#define GET_OUTPUT(IO)    _GET_OUTPUT(IO)

/// check if pin is a timer (wrapper)
#define GET_TIMER(IO)     _GET_TIMER(IO)

// Shorthand
#define OUT_WRITE(IO,V)   do{ SET_OUTPUT(IO); WRITE(IO,V); }while(0)

#endif // _HAL_IO_H
