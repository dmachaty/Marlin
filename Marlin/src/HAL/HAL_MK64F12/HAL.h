/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2018 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

//  System
//  Framework
#include <MK64F12.h>
//  Project
#include <Arduino.h>
#include <pinmapping.h>
#include "HardwareSerial.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"

/*
 * NVIC Interrupt priorities
 * Grouping disabled (AIRCR->SCB), 4 bits, 16 preempt priorities
 * 
 * Priority  0 (0, 0,0):  PORTx (external pins like killswitch must trigger always!)
 * Priority  1 (0, 1,0):  Timer0
 * Priority  2 (0, 2,0):  Timer1
 * Priority  3 (0, 3,0):  SHDC
 * Priority  4 (0, 4,0):  UARTs
 * Priority  5 (0, 5,0):  PWM
 * Priority  6 (0, 6,0):  
 * Priority  7 (0, 7,0):  
 * Priority  8 (0, 8,0):  
 * Priority  9 (0, 9,0):  
 * Priority 10 (0,10,0):  
 * Priority 11 (0,11,0):  
 * Priority 12 (0,12,0):  
 * Priority 13 (0,13,0):  
 * Priority 14 (0,14,0):  
 * Priority 15 (0,15,0):  SYSTICK (by default from CORE_CM4.h)
  * 
 */

#define SERIAL_PORT 0