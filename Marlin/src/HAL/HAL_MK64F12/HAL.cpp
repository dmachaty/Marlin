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
#ifdef TARGET_MK64FN1M0

// System
#include <stdio.h>
#include <stdarg.h>
// Framework
#include <fsl_gpio_hal.h>
#include <fsl_sdhc_hal.h>
#include <fsl_port_hal.h>
#include <fsl_smc_hal.h>
#include <fsl_wdog_hal.h>
#include <fsl_ewm_hal.h>
#include <MK64F12.h>
#include <MK64F12_extension.h>
// Project
#include "../../inc/MarlinConfig.h"
#include <Arduino.h>
#include "HAL.h"
#include "HAL_Board.h"
#include "HAL_Board_Pins.h"
#include "HAL_Board_Clock.h"
#include "HAL_UART.h"

volatile uint64_t _millis;

HAL_UART MYSERIAL0(UART0);

void HAL_TestSuite_Init(void){
  MYSERIAL0.begin(115200);
}

void HAL_TestSuite_Loop(void){
  // Mark the cycle start time
  uint32_t time_loopStart = _millis;

  // Set Blue LED ON
  GPIO_HAL_WritePinOutput(PTB, 21, 0); // Blue LED

  // Clear serial screen
  MYSERIAL0.printf("\033[2J\033[H");  // clear screen command  

  // Report UART status:
  HAL_UART_STATUS_t status = MYSERIAL0.getStatus();
  MYSERIAL0.printf("Baud rate: %d\r\n", status.baudRate);
  MYSERIAL0.printf("RX RD pos: %d\r\n", status.RxQueueReadPos);
  MYSERIAL0.printf("RX WR pos: %d\r\n", status.RxQueueWritePos);
  MYSERIAL0.printf("_millis:   %d\r\n", _millis);
  
  // Set Blue LED Off
  GPIO_HAL_WritePinOutput(PTB, 21, 1); // Blue LED

  // Make it a second cycle
  while((_millis - time_loopStart) < 1000);
}

int main(void){

  HAL_Board_Init();

  HAL_TestSuite_Init();

  uint32_t i;

  while(1){
    HAL_TestSuite_Loop();
  }

}
#endif