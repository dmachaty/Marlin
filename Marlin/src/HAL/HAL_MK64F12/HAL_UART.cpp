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

//  System
//  Framework
#include <fsl_gpio_hal.h>
#include <fsl_interrupt_manager.h>
#include <fsl_uart_hal.h>
#include <MK64F12.h>
#include <MK64F12_extension.h>
//  Project
#include "HAL.h"
#include "HAL_UART.h"

#if    SERIAL_PORT == 0 || SERIAL_PORT_2 == 0
  HAL_UART Serial = HAL_UART(UART0);
#elif  SERIAL_PORT == 1 || SERIAL_PORT_2 == 1
  HAL_UART Serial1 = HAL_UART(UART1);
// Note - other serial ports have only 1-word FIFOs
#endif

// Interrupt handlers
extern "C"  {
  // UART0
  void UART0_RX_TX_IRQHandler() {
    #if SERIAL_PORT == 0 || SERIAL_PORT_2 == 0
      Serial.IRQHandler(UART0);
    #endif
  }
  void UART0_ERR_IRQHandler()   {
    #if SERIAL_PORT == 0 || SERIAL_PORT_2 == 0
      Serial.IRQErrHandler(UART0);
    #endif
  }

  // UART1
  void UART1_RX_TX_IRQHandler() {
    #if SERIAL_PORT == 1 || SERIAL_PORT_2 == 1
      Serial1.IRQHandler(UART1);
    #endif
  }
  void UART1_ERR_IRQHandler()   {
    #if SERIAL_PORT == 1 || SERIAL_PORT_2 == 1
      Serial1.IRQErrHandler(UART1);
    #endif
  }
}

void HAL_UART::begin(uint32_t baudRate) {

    // Initialize UART interface
    UART_HAL_Init(UARTx);
    
    // Configure packet parameters
    UART_HAL_SetBaudRate(UARTx, SystemCoreClock, baudRate);
    UART_HAL_SetParityMode(UARTx, HAL_UART_UART0_DEFAULT_PARITY);
    UART_HAL_SetBitCountPerChar(UARTx, HAL_UART_UART0_DEFAULT_DATABITS);
    UART_HAL_SetStopBitCount(UARTx, HAL_UART_UART0_DEFAULT_STOPBITS);
    
    // Enable FIFOs
    UART_HAL_SetTxFifoCmd(UARTx, true);
    UART_HAL_SetRxFifoCmd(UARTx, true);
    
    // Flush FIFOs
    UART_HAL_FlushTxFifo(UARTx);
    UART_HAL_FlushRxFifo(UARTx);
    
    // Enable receiver and transmitter
    UART_HAL_EnableTransmitter(UARTx);
    UART_HAL_EnableReceiver(UARTx);

    UART_HAL_SetIntMode(UARTx, kUartIntRxDataRegFull, true);
    //UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, true); // -> SET ONLY WHEN NECESSARRY
    // TODO - enable error int?

    if(UARTx == UART0){
        NVIC_SetPriority(UART0_RX_TX_IRQn, NVIC_EncodePriority(0,3,0) );
        NVIC_EnableIRQ(UART0_RX_TX_IRQn);    }
    if(UARTx == UART1){
        NVIC_SetPriority(UART1_RX_TX_IRQn, NVIC_EncodePriority(0,3,0) );
        NVIC_EnableIRQ(UART1_RX_TX_IRQn);    }
}

void HAL_UART::IRQHandler(UART_Type * uartInstance){

    uint8_t inputData;

    if (UART_RD_S1_RDRF(uartInstance)) {                        // Receive data available (>1 word)

      while (UART_HAL_GetRxDatawordCountInFifo(uartInstance)) { // Read until no more characters in FIFO

        UART_HAL_Getchar(uartInstance, &inputData);             // Read single byte

        if ((RxQueueWritePos + 1) % RX_BUFFER_SIZE != RxQueueReadPos) {   // Avoid overwriting unread data
          RxBuffer[RxQueueWritePos] = inputData;                          // Write data to buffer
          RxQueueWritePos = (RxQueueWritePos + 1) % RX_BUFFER_SIZE;       // Move to next buffer position (turnover at BUFFER_SIZE)
        }
        else
          break;  // Cannot read more data (buffer full)
      }
    }


    #if TX_BUFFER_SIZE > 0
  
      if (UART_RD_S1_TDRE(uartInstance)) {                                // TX data empty
  
        UART_HAL_SetIntMode(uartInstance, kUartIntTxDataRegEmpty, false); // Disable TDRE interrupt
  
        while (UART_HAL_GetTxDatawordCountInFifo(uartInstance));          // Wait for FIFO buffer empty

        uint8_t i;  
        for (i = 0; (i < HAL_UART_TX_FIFO_SIZE) && (TxQueueWritePos != TxQueueReadPos); i++) {  // Transfer up to UART_TX_FIFO_SIZE bytes of data
          UART_HAL_Putchar(uartInstance, TxBuffer[TxQueueReadPos]);
          TxQueueReadPos = (TxQueueReadPos+1) % TX_BUFFER_SIZE;
        }
  
        // If there is no more data to send, disable the transmit interrupt - else enable it or keep it enabled
        if (TxQueueWritePos == TxQueueReadPos) {
          UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, false);
        } else UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, true);
      }
    #endif
}

void HAL_UART::IRQErrHandler(UART_Type * uartInstance){
  // No line errors implemented
  // TODO - enable framing IRQ, check status, read dummy byte, quit
}

void HAL_UART::printHex(uint8_t data){

    uint8_t nibbleTop = ((data & 0xF0)>>4);
    uint8_t nibbleBot = ((data & 0x0F));

    UART_HAL_Putchar(UART0, '0');
    UART_HAL_Putchar(UART0, 'x');
    switch(nibbleTop){
      case  0x0:  UART_HAL_Putchar(UART0, '0'); break;
      case  0x1:  UART_HAL_Putchar(UART0, '1'); break;
      case  0x2:  UART_HAL_Putchar(UART0, '2'); break;
      case  0x3:  UART_HAL_Putchar(UART0, '3'); break;
      case  0x4:  UART_HAL_Putchar(UART0, '4'); break;
      case  0x5:  UART_HAL_Putchar(UART0, '5'); break;
      case  0x6:  UART_HAL_Putchar(UART0, '6'); break;
      case  0x7:  UART_HAL_Putchar(UART0, '7'); break;
      case  0x8:  UART_HAL_Putchar(UART0, '8'); break;
      case  0x9:  UART_HAL_Putchar(UART0, '9'); break;
      case  0xA:  UART_HAL_Putchar(UART0, 'A'); break;
      case  0xB:  UART_HAL_Putchar(UART0, 'B'); break;
      case  0xC:  UART_HAL_Putchar(UART0, 'C'); break;
      case  0xD:  UART_HAL_Putchar(UART0, 'D'); break;
      case  0xE:  UART_HAL_Putchar(UART0, 'E'); break;
      case  0xF:  UART_HAL_Putchar(UART0, 'F'); break;
    }
    switch(nibbleBot){
      case  0x0:  UART_HAL_Putchar(UART0, '0'); break;
      case  0x1:  UART_HAL_Putchar(UART0, '1'); break;
      case  0x2:  UART_HAL_Putchar(UART0, '2'); break;
      case  0x3:  UART_HAL_Putchar(UART0, '3'); break;
      case  0x4:  UART_HAL_Putchar(UART0, '4'); break;
      case  0x5:  UART_HAL_Putchar(UART0, '5'); break;
      case  0x6:  UART_HAL_Putchar(UART0, '6'); break;
      case  0x7:  UART_HAL_Putchar(UART0, '7'); break;
      case  0x8:  UART_HAL_Putchar(UART0, '8'); break;
      case  0x9:  UART_HAL_Putchar(UART0, '9'); break;
      case  0xA:  UART_HAL_Putchar(UART0, 'A'); break;
      case  0xB:  UART_HAL_Putchar(UART0, 'B'); break;
      case  0xC:  UART_HAL_Putchar(UART0, 'C'); break;
      case  0xD:  UART_HAL_Putchar(UART0, 'D'); break;
      case  0xE:  UART_HAL_Putchar(UART0, 'E'); break;
      case  0xF:  UART_HAL_Putchar(UART0, 'F'); break;
    }
    UART_HAL_Putchar(UART0, ' ');
}

size_t HAL_UART::printf(const char *format, ...) {
  char RxBuffer[256];
  va_list vArgs;
  va_start(vArgs, format);
  int length = vsnprintf(RxBuffer, 256, format, vArgs);
  va_end(vArgs);
  
  /* Original function
  if (length > 0 && length < 256) {
    for (size_t i = 0; i < (size_t)length; ++i)
      write(RxBuffer[i]);
  } */

  if (length > 0) {
    for (size_t i = 0; i < (size_t)length; i++)
      write(RxBuffer[i]);
  }

  return length;
}

size_t HAL_UART::write(uint8_t data) {

#if TX_BUFFER_SIZE > 0

  size_t bytes = 0;
  uint32_t fifolvl = 0;

  // If the Tx Buffer is full, wait for space to clear
  if ((TxQueueWritePos+1) % TX_BUFFER_SIZE == TxQueueReadPos) flushTX();

  // Temporarily lock out UART transmit interrupts during this read so the UART transmit interrupt won't
  // cause problems with the index values
  UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, false);

  // Get FIFO level
  fifolvl = UART_HAL_GetTxDatawordCountInFifo(UARTx);

  // If the queue is empty and there's space in the FIFO, immediately send the byte
  if ((TxQueueWritePos == TxQueueReadPos) && (fifolvl < HAL_UART_TX_FIFO_SIZE)) {
    UART_HAL_Putchar(UARTx, data);
  }
  // Otherwiise, write the byte to the transmit buffer
  else if ((TxQueueWritePos+1) % TX_BUFFER_SIZE != TxQueueReadPos) {
    TxBuffer[TxQueueWritePos] = data;
    TxQueueWritePos = (TxQueueWritePos+1) % TX_BUFFER_SIZE;
    bytes++;
  }

  // Re-enable the TX Interrupt
  UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, true);

  return bytes;

#else
  
  while ((UART_HAL_GetTxDatawordCountInFifo(UARTx)) >= (HAL_UART_TX_FIFO_SIZE - 2));      // Wait for FIFO to clear at least one byte
  
  UART_HAL_Putchar(UARTx, data);  // Send byte
  return 1U;  

#endif
}

#if TX_BUFFER_SIZE > 0
  void HAL_UART::flushTX() {
    // Wait for the tx buffer and FIFO to drain
    while ((TxQueueWritePos != TxQueueReadPos) && UART_HAL_GetTxDatawordCountInFifo(UARTx));
  }
#endif

size_t HAL_UART::available() {
  return (RxQueueWritePos + RX_BUFFER_SIZE - RxQueueReadPos) % RX_BUFFER_SIZE;
}

void HAL_UART::flush() {
  RxQueueWritePos = 0;
  RxQueueReadPos = 0;
}

int16_t HAL_UART::peek() {
  int16_t data = -1;

  // Temporarily lock out UART receive interrupts during this read so the UART receive
  // interrupt won't cause problems with the index values
  UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, false);

  if (RxQueueReadPos != RxQueueWritePos)
    data = RxBuffer[RxQueueReadPos];

  // Re-enable UART interrupts
  UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, true);

  return data;
}

int16_t HAL_UART::read() {
  int16_t data = -1;

  // Temporarily lock out UART receive interrupts during this read so the UART receive
  // interrupt won't cause problems with the index values
  UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, false);

  if (RxQueueReadPos != RxQueueWritePos) {
    data = RxBuffer[RxQueueReadPos];
    RxQueueReadPos = (RxQueueReadPos + 1) % RX_BUFFER_SIZE;
  }

  // Re-enable UART interrupts
  UART_HAL_SetIntMode(UARTx, kUartIntTxDataRegEmpty, true);

  return data;
}

HAL_UART_STATUS_t HAL_UART::getStatus(void){
  HAL_UART_STATUS_t status;
  status.baudRate = Serial.baudRate;
  status.RxQueueReadPos = Serial.RxQueueReadPos;
  status.RxQueueWritePos = Serial.RxQueueWritePos;
  #if TX_BUFFER_SIZE > 0
  status.TxQueueReadPos = Serial.TxQueueReadPos;
  status.TxQueueWritePos = Serial.TxQueueWritePos;
  #endif
  return status;
}

#endif
