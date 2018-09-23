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
#ifndef _HAL_UART_H
#define _HAL_UART_H

#include <MK64F12.h>

#define HAL_UART_TX_FIFO_SIZE   8   // Do not modify, fixed value
#define HAL_UART_RX_FIFO_SIZE   8   // Do not modify, fixed value

//#define TX_BUFFER_SIZE  128
#define RX_BUFFER_SIZE  128

#define HAL_UART_UART0_DEFAULT_PARITY       kUartParityDisabled
#define HAL_UART_UART0_DEFAULT_DATABITS     kUart8BitsPerChar
#define HAL_UART_UART0_DEFAULT_STOPBITS     kUartOneStopBit

void HAL_UART_Init(void);

struct HAL_UART_STATUS_t {
    uint32_t    baudRate;
    uint32_t    RxQueueReadPos;
    uint32_t    RxQueueWritePos;
    
    #if TX_BUFFER_SIZE > 0
    uint32_t    TxQueueReadPos;
    uint32_t    TxQueueWritePos;
    #endif
};



class HAL_UART {
    private:
        UART_Type   *UARTx;
        uint32_t    baudRate;
        uint8_t     RxBuffer[RX_BUFFER_SIZE];
        uint32_t    RxQueueWritePos;
        uint32_t    RxQueueReadPos;
    #if TX_BUFFER_SIZE > 0
        uint8_t     TxBuffer[TX_BUFFER_SIZE];
        uint32_t    TxQueueWritePos;
        uint32_t    TxQueueReadPos;
    #endif
    public:
        HAL_UART(UART_Type * UARTx)
        :   UARTx(UARTx),
            baudRate(115200),
            RxQueueWritePos(0),
            RxQueueReadPos(0)
        #if TX_BUFFER_SIZE > 0
            , TxQueueWritePos(0)
            , TxQueueReadPos(0)
        #endif
        {}
        void begin(uint32_t baudRate);
        void printHex(uint8_t data);
        size_t printf(const char *format, ...);
        void IRQHandler(UART_Type * uartInstance);
        void IRQErrHandler(UART_Type * uartInstance);
        size_t write(uint8_t data);

        #if TX_BUFFER_SIZE > 0
            void flushTX(void);
        #endif

        size_t available(void);
        void flush(void);
        int16_t peek(void);
        int16_t read(void);
        HAL_UART_STATUS_t getStatus(void);
};

#endif // _HAL_UART_H