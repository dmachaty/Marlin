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
#include <fsl_gpio_hal.h>
#include <fsl_port_hal.h>
#include <fsl_clock_manager.h>
#include <MK64F12.h>
//  Project
#include "HAL_Board_Pins.h"

void HAL_Board_Pins_SetupGPIOs(void){
    /*
     * For multiplex alt functions, see Reference Manual,
     * chapter 5.1 K64 Signal Multiplexing and Pin Assignments
     */

    /**********************************************************
     * General purpose IO Pins
     *********************************************************/
    // LEDs
    PORT_HAL_SetMuxMode(PORTB, 21, kPortMuxAsGpio); // LED Blue
    PORT_HAL_SetMuxMode(PORTB, 22, kPortMuxAsGpio); // LED Red
    PORT_HAL_SetMuxMode(PORTE, 26, kPortMuxAsGpio); // LED Green
    GPIO_HAL_SetPinDir(PTB, 21, kGpioDigitalOutput); // LED Blue
    GPIO_HAL_SetPinDir(PTB, 22, kGpioDigitalOutput); // LED Red
    GPIO_HAL_SetPinDir(PTE, 26, kGpioDigitalOutput); // LED Green
    
    // Printer control signals
    PORT_HAL_SetMuxMode(PORTB, 18, kPortMuxAsGpio); // X_MIN_PIN           
    PORT_HAL_SetMuxMode(PORTC, 16, kPortMuxAsGpio); // X_MAX_PIN           
    PORT_HAL_SetMuxMode(PORTB, 19, kPortMuxAsGpio); // Y_MIN_PIN           
    PORT_HAL_SetMuxMode(PORTC, 17, kPortMuxAsGpio); // Y_MAX_PIN           
    PORT_HAL_SetMuxMode(PORTC, 1 , kPortMuxAsGpio); // Z_MIN_PIN           
    PORT_HAL_SetMuxMode(PORTB, 9 , kPortMuxAsGpio); // Z_MAX_PIN           
    PORT_HAL_SetMuxMode(PORTA, 1 , kPortMuxAsGpio); // X_STEP_PIN          
    PORT_HAL_SetMuxMode(PORTC, 9 , kPortMuxAsGpio); // X_DIR_PIN           
    PORT_HAL_SetMuxMode(PORTB, 23, kPortMuxAsGpio); // X_ENABLE_PIN        
    PORT_HAL_SetMuxMode(PORTC, 0 , kPortMuxAsGpio); // Y_STEP_PIN          
    PORT_HAL_SetMuxMode(PORTA, 2 , kPortMuxAsGpio); // Y_DIR_PIN           
    PORT_HAL_SetMuxMode(PORTC, 7 , kPortMuxAsGpio); // Y_ENABLE_PIN        
    PORT_HAL_SetMuxMode(PORTC, 2 , kPortMuxAsGpio); // Z_STEP_PIN          
    PORT_HAL_SetMuxMode(PORTC, 5 , kPortMuxAsGpio); // Z_DIR_PIN           
    PORT_HAL_SetMuxMode(PORTC, 3 , kPortMuxAsGpio); // Z_ENABLE_PIN        
    PORT_HAL_SetMuxMode(PORTE, 26, kPortMuxAsGpio); // E0_STEP_PIN         
    PORT_HAL_SetMuxMode(PORTC, 12, kPortMuxAsGpio); // E0_DIR_PIN          
    PORT_HAL_SetMuxMode(PORTC, 4 , kPortMuxAsGpio); // E0_ENABLE_PIN       
    PORT_HAL_SetMuxMode(PORTC, 11, kPortMuxAsGpio); // E1_STEP_PIN         
    PORT_HAL_SetMuxMode(PORTC, 10, kPortMuxAsGpio); // E1_DIR_PIN          
    PORT_HAL_SetMuxMode(PORTB, 20, kPortMuxAsGpio); // E1_ENABLE_PIN       
    PORT_HAL_SetMuxMode(PORTC, 18, kPortMuxAsGpio); // HEATER_BED_PIN      
    PORT_HAL_SetMuxMode(PORTD, 4 , kPortMuxAsGpio); // HEATER_0_PIN        
    PORT_HAL_SetMuxMode(PORTD, 5 , kPortMuxAsGpio); // HEATER_1_PIN        
    PORT_HAL_SetMuxMode(PORTD, 6 , kPortMuxAsGpio); // FAN_PIN             
    PORT_HAL_SetMuxMode(PORTD, 7 , kPortMuxAsGpio); // FAN1_PIN            

    GPIO_HAL_SetPinDir(PTB, 18, kGpioDigitalInput); // X_MIN_PIN           
    GPIO_HAL_SetPinDir(PTC, 16, kGpioDigitalInput); // X_MAX_PIN           
    GPIO_HAL_SetPinDir(PTB, 19, kGpioDigitalInput); // Y_MIN_PIN           
    GPIO_HAL_SetPinDir(PTC, 17, kGpioDigitalInput); // Y_MAX_PIN           
    GPIO_HAL_SetPinDir(PTC, 1 , kGpioDigitalInput); // Z_MIN_PIN           
    GPIO_HAL_SetPinDir(PTB, 9 , kGpioDigitalInput); // Z_MAX_PIN           
    GPIO_HAL_SetPinDir(PTA, 1 , kGpioDigitalOutput); // X_STEP_PIN          
    GPIO_HAL_SetPinDir(PTC, 9 , kGpioDigitalOutput); // X_DIR_PIN           
    GPIO_HAL_SetPinDir(PTB, 23, kGpioDigitalOutput); // X_ENABLE_PIN        
    GPIO_HAL_SetPinDir(PTC, 0 , kGpioDigitalOutput); // Y_STEP_PIN          
    GPIO_HAL_SetPinDir(PTA, 2 , kGpioDigitalOutput); // Y_DIR_PIN           
    GPIO_HAL_SetPinDir(PTC, 7 , kGpioDigitalOutput); // Y_ENABLE_PIN        
    GPIO_HAL_SetPinDir(PTC, 2 , kGpioDigitalOutput); // Z_STEP_PIN          
    GPIO_HAL_SetPinDir(PTC, 5 , kGpioDigitalOutput); // Z_DIR_PIN           
    GPIO_HAL_SetPinDir(PTC, 3 , kGpioDigitalOutput); // Z_ENABLE_PIN        
    GPIO_HAL_SetPinDir(PTE, 26, kGpioDigitalOutput); // E0_STEP_PIN         
    GPIO_HAL_SetPinDir(PTC, 12, kGpioDigitalOutput); // E0_DIR_PIN          
    GPIO_HAL_SetPinDir(PTC, 4 , kGpioDigitalOutput); // E0_ENABLE_PIN       
    GPIO_HAL_SetPinDir(PTC, 11, kGpioDigitalOutput); // E1_STEP_PIN         
    GPIO_HAL_SetPinDir(PTC, 10, kGpioDigitalOutput); // E1_DIR_PIN          
    GPIO_HAL_SetPinDir(PTB, 20, kGpioDigitalOutput); // E1_ENABLE_PIN       
    GPIO_HAL_SetPinDir(PTC, 18, kGpioDigitalOutput); // HEATER_BED_PIN      
    GPIO_HAL_SetPinDir(PTD, 4 , kGpioDigitalOutput); // HEATER_0_PIN        
    GPIO_HAL_SetPinDir(PTD, 5 , kGpioDigitalOutput); // HEATER_1_PIN        
    GPIO_HAL_SetPinDir(PTD, 6 , kGpioDigitalOutput); // FAN_PIN             
    GPIO_HAL_SetPinDir(PTD, 7 , kGpioDigitalOutput); // FAN1_PIN    

    /**********************************************************
     * Analog Pins
     *********************************************************/
    PORT_HAL_SetMuxMode(PORTB,  2, kPortPinDisabled);  // TEMP_0_PIN   - ADC0_SE12
    PORT_HAL_SetMuxMode(PORTB,  3, kPortPinDisabled);  // TEMP_BED_PIN - ADC0_SE13 
    PORT_HAL_SetMuxMode(PORTB, 10, kPortPinDisabled);  // TEMP_1_PIN   - ADC1_SE14 
    PORT_HAL_SetMuxMode(PORTB, 11, kPortPinDisabled);  // TEMP_2_PIN   - ADC1_SE15

    /**********************************************************
     * UART Pins
     *********************************************************/
    PORT_HAL_SetMuxMode(PORTB, 17, kPortMuxAlt3); // UART TX
    PORT_HAL_SetMuxMode(PORTB, 16, kPortMuxAlt3); // UART RX

    /**********************************************************
     * Clock pins
     *********************************************************/
    PORT_HAL_SetMuxMode(PORTA, 18, kPortPinDisabled); // External 50MHz input
    
    /**********************************************************
     * SPI Pins
     *********************************************************/
    PORT_HAL_SetMuxMode(PORTD, 0, kPortMuxAlt2); // SPI Pin CS0
    PORT_HAL_SetMuxMode(PORTD, 1, kPortMuxAlt2); // SPI Pin CLOCK
    PORT_HAL_SetMuxMode(PORTD, 2, kPortMuxAlt2); // SPI Pin SOUT
    PORT_HAL_SetMuxMode(PORTD, 3, kPortMuxAlt2); // SPI Pin SIN
    
    /**********************************************************
     * SDHC Pins
     *********************************************************/
    PORT_HAL_SetMuxMode(PORTE, 0, kPortMuxAlt4); // SDHC Pin D1
    PORT_HAL_SetMuxMode(PORTE, 1, kPortMuxAlt4); // SDHC Pin D0
    PORT_HAL_SetMuxMode(PORTE, 2, kPortMuxAlt4); // SDHC Pin CLK
    PORT_HAL_SetMuxMode(PORTE, 3, kPortMuxAlt4); // SDHC Pin CMD
    PORT_HAL_SetMuxMode(PORTE, 4, kPortMuxAlt4); // SDHC Pin D3
    PORT_HAL_SetMuxMode(PORTE, 5, kPortMuxAlt4); // SDHC Pin D2
    PORT_HAL_SetMuxMode(PORTE, 6, kPortMuxAsGpio); // Presence detect as GPIO
}

//  Set GPIOs to safe state if necesarry
void HAL_Board_Pins_SetupGPIODefaults(void){
    GPIO_HAL_WritePinOutput(PTB, 21, 1);    // Turn LED Off
    GPIO_HAL_WritePinOutput(PTB, 22, 1);    // Turn LED Off
    GPIO_HAL_WritePinOutput(PTE, 26, 1);    // Turn LED Off
}

// Enable port clocks, set GPIOs, set default values
void HAL_Board_Pins_Init(void){
    HAL_Board_Pins_SetupGPIOs();
    HAL_Board_Pins_SetupGPIODefaults();
}