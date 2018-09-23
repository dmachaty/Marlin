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

// System
// Framework
#include <fsl_clock_manager.h>
#include <fsl_sim_hal.h>
#include <fsl_smc_hal.h>
// Project
#include "HAL_Board_Clock.h"

extern uint64_t _millis;

/*
 * This configuration uses external 50MHz clock, driven to PLL generator, producing 120MHz system clock.
 * MCU starts in FEI mode (FLL from Internal RC), run mode is PEE (PLL from external clock).
 * 
 */
const clock_manager_user_config_t halboardclock_CMConfig =
{
    .mcgConfig =
    {
        // Enable PEE Mode (PLL from External clock)
        .mcg_mode           = kMcgModePEE,
        
        // Enable Internal RC clock @ 32k
        .irclkEnable        = true,         // MCGIRCLK 4MHz RC clock generator enable
        .irclkEnableInStop  = false,        // MCGIRCLK 4MHz RC clock generator disable in stop mode
        .ircs               = kMcgIrcSlow,  // MCGIRCLK Set to 32k mode
        .fcrdiv             = 0U,           // MCGIRCLK Divider (FCRDIV) 0 (/1)

        // Enable FLL @ 20.833MHz (FLL factor 640 when drs=low, dmx32=0, clk=50MHz/1536)
        .frdiv              = 7U,           // FLL Divider (FRDIV) 7 (/1536) = 32.552kHz (range 31.25 .. 39.0625 kHz)
        .drs                = kMcgDcoRangeSelLow,   // DCO Range Low
        .dmx32              = kMcgDmx32Default,     // DCO Default range 25%
        .oscsel             = kMcgOscselOsc,        // DCO Source from OSC

        // Enable PLL @
        .pll0EnableInFllMode = false,               // PLL0 disable in FLL Mode (only transition)
        .pll0EnableInStop    = false,               // PLL0 disable in STOP mode
        .prdiv0              = 0x13U,               // N=20 -> VCO=2.5MHz (Range 2..4MHz)
        .vdiv0               = 0x18U,               // M=48 (VDIV=24..55) -> PLLOUT=120MHz
    },
    .oscerConfig =
    {
        .enable       = true,               // Enable OSCERCLK (External reference clock)
        .enableInStop = false,              // Disable OSCERCLK in stop mode
    },
    .simConfig =
    {
        .pllFllSel = kClockPllFllSelPll,    // Use PLL as a main clock source
        .er32kSrc  = kClockEr32kSrcRtc,     // Use RTC as 32kHz clock source
        .outdiv1   = 0U,                    // Core clock divider       /1 = 120Mhz (max 120MHz)
        .outdiv2   = 1U,                    // Bus clock divider        /2 = 60MHz  (max 60MHz)
        .outdiv3   = 2U,                    // FlexBus clock divider    /4 = 30MHz  (max 50MHz)
        .outdiv4   = 4U                     // Flash clock divider      /5 = 24MHz  (max 25MHz)
    }
};

// Main oscillator configuration (OSC0 50MHz input)
osc_user_config_t halboardclock_OSCConfig =
{
    .freq                = 50000000U,           // External 50MHz clock
    .enableCapacitor2p   = false,               // No load caps
    .enableCapacitor4p   = false,               // No load caps
    .enableCapacitor8p   = false,               // No load caps
    .enableCapacitor16p  = false,               // No load caps
    .hgo                 = kOscGainLow,         // Low gain oscillator
    .range               = kOscRangeVeryHigh,   // Range 8..32MHz, 50MHz in ext clk mode
    .erefs               = kOscSrcExt           // External clock mode instead of oscillator
};

// RTC oscillator configuration (32KHz crystal)
rtc_osc_user_config_t halboardclock_RTCConfig =
{
        .freq                = 32768U,
        .enableCapacitor2p   = false,
        .enableCapacitor4p   = false,
        .enableCapacitor8p   = false,
        .enableCapacitor16p  = false,
        .enableOsc           = true
};

// Enable all port clocks
void HAL_Board_Clock_EnableAllPortClocks(void){
    CLOCK_SYS_EnablePortClock(PORTA_IDX);
    CLOCK_SYS_EnablePortClock(PORTB_IDX);
    CLOCK_SYS_EnablePortClock(PORTC_IDX);
    CLOCK_SYS_EnablePortClock(PORTD_IDX);
    CLOCK_SYS_EnablePortClock(PORTE_IDX);
}

void HAL_Board_Clock_Init(void){
    
    // Basic clocking initialization
    HAL_Board_Clock_EnableAllPortClocks();                              // Enable clock to all ports
    SMC_HAL_SetProtection(SMC, 0);                                      // Prevent any low power modes
    CLOCK_SYS_OscInit(OSC_IDX, &halboardclock_OSCConfig);               // Configure OSC0 oscillator
    CLOCK_SYS_RtcOscInit(RTC_IDX, &halboardclock_RTCConfig);            // Configure RTC oscillator
    CLOCK_SYS_SetSimConfigration(&halboardclock_CMConfig.simConfig);    // Configure SIM module
    CLOCK_SYS_BootToFee(&halboardclock_CMConfig.mcgConfig);             // Swith clock configuration
    SystemCoreClockUpdate();                                            // Make sure CPU_XTAL_CLK_HZ is correct

    // Peripheral clocking initialization
    CLOCK_SYS_EnableUartClock(UART0_IDX);

    _millis = 0;                            // Initialize the millisecond counter value
    SysTick_Config(SystemCoreClock / 1000); // Start millisecond global counter
}

extern "C" {
  void SysTick_Handler(void){
    _millis++;
  }
}