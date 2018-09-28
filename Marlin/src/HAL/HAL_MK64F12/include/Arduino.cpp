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

#ifdef TARGET_MK64FN1M0

#include "../HAL_PWM.h"
#include "../HAL_IO.h"
#include "../HAL_ADC.h"

#include "../../../inc/MarlinConfig.h"
#include "../../shared/Delay.h"

// Interrupts
void cli(void) { __disable_irq(); } // Disable
void sei(void) { __enable_irq(); }  // Enable

// Time functions
void _delay_ms(const int delay_ms) {
  delay(delay_ms);
}

extern uint32_t _millis;
uint32_t millis() {
  return _millis;
}

// This is required for some Arduino libraries we are using
void delayMicroseconds(uint32_t us) {
  DELAY_US(us);
}

extern "C" void delay(const int msec) {
  volatile millis_t end = _millis + msec;
  SysTick->VAL = SysTick->LOAD; // reset systick counter so next systick is in exactly 1ms
                                // this could extend the time between systicks by upto 1ms
  while PENDING(_millis, end) __WFE();
}

// IO functions
// As defined by Arduino INPUT(0x0), OUTPUT(0x1), INPUT_PULLUP(0x2)
void pinMode(const pin_t pin, const uint8_t mode) {
  if (!VALID_PIN(pin)) return;

  switch (mode) {
    case INPUT:
      MK64F12_GPIO(MK64F12_PIN_PORT(pin))->PDDR &= ~PIN_BIT(MK64F12_PIN_PIN(pin));
      break;
    case OUTPUT:
      MK64F12_GPIO(MK64F12_PIN_PORT(pin))->PDDR |=  PIN_BIT(MK64F12_PIN_PIN(pin));
      break;
    case INPUT_PULLUP:
      MK64F12_GPIO(MK64F12_PIN_PORT(pin))->PDDR &= ~PIN_BIT(MK64F12_PIN_PIN(pin));
      MK64F12_PORT(MK64F12_PIN_PORT(pin))->PCR[MK64F12_PIN_PIN(pin)]  |= 0x03;  // PS (0x01) || PE (0x02)
      break;
    case INPUT_PULLDOWN:
      MK64F12_GPIO(MK64F12_PIN_PORT(pin))->PDDR &= ~PIN_BIT(MK64F12_PIN_PIN(pin));
      MK64F12_PORT(MK64F12_PIN_PORT(pin))->PCR[MK64F12_PIN_PIN(pin)]  |= 0x02;  // !PS (0x00) || PE (0x02)
      break;
    default: return;
  }
}

void digitalWrite(pin_t pin, uint8_t pin_status) {
  if (!VALID_PIN(pin)) return;

  if (pin_status)
    MK64F12_GPIO(MK64F12_PIN_PORT(pin))->PSOR = PIN_BIT(MK64F12_PIN_PIN(pin));
  else
    MK64F12_GPIO(MK64F12_PIN_PORT(pin))->PCOR = PIN_BIT(MK64F12_PIN_PIN(pin));

  pinMode(pin, OUTPUT);  // Set pin mode on every write (Arduino version does this)
}

bool digitalRead(pin_t pin) {
  if (!VALID_PIN(pin)) return false;

  return MK64F12_GPIO(MK64F12_PIN_PORT(pin))->PDIR & PIN_BIT(MK64F12_PIN_PIN(pin)) ? 1 : 0;
}

/* FUUUUUUUUUUUCK
void analogWrite(pin_t pin, int pwm_value) {  // 1 - 254: pwm_value, 0: LOW, 255: HIGH
  if (!VALID_PIN(pin)) return;

  #define MR0_MARGIN 200       // if channel value too close to MR0 the system locks up

  static bool out_of_PWM_slots = false;

  uint value = MAX(MIN(pwm_value, 255), 0);
  if (value == 0 || value == 255) {  // treat as digital pin
    MK64F12_PWM_detach_pin(pin);    // turn off PWM
    digitalWrite(pin, value);
  }
  else {
    if (MK64F12_PWM_attach_pin(pin, 1, LPC_PWM1->MR0, 0xFF))
      LPC1768_PWM_write(pin, map(value, 0, 255, 1, LPC_PWM1->MR0));  // map 1-254 onto PWM range
    else {                                                                 // out of PWM channels
      if (!out_of_PWM_slots) SERIAL_ECHOPGM(".\nWARNING - OUT OF PWM CHANNELS\n.\n");  //only warn once
      out_of_PWM_slots = true;
      digitalWrite(pin, value);  // treat as a digital pin if out of channels
    }
  }
}
*/

extern bool HAL_adc_finished();

uint16_t analogRead(pin_t adc_pin) {
  HAL_adc_start_conversion(adc_pin);
  while (!HAL_adc_finished());  // Wait for conversion to finish
  return HAL_adc_get_result();
}

// **************************
// Persistent Config Storage
// **************************

void eeprom_write_byte(unsigned char *pos, unsigned char value) {

}

unsigned char eeprom_read_byte(uint8_t * pos) { return '\0'; }

void eeprom_read_block(void *__dst, const void *__src, size_t __n) { }

void eeprom_update_block(const void *__src, void *__dst, size_t __n) { }

char *dtostrf (double __val, signed char __width, unsigned char __prec, char *__s) {
  char format_string[20];
  snprintf(format_string, 20, "%%%d.%df", __width, __prec);
  sprintf(__s, format_string, __val);
  return __s;
}

int32_t random(int32_t max) {
  return rand() % max;
}

int32_t random(int32_t min, int32_t max) {
  return min + rand() % (max - min);
}

void randomSeed(uint32_t value) {
  srand(value);
}

int map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // TARGET_LPC1768
