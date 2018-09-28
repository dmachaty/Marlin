#include <fsl_adc16_hal.h>
#include <fsl_sim_hal.h>
#include <fsl_gpio_hal.h>
#include <fsl_port_hal.h>

#define ADC_DONE      0x80000000
#define ADC_OVERRUN   0x40000000

uint8_t activeADC = 0;
uint8_t activeCHN = 0;

void HAL_adc_init(void) {

    adc16_converter_config_t    configConverter;
    adc16_chn_config_t          configChannel;
    adc16_hw_average_config_t   configHwAverage;

    configConverter.clkSrc = kAdc16ClkSrcOfBusClk;                  // Enable clock
    configConverter.clkDividerMode = kAdc16ClkDividerOf8;           // 15MHz @ 120MHz bus clock
    configConverter.refVoltSrc = kAdc16RefVoltSrcOfVref;            // VRef source
    configConverter.hwTriggerEnable = false;                        // Software trigger
    configConverter.continuousConvEnable = false;                   // Single conversion
    configConverter.resolution = kAdc16ResolutionBitOfSingleEndAs10;// 10Bit mode
    ADC16_HAL_ConfigConverter(ADC0, &configConverter);
    ADC16_HAL_ConfigConverter(ADC1, &configConverter);

    configHwAverage.hwAverageCountMode = kAdc16HwAverageCountOf32;
    configHwAverage.hwAverageEnable = true;
    ADC16_HAL_ConfigHwAverage(ADC0, &configHwAverage);
    ADC16_HAL_ConfigHwAverage(ADC1, &configHwAverage);

    SIM_HAL_EnableClock(SIM, kSimClockGateAdc0); // Enable clock
    SIM_HAL_EnableClock(SIM, kSimClockGateAdc1); // Enable clock
}

// externals need to make the call to KILL compile
#include "../../core/language.h"

extern void kill(const char*);
extern const char errormagic[];

void HAL_adc_enable_channel(int ch) {
  pin_t pin = analogInputToDigitalPin(ch);

  if (pin == -1) {
    SERIAL_PRINTF("%sINVALID ANALOG PORT:%d\n", errormagic, ch);
    kill(MSG_KILLED);
  }

    // All 6 analog pins on PORTB
    PORT_HAL_SetMuxMode(PORTB, MK64F12_PIN_PIN(pin), kPortPinDisabled); // Set as ADC pin
}

void HAL_adc_start_conversion(const uint8_t ch) {
  if (analogInputToDigitalPin(ch) == -1) {
    SERIAL_PRINTF("HAL: HAL_adc_start_conversion: invalid channel %d\n", ch);
    return;
  }

  adc16_chn_t channel;
  ADC_Type * adc;

    switch(ch){
        case 0: {
                    channel = kAdc16Chn8;
                    adc = ADC0;
                    activeADC = 0;
                    activeCHN = 0; }
        case 1: {
                    channel = kAdc16Chn9;
                    adc = ADC0;
                    activeADC = 0;
                    activeCHN = 1; }
        case 2: {
                    channel = kAdc16Chn12;
                    adc = ADC0;
                    activeADC = 0;
                    activeCHN = 2; }
        case 3: {
                    channel = kAdc16Chn13;
                    adc = ADC0;
                    activeADC = 0;
                    activeCHN = 3; }
        case 4: {
                    channel = kAdc16Chn14;
                    adc = ADC1;
                    activeADC = 1;
                    activeCHN = 4; }
        case 5: {
                    channel = kAdc16Chn15;
                    adc = ADC1;
                    activeADC = 1;
                    activeCHN = 5; }
    }

    adc16_chn_config_t config;
    config.chnIdx = channel;
    config.convCompletedIntEnable = false;
    config.diffConvEnable = false;
    ADC16_HAL_ConfigChn(adc, 0, &config);
}

bool HAL_adc_finished(void) {
    // Two ADCs, both must be done (don't know which one is used)
    bool conv_done;
    if( (ADC16_HAL_GetConvActiveFlag(ADC0) == 0)&&(ADC16_HAL_GetConvActiveFlag(ADC1) == 0) )
        conv_done = true;
    else
        conv_done = false;
    return conv_done;
}

// possible config options if something similar is extended to more platforms.
#define ADC_USE_MEDIAN_FILTER          // Filter out erroneous readings
#define ADC_MEDIAN_FILTER_SIZE     23  // Higher values increase step delay (phase shift),
                                       // (ADC_MEDIAN_FILTER_SIZE + 1) / 2 sample step delay (12 samples @ 500Hz: 24ms phase shift)
                                       // Memory usage per ADC channel (bytes): (6 * ADC_MEDIAN_FILTER_SIZE) + 16
                                       // 8 * ((6 * 23) + 16 ) = 1232 Bytes for 8 channels

#define ADC_USE_LOWPASS_FILTER         // Filter out high frequency noise
#define ADC_LOWPASS_K_VALUE         6  // Higher values increase rise time
                                       // Rise time sample delays for 100% signal convergence on full range step
                                       // (1 : 13, 2 : 32, 3 : 67, 4 : 139, 5 : 281, 6 : 565, 7 : 1135, 8 : 2273)
                                       // K = 6, 565 samples, 500Hz sample rate, 1.13s convergence on full range step
                                       // Memory usage per ADC channel (bytes): 4 (32 Bytes for 8 channels)


// Sourced from https://embeddedgurus.com/stack-overflow/tag/median-filter/
struct MedianFilter {
  #define STOPPER 0                // Smaller than any datum
  struct Pair {
    Pair   *point;                 // Pointers forming list linked in sorted order
    uint16_t  value;               // Values to sort
  };

  Pair buffer[ADC_MEDIAN_FILTER_SIZE] = {}; // Buffer of nwidth pairs
  Pair *datpoint = buffer;                  // Pointer into circular buffer of data
  Pair small = {NULL, STOPPER};             // Chain stopper
  Pair big = {&small, 0};                   // Pointer to head (largest) of linked list.

  uint16_t update(uint16_t datum) {
    Pair *successor;                        // Pointer to successor of replaced data item
    Pair *scan;                             // Pointer used to scan down the sorted list
    Pair *scanold;                          // Previous value of scan
    Pair *median;                           // Pointer to median
    uint16_t i;

    if (datum == STOPPER) {
      datum = STOPPER + 1;                  // No stoppers allowed.
    }

    if ( (++datpoint - buffer) >= (ADC_MEDIAN_FILTER_SIZE)) {
      datpoint = buffer;                    // Increment and wrap data in pointer.
    }

    datpoint->value = datum;                // Copy in new datum
    successor = datpoint->point;            // Save pointer to old value's successor
    median = &big;                          // Median initially to first in chain
    scanold = NULL;                         // Scanold initially null.
    scan = &big;                            // Points to pointer to first (largest) datum in chain

    // Handle chain-out of first item in chain as special case
    if (scan->point == datpoint) {
      scan->point = successor;
    }
    scanold = scan;                         // Save this pointer and
    scan = scan->point ;                    // step down chain

    // Loop through the chain, normal loop exit via break.
    for (i = 0 ; i < ADC_MEDIAN_FILTER_SIZE; ++i) {
      // Handle odd-numbered item in chain
      if (scan->point == datpoint) {
        scan->point = successor;            // Chain out the old datum
      }

      if (scan->value < datum) {            // If datum is larger than scanned value
        datpoint->point = scanold->point;   // Chain it in here
        scanold->point = datpoint;          // Mark it chained in
        datum = STOPPER;
      }

      // Step median pointer down chain after doing odd-numbered element
      median = median->point;               // Step median pointer
      if (scan == &small) {
        break;                              // Break at end of chain
      }
      scanold = scan;                       // Save this pointer and
      scan = scan->point;                   // step down chain

      // Handle even-numbered item in chain.
      if (scan->point == datpoint) {
        scan->point = successor;
      }

      if (scan->value < datum) {
        datpoint->point = scanold->point;
        scanold->point = datpoint;
        datum = STOPPER;
      }

      if (scan == &small) {
        break;
      }

      scanold = scan;
      scan = scan->point;
    }
    return median->value;
  }
};

struct LowpassFilter {
  uint32_t data_delay = 0;
  uint16_t update(const uint16_t value) {
    data_delay -= (data_delay >> (ADC_LOWPASS_K_VALUE)) - value;
    return (uint16_t)(data_delay >> (ADC_LOWPASS_K_VALUE));
  }
};

uint16_t HAL_adc_get_result(void) {

    uint32_t data;
    if(activeADC == 0)  data = ADC16_HAL_GetChnConvValue(ADC0, 0);
    else                data = ADC16_HAL_GetChnConvValue(ADC1, 0);

    if (data & ADC_OVERRUN) return 0;
    
  #ifdef ADC_USE_MEDIAN_FILTER
    static MedianFilter median_filter[NUM_ANALOG_INPUTS];
    data = median_filter[activeCHN].update(data);
  #endif

  #ifdef ADC_USE_LOWPASS_FILTER
    static LowpassFilter lowpass_filter[NUM_ANALOG_INPUTS];
    data = lowpass_filter[activeCHN].update(data);
  #endif

  return (data & 0x3FF);    // return 10bit value as Marlin expects
}