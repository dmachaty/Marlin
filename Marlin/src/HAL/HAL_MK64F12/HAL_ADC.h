// ADC
#define HAL_ANALOG_SELECT(pin) HAL_adc_enable_channel(pin)
#define HAL_START_ADC(pin)     HAL_adc_start_conversion(pin)
#define HAL_READ_ADC()         HAL_adc_get_result()
#define HAL_ADC_READY()        HAL_adc_finished()

void HAL_adc_init(void);
void HAL_adc_enable_channel(int pin);
void HAL_adc_start_conversion(const uint8_t adc_pin);
uint16_t HAL_adc_get_result(void);
bool HAL_adc_finished(void);