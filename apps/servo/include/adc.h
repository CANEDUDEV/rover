#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int16_t adc_to_servo_position(uint16_t adc_value);
uint16_t adc_to_servo_current(uint16_t adc_value);
uint16_t adc_to_battery_voltage(uint16_t adc_value);
uint16_t adc_to_servo_voltage(uint16_t adc_value);
uint16_t adc_to_h_bridge_current(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
