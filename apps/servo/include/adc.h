#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*******************************************************
 * Convert ADC reading to a sensor power CAN message.
 *
 * @param adc_value The raw ADC reading.
 * @param frame CAN frame populated by the function.
 ******************************************************/
uint16_t adc_to_sensor_power(uint16_t adc_value);
uint16_t adc_to_servo_current(uint16_t adc_value);
uint16_t adc_to_battery_voltage(uint16_t adc_value);
uint16_t adc_to_servo_voltage(uint16_t adc_value);
uint16_t adc_to_h_bridge_current(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
