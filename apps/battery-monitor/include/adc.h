#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ADC1_NUM_CHANNELS 4
#define ADC2_NUM_CHANNELS 6
#define ADC_NUM_SAMPLES 100

typedef struct {
  uint16_t adc1_buf[ADC1_NUM_CHANNELS];
  uint16_t adc2_buf[ADC2_NUM_CHANNELS];
} adc_reading_t;

typedef struct {
  uint16_t adc1_buf[ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS];
  uint16_t adc2_buf[ADC_NUM_SAMPLES * ADC2_NUM_CHANNELS];
} adc_samples_t;

void adc_average_samples(adc_reading_t *average,
                         const volatile adc_samples_t *samples);

uint16_t adc_to_cell_voltage(uint16_t adc_value);
uint16_t adc_to_reg_out_current(uint16_t adc_value);
uint16_t adc_to_reg_out_voltage(uint16_t adc_value);
uint32_t adc_to_vbat_out_current(uint16_t adc_value);
uint16_t adc_to_vbat_out_voltage(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
