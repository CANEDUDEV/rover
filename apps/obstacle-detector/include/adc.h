#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ADC1_NUM_CHANNELS 4
#define ADC_NUM_SAMPLES 10

typedef struct {
  uint16_t adc1_buf[ADC1_NUM_CHANNELS];
} adc_reading_t;

typedef struct {
  uint16_t adc1_buf[ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS];
} adc_samples_t;

void adc_average_samples(adc_reading_t *average,
                         const volatile adc_samples_t *samples);

float adc_value_to_distance(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
