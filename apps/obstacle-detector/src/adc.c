#include "adc.h"

#define ADC_RESOLUTION_12BIT ((1 << 12) - 1)

void adc_average_samples(adc_reading_t *average,
                         const volatile adc_samples_t *samples) {
  uint32_t sum_adc1[ADC1_NUM_CHANNELS] = {0};

  for (uint32_t i = 0; i < ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS; i++) {
    sum_adc1[i % ADC1_NUM_CHANNELS] += samples->adc1_buf[i];
  }

  for (uint32_t i = 0; i < ADC1_NUM_CHANNELS; i++) {
    average->adc1_buf[i] = sum_adc1[i] / ADC_NUM_SAMPLES;
  }
}

// Datasheet says multiply by 6 for 10-bit ADC.
// Since we have 12 bit ADC, we divide by 4 to convert to 10 bit.
float adc_value_to_distance(uint16_t adc_value) {
  const float scale_factor = 6 / 4.0F;
  const float offset = -300.0F;

  float distance = (scale_factor * (float)adc_value) + offset;
  if (distance < 0) {
    distance = 0;
  }

  return distance;
}
