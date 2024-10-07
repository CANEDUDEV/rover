#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ADC1_NUM_CHANNELS 5
#define ADC2_NUM_CHANNELS 6
#define ADC_NUM_SAMPLES 100

// Max number of battery cells supported
#define BATTERY_CELLS_MAX 6

typedef struct {
  uint16_t adc1_buf[ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS];
  uint16_t adc2_buf[ADC_NUM_SAMPLES * ADC2_NUM_CHANNELS];
} adc_samples_t;

typedef struct {
  uint16_t cells[BATTERY_CELLS_MAX];
  uint32_t reg_out_current;
  uint32_t reg_out_voltage;
  uint32_t vbat_out_voltage;
  uint32_t vbat_out_current;
} adc_reading_t;

void adc_average_samples(const adc_samples_t *samples, adc_reading_t *reading);

uint16_t adc_to_cell_voltage(uint16_t adc_value);
uint16_t adc_to_reg_out_current(uint16_t adc_value);
uint16_t adc_to_reg_out_voltage(uint16_t adc_value);
uint32_t adc_to_vbat_out_current(uint16_t adc_value);
uint16_t adc_to_vbat_out_voltage(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
