#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ADC1_NUM_CHANNELS 4
#define ADC2_NUM_CHANNELS 4

typedef struct {
  uint16_t adc1_buf[ADC1_NUM_CHANNELS];
  uint16_t adc2_buf[ADC2_NUM_CHANNELS];
} adc_reading_t;

/*
 * X11 and X12 jumper configuration
 * | X11 | X12 | Rout      |
 * |-----|-----|-----------|
 * | OFF | OFF | 10.2 kOhm |
 * | ON  | OFF | 5.1 kOhm  |
 * | OFF | ON  | 3.4 kOhm  |
 * | ON  | ON  | 2.55 kOhm |
 */
typedef enum {
  ALL_OFF,
  X11_ON,
  X12_ON,
  ALL_ON,
} jumper_config_t;

uint16_t adc_to_cell_voltage(uint16_t adc_value);
uint16_t adc_to_reg_out_current(uint16_t adc_value);
uint32_t adc_to_vbat_out_current(uint16_t adc_value);
void set_jumper_config(jumper_config_t jumper_config);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
