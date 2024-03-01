#include "adc.h"

#include <math.h>

#include "jumpers.h"
#include "lt6106.h"
#include "voltage-divider.h"

#define ADC_REF_VOLTAGE_MV 3300
#define ADC_RESOLUTION_12BIT ((1 << 12) - 1)

static float adc_value_to_voltage(uint16_t adc_value);

void adc_average_samples(adc_reading_t *average,
                         const volatile adc_samples_t *samples) {
  uint32_t sum_adc1[ADC1_NUM_CHANNELS] = {0};
  uint32_t sum_adc2[ADC2_NUM_CHANNELS] = {0};

  for (uint32_t i = 0; i < ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS; i++) {
    sum_adc1[i % ADC1_NUM_CHANNELS] += samples->adc1_buf[i];
  }

  for (uint8_t j = 0; j < ADC1_NUM_CHANNELS; j++) {
    average->adc1_buf[j] = sum_adc1[j] / ADC_NUM_SAMPLES;
  }

  for (uint32_t i = 0; i < ADC_NUM_SAMPLES * ADC2_NUM_CHANNELS; i++) {
    sum_adc2[i % ADC2_NUM_CHANNELS] += samples->adc2_buf[i];
  }

  for (uint8_t j = 0; j < ADC2_NUM_CHANNELS; j++) {
    average->adc2_buf[j] = sum_adc2[j] / ADC_NUM_SAMPLES;
  }
}

uint16_t adc_to_cell_voltage(uint16_t adc_value) {
  const voltage_divider_t divider = {
      .r1 = 13000,
      .r2 = 1600,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  float cell_voltage =
      reverse_voltage_division(measured_output_voltage, &divider);
  return (uint16_t)roundf(cell_voltage);
}

uint16_t adc_to_reg_out_current(uint16_t adc_value) {
  const lt6106_current_sensor_t sensor = {
      .r_in = 51,
      .r_out = 5100,
      .r_sense = 0.005F,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  float i_sense = lt6106_sense_current(measured_output_voltage, &sensor);
  return (uint16_t)roundf(i_sense);
}

uint16_t adc_to_reg_out_voltage(uint16_t adc_value) {
  const voltage_divider_t divider = {
      .r1 = 47000,
      .r2 = 10000,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  float reg_out = reverse_voltage_division(measured_output_voltage, &divider);
  return (uint16_t)roundf(reg_out);
}

/*
 * Uses LT6106 as well, but has variable r_out configured by placing jumpers on
 * X11 and X12.
 *
 * Note: we return uint32_t because we can measure values from 20 mA to 120 A.
 */
uint32_t adc_to_vbat_out_current(uint16_t adc_value) {
  const lt6106_current_sensor_t sensor = {
      .r_in = 51,
      .r_out = get_current_measure_jumper_r_out(),
      .r_sense = 0.0005F,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  float i_sense = lt6106_sense_current(measured_output_voltage, &sensor);
  return (uint32_t)roundf(i_sense);
}

uint16_t adc_to_vbat_out_voltage(uint16_t adc_value) {
  const voltage_divider_t divider = {
      .r1 = 13000,
      .r2 = 1600,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  float vbat_out = reverse_voltage_division(measured_output_voltage, &divider);
  return (uint16_t)roundf(vbat_out);
}

// Returns voltage in mV.
static float adc_value_to_voltage(uint16_t adc_value) {
  return ADC_REF_VOLTAGE_MV * adc_value / (float)ADC_RESOLUTION_12BIT;
}
