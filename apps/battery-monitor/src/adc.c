#include "adc.h"

#include <math.h>

#include "battery.h"

#define ADC_REF_VOLTAGE 3300            // in mV
#define ADC_RESOLUTION ((1 << 12) - 1)  // 12-bit ADC

static float vbat_sense_r_out;

/* Voltage divider with R1 = 13kOhm and R2 = 1.6kOhm
 * v_cell = v_out * (R1 + R2) / R2
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION
 */
uint16_t adc_to_cell_voltage(const uint16_t adc_value) {
  float v_out = ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;
  const float numerator = v_out * (13000 + 1600);
  const float denominator = 1600;
  float cell_voltage = numerator / denominator;
  return (uint16_t)roundf(cell_voltage);
}

/* The LT6106 current sensor specifies that
 * i_sense = v_out * r_in / (r_sense * r_out).
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION
 *
 * Resistances for Battery Node rev B:
 * r_in = 51 Ohm
 * r_sense = 0.005 Ohm
 * r_out = 5100 Ohm
 */
uint16_t adc_to_reg_out_current(const uint16_t adc_value) {
  float v_out = ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;
  const float r_in = 51;
  const float r_out = 5100;
  // Invert r_sense to avoid floating point, 1/0.005Ohm = 200 Ohm
  const float inv_r_sense = 200;

  // Multiply by inv_r_sense instead of dividing by r_sense
  float i_sense = v_out * r_in * inv_r_sense / r_out;  // in mA
  return (uint16_t)roundf(i_sense);
}

/* Voltage divider with R1 = 47kOhm and R2 = 10kOhm
 * reg_vout = v_out * (R1 + R2) / R2
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION
 */
uint16_t adc_to_reg_out_voltage(uint16_t adc_value) {
  float v_out = ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;
  const float numerator = v_out * (47000 + 10000);
  const float denominator = 10000;
  float reg_vout = numerator / denominator;
  return (uint16_t)roundf(reg_vout);
}

/*
 * Uses LT6106 as well, with variable r_out configured by placing jumpers on X11
 * and X12.
 * i_sense = v_out * r_in / (r_sense * r_out)
 *
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION
 *
 * Resistances:
 * r_in = 51 Ohm
 * r_sense = 0.5 mOhm
 * r_out: See vbat_sense_r_out
 *
 * Note: we return uint32_t because we can measure values from 20 mA to 120 A.
 */
uint32_t adc_to_vbat_out_current(const uint16_t adc_value) {
  float v_out = ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;  // in mV
  const float r_in = 51;
  // Invert r_sense to avoid floating point, 1/0.5mOhm = 2000 Ohm
  const float inv_r_sense = 2000;

  // Multiply by inv_r_sense instead of dividing by r_sense
  float i_sense = v_out * r_in * inv_r_sense / vbat_sense_r_out;  // in mA
  return (uint16_t)roundf(i_sense);
}

void set_jumper_config(jumper_config_t jumper_config) {
  switch (jumper_config) {
    case ALL_OFF:
      vbat_sense_r_out = 10200;  // NOLINT
      break;
    case X11_ON:
      vbat_sense_r_out = 5100;  // NOLINT
      break;
    case X12_ON:
      vbat_sense_r_out = 3400;  // NOLINT
      break;
    case ALL_ON:
      vbat_sense_r_out = 2550;  // NOLINT
      break;
    default:  // Ignore any other value
      break;
  }
}
