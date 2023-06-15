#include "adc.h"

#include "battery.h"

#define ADC_MAX 4095          // 12-bit ADC
#define ADC_REF_VOLTAGE 3300  // in mV

static uint32_t vbat_sense_r_out;

/* Voltage divider with R1 = 1kOhm and R2 = 2kOhm
 * v_cell = v_out * (R1 + R2) / R2
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value / ADC_MAX
 */
uint16_t adc_to_cell_voltage(const uint16_t adc_value) {
  uint32_t v_out = (ADC_REF_VOLTAGE * adc_value) / ADC_MAX;
  const uint32_t numerator = v_out * (1000 + 2000);
  const uint32_t denominator = 2000;
  uint16_t v_cell = (uint16_t)(numerator / denominator);
  if (v_cell < BATTERY_CELL_DETECTION_THRESHOLD) {
    v_cell = 0;
  }
  return v_cell;
}

/* The LT6106 current sensor specifies that
 * i_sense = v_out * r_in / (r_sense * r_out).
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value / ADC_MAX
 *
 * Resistances for Battery Node rev B:
 * r_in = 51 Ohm
 * r_sense = 0.005 Ohm
 * r_out = 5100 Ohm
 */
uint16_t adc_to_reg_out_current(const uint16_t adc_value) {
  uint32_t v_out = (ADC_REF_VOLTAGE * adc_value) / ADC_MAX;
  const uint32_t r_in = 51;
  const uint32_t r_out = 5100;
  // Invert r_sense to avoid floating point, 1/0.005Ohm = 200 Ohm
  const uint32_t inv_r_sense = 200;

  // Multiply by inv_r_sense instead of dividing by r_sense
  uint16_t i_sense = (uint16_t)((v_out * r_in * inv_r_sense) / r_out);  // in mA
  return i_sense;
}

/*
 * Uses LT6106 as well, with variable r_out configured by placing jumpers on X11
 * and X12.
 * i_sense = v_out * r_in / (r_sense * r_out)
 *
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value / ADC_MAX
 *
 * Resistances:
 * r_in = 51 Ohm
 * r_sense = 0.5 mOhm
 * r_out: See vbat_sense_r_out
 *
 * Note: we use uint32_t because we can measure values from 20 mA to 120 A.
 */
uint32_t adc_to_vbat_out_current(const uint16_t adc_value) {
  uint32_t v_out = (ADC_REF_VOLTAGE * adc_value) / ADC_MAX;  // in mV
  const uint32_t r_in = 51;
  // Invert r_sense to avoid floating point, 1/0.5mOhm = 2000 Ohm
  const uint32_t inv_r_sense = 2000;

  // Multiply by inv_r_sense instead of dividing by r_sense
  uint32_t i_sense = (v_out * r_in * inv_r_sense) / vbat_sense_r_out;  // in mA
  return i_sense;
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
