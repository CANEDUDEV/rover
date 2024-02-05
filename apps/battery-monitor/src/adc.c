#include "adc.h"

#include <math.h>

#define ADC_REF_VOLTAGE 3300            // in mV
#define ADC_RESOLUTION ((1 << 12) - 1)  // 12-bit ADC

static float vbat_sense_r_out;

typedef struct {
  uint16_t r1;
  uint16_t r2;
} voltage_divider_t;

typedef struct {
  float r_in;
  float r_out;
  float r_sense;
} lt6106_current_sensor_t;

static float sense_current(float voltage,
                           const lt6106_current_sensor_t *sensor);
static float adc_value_to_voltage(uint16_t adc_value);
static float divide_voltage(float voltage, const voltage_divider_t *divider);

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
  float measured_voltage = adc_value_to_voltage(adc_value);
  float cell_voltage = divide_voltage(measured_voltage, &divider);
  return (uint16_t)roundf(cell_voltage);
}

uint16_t adc_to_reg_out_current(uint16_t adc_value) {
  const lt6106_current_sensor_t sensor = {
      .r_in = 51,
      .r_out = 5100,
      .r_sense = 0.005F,
  };
  float measured_voltage = adc_value_to_voltage(adc_value);
  float i_sense = sense_current(measured_voltage, &sensor);
  return (uint16_t)roundf(i_sense);
}

uint16_t adc_to_reg_out_voltage(uint16_t adc_value) {
  const voltage_divider_t divider = {
      .r1 = 47000,
      .r2 = 10000,
  };
  float measured_voltage = adc_value_to_voltage(adc_value);
  float reg_vout = divide_voltage(measured_voltage, &divider);
  return (uint16_t)roundf(reg_vout);
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
      .r_out = vbat_sense_r_out,
      .r_sense = 0.0005F,
  };
  float measured_voltage = adc_value_to_voltage(adc_value);
  float i_sense = sense_current(measured_voltage, &sensor);
  return (uint32_t)roundf(i_sense);
}

uint16_t adc_to_vbat_out_voltage(uint16_t adc_value) {
  const voltage_divider_t divider = {
      .r1 = 13000,
      .r2 = 1600,
  };
  float measured_voltage = adc_value_to_voltage(adc_value);
  float vbat_out = divide_voltage(measured_voltage, &divider);
  return (uint16_t)roundf(vbat_out);
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

// Returns voltage in mV.
static float adc_value_to_voltage(uint16_t adc_value) {
  return ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;
}

static float divide_voltage(float voltage, const voltage_divider_t *divider) {
  return voltage * (float)(divider->r1 + divider->r2) / (float)divider->r2;
}

/* The LT6106 current sensor specifies that
 * i_sense = voltage * r_in / (r_sense * r_out).
 *
 * Returns current in mA.
 */
static float sense_current(float voltage,
                           const lt6106_current_sensor_t *sensor) {
  return voltage * sensor->r_in / (sensor->r_sense * sensor->r_out);
}
