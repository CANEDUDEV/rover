#include "adc.h"

#include <math.h>

#include "jumpers.h"
#include "lt6106.h"
#include "voltage-divider.h"

#define ADC_REF_VOLTAGE_MV 3300
#define ADC_RESOLUTION_12BIT ((1 << 12) - 1)
#define VREFINT_CAL_ADDR ((uint16_t *)(0x1FFFF7BA))

// Constants for average indices
#define CELL0_INDEX 0
#define CELL1_INDEX 1
#define CELL2_INDEX 2
#define CELL3_INDEX 3
#define VREFINT_INDEX 4
#define CELL4_INDEX (ADC1_NUM_CHANNELS + 0)
#define CELL5_INDEX (ADC1_NUM_CHANNELS + 1)
#define REG_OUT_CURRENT_INDEX (ADC1_NUM_CHANNELS + 2)
#define REG_OUT_VOLTAGE_INDEX (ADC1_NUM_CHANNELS + 3)
#define VBAT_OUT_VOLTAGE_INDEX (ADC1_NUM_CHANNELS + 4)
#define VBAT_OUT_CURRENT_INDEX (ADC1_NUM_CHANNELS + 5)

static uint32_t vdda;
static const int cell_indices[] = {
    CELL0_INDEX, CELL1_INDEX, CELL2_INDEX,
    CELL3_INDEX, CELL4_INDEX, CELL5_INDEX,
};

void adc_update_vdda(uint16_t vrefint_cal, uint16_t vrefint);
static uint16_t adc_value_to_voltage(uint16_t adc_value);

void adc_average_samples(const adc_samples_t *samples, adc_reading_t *reading) {
  // Need to initialize to 0, otherwise garbage data can be present
  uint32_t sum_adc1[ADC1_NUM_CHANNELS] = {0};
  uint32_t sum_adc2[ADC2_NUM_CHANNELS] = {0};

  for (int i = 0; i < ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS; i++) {
    sum_adc1[i % ADC1_NUM_CHANNELS] += samples->adc1_buf[i];
  }
  for (int i = 0; i < ADC_NUM_SAMPLES * ADC2_NUM_CHANNELS; i++) {
    sum_adc2[i % ADC2_NUM_CHANNELS] += samples->adc2_buf[i];
  }

  uint32_t adc_average[ADC1_NUM_CHANNELS + ADC2_NUM_CHANNELS];

  for (int i = 0; i < ADC1_NUM_CHANNELS; i++) {
    adc_average[i] = sum_adc1[i] / ADC_NUM_SAMPLES;
  }
  for (int i = 0; i < ADC2_NUM_CHANNELS; i++) {
    adc_average[ADC1_NUM_CHANNELS + i] = sum_adc2[i] / ADC_NUM_SAMPLES;
  }

  adc_update_vdda((*VREFINT_CAL_ADDR), adc_average[VREFINT_INDEX]);

  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    reading->cells[i] = adc_to_cell_voltage(adc_average[cell_indices[i]]);
  }

  reading->reg_out_voltage =
      adc_to_reg_out_voltage(adc_average[REG_OUT_VOLTAGE_INDEX]);
  reading->reg_out_current =
      adc_to_reg_out_current(adc_average[REG_OUT_CURRENT_INDEX]);
  reading->vbat_out_voltage =
      adc_to_vbat_out_voltage(adc_average[VBAT_OUT_VOLTAGE_INDEX]);
  reading->vbat_out_current =
      adc_to_vbat_out_current(adc_average[VBAT_OUT_CURRENT_INDEX]);
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

void adc_update_vdda(uint16_t vrefint_cal, uint16_t vrefint) {
  vdda = ADC_REF_VOLTAGE_MV * vrefint_cal / vrefint;
}

// Returns voltage in mV.
static uint16_t adc_value_to_voltage(uint16_t adc_value) {
  return (vdda * adc_value) / ADC_RESOLUTION_12BIT;
}
