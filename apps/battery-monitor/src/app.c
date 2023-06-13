#include "app.h"

#include <string.h>

#include "led.h"
#include "ports.h"

#define ADC_REF_VOLTAGE 3300  // in mV
#define ADC_MAX 4095          // 12-bit ADC

#define POT_ADDR (0x53 << 1)  // Shift left to match STM32 specification
#define POT_IVRA_ADDR 0x0

// Cells that don't exist will report a value close to 0. We set a cell
// detection threshold voltage at 100 mV.
#define BATTERY_CELL_DETECTION_THRESHOLD 100

static uint32_t vbat_sense_r_out;

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
  }
}

void config_voltage_regulator(I2C_HandleTypeDef *hi2c, uint8_t pot_value) {
  uint8_t ivra_write[2] = {POT_IVRA_ADDR, pot_value};
  HAL_I2C_Master_Transmit(hi2c, POT_ADDR, ivra_write, sizeof(ivra_write),
                          HAL_MAX_DELAY);
}

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

// NOLINTBEGIN(*-magic-numbers)
void parse_adc_values(const adc_reading_t *adc_reading,
                      battery_state_t *battery_state) {
  for (int i = 0; i < 4; i++) {
    battery_state->cells[i] = adc_to_cell_voltage(adc_reading->adc1_buf[i]);
  }
  battery_state->cells[4] = adc_to_cell_voltage(adc_reading->adc2_buf[0]);
  battery_state->cells[5] = adc_to_cell_voltage(adc_reading->adc2_buf[1]);
  battery_state->reg_out_current =
      adc_to_reg_out_current(adc_reading->adc2_buf[2]);
  battery_state->vbat_out_current =
      adc_to_vbat_out_current(adc_reading->adc2_buf[3]);
}
// NOLINTEND(*-magic-numbers)

battery_charge_t lowest_cell(const battery_charge_t *cell_charge) {
  battery_charge_t lowest = CHARGE_100_PERCENT;
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    // Comparison based on enum ordering
    if (cell_charge[i] < lowest) {
      lowest = cell_charge[i];
    }
  }
  return lowest;
}

uint8_t set_charge_state_led(const battery_charge_t *charge) {
  switch (*charge) {
    case CHARGE_100_PERCENT:
      set_led_color(LED6, GREEN);
      set_led_color(LED7, GREEN);
      return 0;
    case CHARGE_80_PERCENT:
      set_led_color(LED6, GREEN);
      set_led_color(LED7, NONE);
      return 0;
    case CHARGE_60_PERCENT:
      set_led_color(LED6, ORANGE);
      set_led_color(LED7, ORANGE);
      return 0;
    case CHARGE_40_PERCENT:
      set_led_color(LED6, ORANGE);
      set_led_color(LED7, NONE);
      return 0;
    case CHARGE_20_PERCENT:
      set_led_color(LED6, RED);
      set_led_color(LED7, RED);
      return 0;
    case LOW_VOLTAGE_CUTOFF:
    default:
      return 1;
  }
}

// Always report lowest detected charge to detect the most discharged cell.
battery_charge_t read_battery_charge(const battery_state_t *battery_state) {
  battery_charge_t cell_charge[BATTERY_CELLS_MAX];
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    // If cell is not connected, we report it as fully charged to not use its
    // values in the low voltage detection logic.
    if (battery_state->cells[i] < BATTERY_CELL_DETECTION_THRESHOLD) {
      cell_charge[i] = CHARGE_100_PERCENT;
      continue;
    }

    if (battery_state->cells[i] <= LOW_VOLTAGE_CUTOFF) {
      cell_charge[i] = LOW_VOLTAGE_CUTOFF;
    } else if (battery_state->cells[i] <= CHARGE_20_PERCENT) {
      cell_charge[i] = CHARGE_20_PERCENT;
    } else if (battery_state->cells[i] <= CHARGE_40_PERCENT) {
      cell_charge[i] = CHARGE_40_PERCENT;
    } else if (battery_state->cells[i] <= CHARGE_60_PERCENT) {
      cell_charge[i] = CHARGE_60_PERCENT;
    } else if (battery_state->cells[i] <= CHARGE_80_PERCENT) {
      cell_charge[i] = CHARGE_80_PERCENT;
    } else {
      cell_charge[i] = CHARGE_100_PERCENT;
    }
  }
  return lowest_cell(cell_charge);
}
