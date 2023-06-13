#include "app.h"

#include <string.h>

#include "adc.h"
#include "battery.h"
#include "led.h"
#include "ports.h"

#define POT_ADDR (0x53 << 1)  // Shift left to match STM32 specification
#define POT_IVRA_ADDR 0x0

void config_voltage_regulator(I2C_HandleTypeDef *hi2c, uint8_t pot_value) {
  uint8_t ivra_write[2] = {POT_IVRA_ADDR, pot_value};
  HAL_I2C_Master_Transmit(hi2c, POT_ADDR, ivra_write, sizeof(ivra_write),
                          HAL_MAX_DELAY);
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
