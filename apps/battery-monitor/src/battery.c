#include "battery.h"

#include <string.h>

#include "adc.h"
#include "jumpers.h"
#include "led.h"
#include "ports.h"

// Constants for array indices
#define REG_OUT_CURRENT_INDEX 2
#define REG_OUT_VOLTAGE_INDEX 3
#define VBAT_OUT_VOLTAGE_INDEX 4
#define VBAT_OUT_CURRENT_INDEX 5

static battery_state_t battery_state;

void handle_battery_state(const adc_reading_t *adc_reading);
void handle_faults(void);
void update_battery_cells(const adc_reading_t *adc_reading);
void update_battery_charge(void);
void update_battery_leds(void);
bool is_power_on(void);
bool is_low_voltage(void);
bool is_over_current_fault(void);
uint16_t *get_lowest_cell(void);

void battery_state_init(void) {
  led_init();
  battery_state_reset();
  battery_state.low_voltage_cutoff = CHARGE_0_PERCENT;
  set_fuse_config(FUSE_100_AMPERE);
}

void battery_state_reset(void) {
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    battery_state.cells[i] = CHARGE_100_PERCENT;
  }
  battery_state.reg_out_voltage = 0;
  battery_state.reg_out_current = 0;
  battery_state.vbat_out_voltage = 0;
  battery_state.vbat_out_current = 0;
  battery_state.charge = CHARGE_100_PERCENT;
  battery_state.over_current_fault = false;
  battery_state.low_voltage_fault = false;
}

battery_state_t *get_battery_state(void) {
  return &battery_state;
}

void set_fuse_config(fuse_config_t fuse_config) {
  switch (fuse_config) {
    case FUSE_50_AMPERE:
      set_over_current_threshold(FUSE_50_AMPERE - 500);  // NOLINT
      break;

    case FUSE_100_AMPERE:
      set_over_current_threshold(FUSE_100_AMPERE - 500);  // NOLINT
      break;

    default:  // Ignore
      break;
  }
}

// We might detect false positives if the jumper config is set to detect low
// currents but the actual currents are much higher. Nevertheless, the largest
// part of the total current should be the vbat_out_current.
//
// TODO: calculate the maximum measured reg_out_current based on the jumper
// config, use it to set a better over_current_threshold that considers the
// reg_out_current separately from the vbat_out_current.
void set_over_current_threshold(uint32_t threshold) {
  battery_state.over_current_threshold = threshold;
}

void update_battery_state(const adc_reading_t *adc_reading) {
  if (!is_power_on()) {
    return;  // Early return if power is not on
  }

  handle_battery_state(adc_reading);
  handle_faults();
}

void handle_battery_state(const adc_reading_t *adc_reading) {
  battery_state.reg_out_current =
      adc_to_reg_out_current(adc_reading->adc2_buf[REG_OUT_CURRENT_INDEX]);
  battery_state.reg_out_voltage =
      adc_to_reg_out_voltage(adc_reading->adc2_buf[REG_OUT_VOLTAGE_INDEX]);
  battery_state.vbat_out_voltage =
      adc_to_vbat_out_voltage(adc_reading->adc2_buf[VBAT_OUT_VOLTAGE_INDEX]);
  battery_state.vbat_out_current =
      adc_to_vbat_out_current(adc_reading->adc2_buf[VBAT_OUT_CURRENT_INDEX]);

  update_battery_cells(adc_reading);
  update_battery_charge();
  update_battery_leds();
  update_voltage_regulator_jumper_state();

  if (is_low_voltage()) {
    battery_state.low_voltage_fault = true;
  }

  if (is_over_current_fault()) {
    battery_state.over_current_fault = true;
  }
}

void handle_faults(void) {
  if (battery_state.low_voltage_fault || battery_state.over_current_fault) {
    HAL_GPIO_WritePin(nPOWER_OFF_GPIO_PORT, nPOWER_OFF_PIN, GPIO_PIN_RESET);
    led_signal_fault();
  }
}

void update_battery_cells(const adc_reading_t *adc_reading) {
  // NOLINTBEGIN(*-magic-numbers)
  int32_t total_voltage = 0;
  int32_t cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[0]);
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells[0] = cell_voltage;
  total_voltage = battery_state.cells[0];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[1]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells[1] = cell_voltage;
  total_voltage += battery_state.cells[1];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[2]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells[2] = cell_voltage;
  total_voltage += battery_state.cells[2];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[3]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells[3] = cell_voltage;
  total_voltage += battery_state.cells[3];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc2_buf[0]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells[4] = cell_voltage;
  total_voltage += battery_state.cells[4];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc2_buf[1]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells[5] = cell_voltage;
  // NOLINTEND(*-magic-numbers)
}

// Always report lowest detected charge to detect the most discharged cell.
void update_battery_charge(void) {
  uint16_t *lowest_cell = get_lowest_cell();

  // If no cells are connected for some reason
  if (!lowest_cell) {
    return;
  }

  charge_t lowest = CHARGE_100_PERCENT;

  if (*lowest_cell <= CHARGE_0_PERCENT) {
    lowest = CHARGE_0_PERCENT;
  } else if (*lowest_cell <= CHARGE_20_PERCENT) {
    lowest = CHARGE_20_PERCENT;
  } else if (*lowest_cell <= CHARGE_40_PERCENT) {
    lowest = CHARGE_40_PERCENT;
  } else if (*lowest_cell <= CHARGE_60_PERCENT) {
    lowest = CHARGE_60_PERCENT;
  } else if (*lowest_cell <= CHARGE_80_PERCENT) {
    lowest = CHARGE_80_PERCENT;
  }

  // Charge state update logic
  const uint8_t report_limit = 10;
  static uint8_t report_count = 0;

  // Don't update charge state until we're sure it's a charge difference and not
  // an outlier.
  if (lowest < battery_state.charge) {
    report_count++;
  }

  if (report_count >= report_limit) {
    report_count = 0;
    battery_state.charge = lowest;
  }
}

void update_battery_leds(void) {
  switch (battery_state.charge) {
    case CHARGE_0_PERCENT:
      set_led_color(LED6, RED);
      set_led_color(LED7, NONE);
      break;

    case CHARGE_40_PERCENT:
      set_led_color(LED6, ORANGE);
      set_led_color(LED7, NONE);
      break;

    case CHARGE_20_PERCENT:
      set_led_color(LED6, RED);
      set_led_color(LED7, RED);
      break;

    case CHARGE_60_PERCENT:
      set_led_color(LED6, ORANGE);
      set_led_color(LED7, ORANGE);
      break;

    case CHARGE_80_PERCENT:
      set_led_color(LED6, GREEN);
      set_led_color(LED7, NONE);
      break;

    case CHARGE_100_PERCENT:
      set_led_color(LED6, GREEN);
      set_led_color(LED7, GREEN);
      break;

    default:
      break;
  }
}

bool is_power_on(void) {
  return HAL_GPIO_ReadPin(nPOWER_OFF_GPIO_PORT, nPOWER_OFF_PIN) == GPIO_PIN_SET;
}

bool is_low_voltage(void) {
  uint16_t *lowest_cell = get_lowest_cell();
  if (!lowest_cell) {
    return false;
  }
  const uint8_t report_limit = 10;
  static uint8_t report_count = 0;
  if (*lowest_cell < battery_state.low_voltage_cutoff) {
    report_count++;
  }
  if (report_count >= report_limit) {
    report_count = 0;
    return true;
  }
  return false;
}

bool is_over_current_fault(void) {
  return battery_state.reg_out_current + battery_state.vbat_out_current >
         battery_state.over_current_threshold;
}

uint16_t *get_lowest_cell(void) {
  uint16_t lowest_charge = CHARGE_100_PERCENT;
  uint16_t *lowest_cell = NULL;
  for (uint8_t i = 0; i < BATTERY_CELLS_MAX; i++) {
    // If cell is not connected, do not use its values in the low voltage
    // detection logic.
    if (battery_state.cells[i] < BATTERY_CELL_DETECTION_THRESHOLD) {
      continue;
    }
    if (battery_state.cells[i] < lowest_charge) {
      lowest_charge = battery_state.cells[i];
      lowest_cell = &battery_state.cells[i];
    }
  }
  return lowest_cell;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
  if (GPIO_PIN != OVER_CURRENT_PIN) {
    return;
  }

  battery_state.over_current_fault = true;
}
