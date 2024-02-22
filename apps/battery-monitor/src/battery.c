#include "battery.h"

#include <math.h>
#include <string.h>

#include "adc.h"
#include "error.h"
#include "jumpers.h"
#include "led.h"
#include "potentiometer.h"
#include "power.h"

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
void update_reg_out_voltage_controller(void);
bool is_reg_out_voltage_stable(void);
bool is_low_voltage_fault(void);
bool is_over_current_fault(void);
uint16_t *get_lowest_cell(void);

void battery_state_init(void) {
  led_init();
  battery_state_reset();

  battery_state.cell_min_voltage = LIPO_CELL_MIN_VOLTAGE;
  battery_state.cell_max_voltage = LIPO_CELL_MAX_VOLTAGE;
  battery_state.target_reg_out_voltage = 0;

  // TODO: set a low voltage cutoff point at 3.2V when running a high amp load.
  // TODO: set a low voltage cutoff point at 3.7 V when running a low amp load.
  battery_state.low_voltage_cutoff = battery_state.cell_min_voltage;

  battery_state.over_current_threshold = DEFAULT_OVER_CURRENT_THRESHOLD_MA;
}

void battery_state_reset(void) {
  set_reg_vout_power_off();
  set_vbat_power_off();

  led_stop_signal_fault();

  memset(battery_state.cell_voltage, 0, sizeof(battery_state.cell_voltage));
  battery_state.reg_out_voltage = 0;
  battery_state.reg_out_current = 0;
  battery_state.vbat_out_voltage = 0;
  battery_state.vbat_out_current = 0;
  battery_state.charge = 0;
  battery_state.over_current_fault = false;
  battery_state.low_voltage_fault = false;
}

battery_state_t *get_battery_state(void) {
  return &battery_state;
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
  if (get_vbat_power_state() == POWER_OFF) {
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
  update_reg_out_voltage_controller();

  if (is_low_voltage_fault()) {
    battery_state.low_voltage_fault = true;
  }

  if (is_over_current_fault()) {
    battery_state.over_current_fault = true;
  }
}

void handle_faults(void) {
  if (battery_state.low_voltage_fault || battery_state.over_current_fault) {
    set_vbat_power_off();
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
  battery_state.cell_voltage[0] = cell_voltage;
  total_voltage = battery_state.cell_voltage[0];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[1]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cell_voltage[1] = cell_voltage;
  total_voltage += battery_state.cell_voltage[1];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[2]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cell_voltage[2] = cell_voltage;
  total_voltage += battery_state.cell_voltage[2];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[3]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cell_voltage[3] = cell_voltage;
  total_voltage += battery_state.cell_voltage[3];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc2_buf[0]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cell_voltage[4] = cell_voltage;
  total_voltage += battery_state.cell_voltage[4];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc2_buf[1]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cell_voltage[5] = cell_voltage;
  // NOLINTEND(*-magic-numbers)
}

// Always report lowest detected charge to detect the most discharged cell.
void update_battery_charge(void) {
  uint16_t *lowest_cell = get_lowest_cell();

  // If no cells are connected for some reason
  if (!lowest_cell) {
    battery_state.charge = BATTERY_CHARGE_100_PERCENT;
    return;
  }

  float charge_percent =
      (float)(*lowest_cell - battery_state.cell_min_voltage) /
      (float)(battery_state.cell_max_voltage - battery_state.cell_min_voltage);

  charge_percent *= BATTERY_CHARGE_100_PERCENT;

  if (charge_percent > BATTERY_CHARGE_100_PERCENT) {
    charge_percent = BATTERY_CHARGE_100_PERCENT;
  }
  if (charge_percent < 0) {
    charge_percent = 0;
  }

  battery_state.charge = (uint8_t)roundf(charge_percent);
}

void update_battery_leds(void) {
  const uint8_t charge_20_percent = 20;
  const uint8_t charge_50_percent = 50;

  if (battery_state.charge < charge_20_percent) {
    set_led_color(LED6, RED);
    set_led_color(LED7, RED);
  }

  else if (battery_state.charge < charge_50_percent) {
    set_led_color(LED6, ORANGE);
    set_led_color(LED7, ORANGE);
  }

  else {
    set_led_color(LED6, GREEN);
    set_led_color(LED7, GREEN);
  }
}

void update_reg_out_voltage_controller(void) {
  if (is_reg_out_voltage_stable()) {
    return;
  }

  uint8_t current_pot_value = 0;
  if (read_potentiometer_value(&current_pot_value) != APP_OK) {
    return;
  }

  int16_t next_pot_value = current_pot_value;

  if (battery_state.reg_out_voltage > battery_state.target_reg_out_voltage &&
      next_pot_value > 0) {
    next_pot_value--;
  }

  if (battery_state.reg_out_voltage < battery_state.target_reg_out_voltage &&
      next_pot_value < UINT8_MAX) {
    next_pot_value++;
  }

  write_potentiometer_value(next_pot_value);
}

bool is_reg_out_voltage_stable(void) {
  const uint16_t accepted_error = 10;
  return battery_state.reg_out_voltage <
             battery_state.target_reg_out_voltage + accepted_error &&
         battery_state.reg_out_voltage >
             battery_state.target_reg_out_voltage - accepted_error;
}

bool is_low_voltage_fault(void) {
  uint16_t *lowest_cell = get_lowest_cell();
  if (!lowest_cell) {
    return false;
  }
  return *lowest_cell < battery_state.low_voltage_cutoff;
}

bool is_over_current_fault(void) {
  return battery_state.reg_out_current + battery_state.vbat_out_current >
         battery_state.over_current_threshold;
}

uint16_t *get_lowest_cell(void) {
  uint16_t lowest_voltage = battery_state.cell_max_voltage;
  uint16_t *lowest_cell = NULL;
  for (uint8_t i = 0; i < BATTERY_CELLS_MAX; i++) {
    // If cell is not connected, do not use its values in the low voltage
    // detection logic.
    if (battery_state.cell_voltage[i] < BATTERY_CELL_DETECTION_THRESHOLD) {
      continue;
    }
    if (battery_state.cell_voltage[i] <= lowest_voltage) {
      lowest_voltage = battery_state.cell_voltage[i];
      lowest_cell = &battery_state.cell_voltage[i];
    }
  }
  return lowest_cell;
}
