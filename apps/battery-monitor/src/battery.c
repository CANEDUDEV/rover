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

void battery_cells_reset(void);

void handle_battery_state(const adc_reading_t *adc_reading);
void handle_faults(void);
void update_battery_cells(const adc_reading_t *adc_reading);
void update_battery_charge(void);
void update_battery_leds(void);
void update_reg_out_voltage_controller(void);
bool is_reg_out_voltage_stable(void);
bool is_low_voltage_fault(void);
uint16_t *get_lowest_cell(void);

battery_state_t *get_battery_state(void) {
  return &battery_state;
}

void battery_state_init(void) {
  led_init();
  battery_state_reset();

  battery_state.cells.min_voltage = LIPO_CELL_MIN_VOLTAGE;
  battery_state.cells.max_voltage = LIPO_CELL_MAX_VOLTAGE;
  battery_state.cells.high_load_threshold = DEFAULT_HIGH_LOAD_THRESHOLD_MA;
  battery_state.cells.low_voltage_cutoff_low =
      DEFAULT_LOW_VOLTAGE_CUTOFF_LOW_MV;
  battery_state.cells.low_voltage_cutoff_high =
      DEFAULT_LOW_VOLTAGE_CUTOFF_HIGH_MV;

  battery_state.charge = 0;
  battery_state.target_reg_out_voltage = 0;

  battery_state.vbat_out.overcurrent_threshold =
      DEFAULT_OVERCURRENT_THRESHOLD_MA;
  battery_state.reg_out.overcurrent_threshold =
      DEFAULT_REG_OUT_OVERCURRENT_THRESHOLD_MA;

  set_reg_out_power_off();
  set_vbat_power_off();
}

void battery_state_reset(void) {
  power_output_reset(&battery_state.vbat_out);
  power_output_reset(&battery_state.reg_out);
  battery_cells_reset();
}

void power_output_reset(power_output_t *output) {
  output->voltage = 0;
  output->current = 0;
  output->overcurrent_fault = false;
  led_stop_signal_fault();
}

void battery_cells_reset(void) {
  memset(battery_state.cells.voltage, 0, sizeof(battery_state.cells.voltage));
  battery_state.cells.low_voltage_fault = false;
  led_stop_signal_fault();
}

void update_battery_state(const adc_reading_t *adc_reading) {
  if (get_vbat_power_state() == POWER_OFF) {
    return;  // Early return if power is not on
  }

  handle_battery_state(adc_reading);
  handle_faults();
}

void handle_battery_state(const adc_reading_t *adc_reading) {
  battery_state.reg_out.current =
      adc_to_reg_out_current(adc_reading->adc2_buf[REG_OUT_CURRENT_INDEX]);
  battery_state.reg_out.voltage =
      adc_to_reg_out_voltage(adc_reading->adc2_buf[REG_OUT_VOLTAGE_INDEX]);
  battery_state.vbat_out.voltage =
      adc_to_vbat_out_voltage(adc_reading->adc2_buf[VBAT_OUT_VOLTAGE_INDEX]);
  battery_state.vbat_out.current =
      adc_to_vbat_out_current(adc_reading->adc2_buf[VBAT_OUT_CURRENT_INDEX]);

  update_battery_cells(adc_reading);
  update_battery_charge();
  update_voltage_regulator_jumper_state();
  update_reg_out_voltage_controller();

  if (is_low_voltage_fault()) {
    battery_state.cells.low_voltage_fault = true;
  }

  if (battery_state.vbat_out.current >
      battery_state.vbat_out.overcurrent_threshold) {
    battery_state.vbat_out.overcurrent_fault = true;
  }

  if (battery_state.reg_out.current >
      battery_state.reg_out.overcurrent_threshold) {
    battery_state.reg_out.overcurrent_fault = true;
  }
}

void handle_faults(void) {
  if (battery_state.cells.low_voltage_fault ||
      battery_state.vbat_out.overcurrent_fault) {
    set_vbat_power_off();
    led_signal_fault();

  }

  else if (battery_state.reg_out.overcurrent_fault) {
    set_reg_out_power_off();
    led_signal_fault();
  }

  else {  // No fault occured, update LED as usual
    update_battery_leds();
  }
}

void update_battery_cells(const adc_reading_t *adc_reading) {
  // NOLINTBEGIN(*-magic-numbers)
  int32_t total_voltage = 0;
  int32_t cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[0]);
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells.voltage[0] = cell_voltage;
  total_voltage = battery_state.cells.voltage[0];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[1]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells.voltage[1] = cell_voltage;
  total_voltage += battery_state.cells.voltage[1];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[2]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells.voltage[2] = cell_voltage;
  total_voltage += battery_state.cells.voltage[2];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc1_buf[3]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells.voltage[3] = cell_voltage;
  total_voltage += battery_state.cells.voltage[3];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc2_buf[0]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells.voltage[4] = cell_voltage;
  total_voltage += battery_state.cells.voltage[4];

  cell_voltage = adc_to_cell_voltage(adc_reading->adc2_buf[1]) - total_voltage;
  if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD) {
    cell_voltage = 0;
  }
  battery_state.cells.voltage[5] = cell_voltage;
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
      (float)(*lowest_cell - battery_state.cells.min_voltage) /
      (float)(battery_state.cells.max_voltage -
              battery_state.cells.min_voltage);

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

  if (battery_state.reg_out.voltage > battery_state.target_reg_out_voltage &&
      next_pot_value > 0) {
    next_pot_value--;
  }

  if (battery_state.reg_out.voltage < battery_state.target_reg_out_voltage &&
      next_pot_value < UINT8_MAX) {
    next_pot_value++;
  }

  write_potentiometer_value(next_pot_value);
}

bool is_reg_out_voltage_stable(void) {
  const uint8_t accepted_error = 10;
  int32_t max_voltage =
      (int32_t)battery_state.target_reg_out_voltage + accepted_error;
  int32_t min_voltage =
      (int32_t)battery_state.target_reg_out_voltage - accepted_error;

  return (int32_t)battery_state.reg_out.voltage < max_voltage &&
         (int32_t)battery_state.reg_out.voltage > min_voltage;
}

bool is_low_voltage_fault(void) {
  uint16_t *lowest_cell = get_lowest_cell();
  if (!lowest_cell) {
    return false;
  }

  if (battery_state.vbat_out.current >=
      battery_state.cells.high_load_threshold) {
    return *lowest_cell <= battery_state.cells.low_voltage_cutoff_high;
  }

  return *lowest_cell <= battery_state.cells.low_voltage_cutoff_low;
}

uint16_t *get_lowest_cell(void) {
  uint16_t lowest_voltage = battery_state.cells.max_voltage;
  uint16_t *lowest_cell = NULL;
  for (uint8_t i = 0; i < BATTERY_CELLS_MAX; i++) {
    // If cell is not connected, do not use its values in the low voltage
    // detection logic.
    if (battery_state.cells.voltage[i] < BATTERY_CELL_DETECTION_THRESHOLD) {
      continue;
    }
    if (battery_state.cells.voltage[i] <= lowest_voltage) {
      lowest_voltage = battery_state.cells.voltage[i];
      lowest_cell = &battery_state.cells.voltage[i];
    }
  }
  return lowest_cell;
}
