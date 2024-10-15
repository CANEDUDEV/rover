#include "battery.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "jumpers.h"
#include "led.h"
#include "potentiometer.h"
#include "power.h"

// STM32Common
#include "error.h"
#include "lfs-wrapper.h"

// Libs
#include "float.h"

static battery_state_t battery_state;
static const char *calibration_filename = "/calibration";

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

void calibrate_cells(const adc_reading_t *reading);
void load_calibration(void);
void save_calibration(void);
void init_default_calibration(void);

battery_state_t *get_battery_state(void) {
  return &battery_state;
}

void battery_state_init(void) {
  led_init();
  battery_state_reset();
  load_calibration();

  battery_state.cells.min_voltage = LIPO_CELL_MIN_VOLTAGE_MV;
  battery_state.cells.max_voltage = LIPO_CELL_MAX_VOLTAGE_MV;
  battery_state.cells.low_voltage_cutoff = DEFAULT_LOW_VOLTAGE_CUTOFF_MV;

  battery_state.charge = 0;
  battery_state.target_reg_out_voltage = 0;

  battery_state.vbat_out.overcurrent_threshold =
      DEFAULT_OVERCURRENT_THRESHOLD_MA;
  battery_state.reg_out.overcurrent_threshold =
      DEFAULT_REG_OUT_OVERCURRENT_THRESHOLD_MA;

  battery_state.calibration_voltage = 0;

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
  if (battery_state.calibration_voltage) {
    printf("calibration initiated\r\n");
    battery_cells_reset();
    calibrate_cells(adc_reading);
    save_calibration();
    return;
  }

  battery_state.reg_out.voltage = adc_reading->reg_out_voltage;
  battery_state.reg_out.current = adc_reading->reg_out_current;
  battery_state.vbat_out.voltage = adc_reading->vbat_out_voltage;
  battery_state.vbat_out.current = adc_reading->vbat_out_current;
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

  handle_faults();
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
  int32_t total_voltage = 0;

  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    float calibrated_voltage = battery_state.cells.calibration_factor[i] *
                               (float)adc_reading->cells[i];
    int32_t cell_voltage = (int32_t)roundf(calibrated_voltage) - total_voltage;
    if (cell_voltage < BATTERY_CELL_DETECTION_THRESHOLD_MV) {
      cell_voltage = 0;
    }
    battery_state.cells.voltage[i] = cell_voltage;
    total_voltage += cell_voltage;
  }
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

  // Avoid blinking the LEDS when the charge is at the limit between two states
  uint8_t new_charge = (uint8_t)roundf(charge_percent);
  const uint8_t charge_error = 5;
  if (new_charge < battery_state.charge - charge_error ||
      new_charge > battery_state.charge + charge_error) {
    battery_state.charge = new_charge;
  }
}

void update_battery_leds(void) {
  const uint8_t warning_percentage = 25;
  const uint8_t caution_percentage = 50;

  if (battery_state.charge < warning_percentage) {
    set_led_color(LED6, RED);
    set_led_color(LED7, RED);
  }

  else if (battery_state.charge < caution_percentage) {
    set_led_color(LED6, ORANGE);
    set_led_color(LED7, ORANGE);
  }

  else {
    set_led_color(LED6, GREEN);
    set_led_color(LED7, GREEN);
  }
}

void update_reg_out_voltage_controller(void) {
  // Not relevant to update the controller when the reg out is off.
  if (get_reg_out_power_state() == POWER_OFF) {
    return;
  }

  // Target voltage is set to 0 on init. Default to 5V or 12V depending on
  // jumper state.
  if (battery_state.target_reg_out_voltage == 0) {
    if (get_voltage_regulator_jumper_state() == VOUT_0_TO_6V) {
      battery_state.target_reg_out_voltage = DEFAULT_REG_OUT_VOLTAGE_LOW_MV;
    } else {
      battery_state.target_reg_out_voltage = DEFAULT_REG_OUT_VOLTAGE_HIGH_MV;
    }
  }

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

  return *lowest_cell <= battery_state.cells.low_voltage_cutoff;
}

uint16_t *get_lowest_cell(void) {
  uint16_t lowest_voltage = battery_state.cells.max_voltage;
  uint16_t *lowest_cell = NULL;
  for (uint8_t i = 0; i < BATTERY_CELLS_MAX; i++) {
    // If cell is not connected, do not use its values in the low voltage
    // detection logic.
    if (battery_state.cells.voltage[i] < BATTERY_CELL_DETECTION_THRESHOLD_MV) {
      continue;
    }
    if (battery_state.cells.voltage[i] <= lowest_voltage) {
      lowest_voltage = battery_state.cells.voltage[i];
      lowest_cell = &battery_state.cells.voltage[i];
    }
  }
  return lowest_cell;
}

void calibrate_cells(const adc_reading_t *reading) {
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    battery_state.cells.calibration_factor[i] =
        (float)battery_state.calibration_voltage / (float)reading->cells[i];
    printf("cell %d calibration factor: ", i);
    float_print(battery_state.cells.calibration_factor[i]);
    printf("\r\n");
  }
  battery_state.calibration_voltage = 0;
}

void load_calibration(void) {
  file_t calibration_file = {
      .name = calibration_filename,
      .size = sizeof(battery_state.cells.calibration_factor),
      .data = battery_state.cells.calibration_factor,
  };
  if (read_file(&calibration_file) != APP_OK) {
    printf("note: couldn't read calibration file, using defaults.\r\n");
    init_default_calibration();
  }
}

void save_calibration(void) {
  file_t calibration_file = {
      .name = calibration_filename,
      .size = sizeof(battery_state.cells.calibration_factor),
      .data = battery_state.cells.calibration_factor,
  };
  write_file_async(&calibration_file);
}

void init_default_calibration(void) {
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    battery_state.cells.calibration_factor[i] = 1.0F;
  }
}
