#include <string.h>

#include "battery-internal.h"
#include "battery.h"
#include "error.h"

// Testing
#include "battery-fakes.h"
#include "test.h"

#define LIPO_OVERVOLTAGE (LIPO_CELL_MAX_VOLTAGE_MV + 100)
#define LIPO_UNDERVOLTAGE (LIPO_CELL_MIN_VOLTAGE_MV - 100)

void test_handle_faults_vbat_out_overcurrent_fault(void);
void test_handle_faults_reg_out_overcurrent_fault(void);
void test_handle_faults_low_voltage_fault(void);
void test_handle_faults_no_fault(void);
void test_update_battery_cells(void);
void test_update_battery_charge(void);
void test_update_battery_charge_no_cells(void);
void test_update_battery_charge_overcharged(void);
void test_update_battery_charge_undercharged(void);
void test_update_battery_leds_full_charge(void);
void test_update_battery_leds_half_charge(void);
void test_update_battery_leds_low_charge(void);
void test_is_reg_out_voltage_stable(void);
void test_is_low_voltage_fault_low_load(void);
void test_is_low_voltage_fault_high_load(void);
void test_get_lowest_cell_all_full(void);
void test_get_lowest_cell_one_lowest(void);
void test_get_lowest_cell_no_cells(void);
void test_update_reg_out_voltage_controller_stable_voltage(void);
void test_update_reg_out_voltage_controller_increase_voltage(void);
void test_update_reg_out_voltage_controller_decrease_voltage(void);

// Helpers
int read_potentiometer_value_returns_almost_max(uint8_t* pot_value);
int read_potentiometer_value_returns_max(uint8_t* pot_value);
int read_potentiometer_value_returns_almost_min(uint8_t* pot_value);
int read_potentiometer_value_returns_min(uint8_t* pot_value);

int main(void) {
  test_handle_faults_vbat_out_overcurrent_fault();
  test_handle_faults_reg_out_overcurrent_fault();
  test_handle_faults_low_voltage_fault();
  test_handle_faults_no_fault();
  test_update_battery_cells();
  test_update_battery_charge();
  test_update_battery_charge_no_cells();
  test_update_battery_charge_overcharged();
  test_update_battery_charge_undercharged();
  test_update_battery_leds_full_charge();
  test_update_battery_leds_half_charge();
  test_update_battery_leds_low_charge();
  test_is_reg_out_voltage_stable();
  test_is_low_voltage_fault_low_load();
  test_is_low_voltage_fault_high_load();
  test_get_lowest_cell_all_full();
  test_get_lowest_cell_one_lowest();
  test_get_lowest_cell_no_cells();
  test_update_reg_out_voltage_controller_stable_voltage();
  test_update_reg_out_voltage_controller_increase_voltage();
  test_update_reg_out_voltage_controller_decrease_voltage();
}

void setup_test(void) {
  battery_state_init();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cells.min_voltage = LIPO_CELL_MIN_VOLTAGE_MV;
  battery_state->cells.max_voltage = LIPO_CELL_MAX_VOLTAGE_MV;
  // 4 cell setup
  for (int i = 0; i < 4; i++) {
    battery_state->cells.voltage[i] = LIPO_CELL_MAX_VOLTAGE_MV;
  }
  battery_state->charge = BATTERY_CHARGE_100_PERCENT;
  battery_state->target_reg_out_voltage = 0;

  // Reset fakes after setup to simplify test code
  reset_fakes();
}

void test_handle_faults_vbat_out_overcurrent_fault(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->vbat_out.overcurrent_fault = true;

  handle_faults();

  ASSERT(set_vbat_power_off_fake.call_count == 1, "expected: 1, got: %u",
         set_vbat_power_off_fake.call_count);
  ASSERT(led_signal_fault_fake.call_count == 1, "expected: 1, got: %u",
         led_signal_fault_fake.call_count);
}

void test_handle_faults_reg_out_overcurrent_fault(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->reg_out.overcurrent_fault = true;

  handle_faults();

  ASSERT(set_reg_out_power_off_fake.call_count == 1, "expected: 1, got: %u",
         set_reg_out_power_off_fake.call_count);
  ASSERT(led_signal_fault_fake.call_count == 1, "expected: 1, got: %u",
         led_signal_fault_fake.call_count);
}

void test_handle_faults_low_voltage_fault(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cells.low_voltage_fault = true;

  handle_faults();

  ASSERT(set_vbat_power_off_fake.call_count == 1, "expected: 1, got: %u",
         set_vbat_power_off_fake.call_count);
  ASSERT(led_signal_fault_fake.call_count == 1, "expected: 1, got: %u",
         led_signal_fault_fake.call_count);
}

void test_handle_faults_no_fault(void) {
  setup_test();

  handle_faults();

  ASSERT(set_vbat_power_off_fake.call_count == 0, "expected: 0, got: %u",
         set_vbat_power_off_fake.call_count);
  ASSERT(led_signal_fault_fake.call_count == 0, "expected: 0, got: %u",
         led_signal_fault_fake.call_count);

  // Make sure the LEDs are updated
  ASSERT(set_led_color_fake.call_count == 2, "expected: 2, got: %u",
         set_led_color_fake.call_count);
}

void test_update_battery_cells(void) {
  setup_test();
  const uint16_t adc_value_4000mv = 544;
  uint16_t adc_value_total = 0;
  adc_reading_t adc_reading;

  // Set all 6 battery cells to 4000mv
  for (int i = 0; i < 4; i++) {
    adc_value_total += adc_value_4000mv;
    adc_reading.adc1_buf[i] = adc_value_total;
  }
  adc_value_total += adc_value_4000mv;
  adc_reading.adc2_buf[0] = adc_value_total;
  adc_value_total += adc_value_4000mv;
  adc_reading.adc2_buf[1] = adc_value_total;

  update_battery_cells(&adc_reading);

  const uint16_t expected_voltage_mv = 4000;
  const uint8_t measure_error_mv = 1;
  measurement_t measurement = {
      .expected_value = expected_voltage_mv,
      .accepted_error = measure_error_mv,
  };
  battery_state_t* battery_state = get_battery_state();

  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    measurement.actual_value = battery_state->cells.voltage[i];
    ASSERT(is_acceptable_measurement(&measurement),
           "cell %d: expected voltage: %u, got: %u", i, expected_voltage_mv,
           battery_state->cells.voltage[i]);
  }
}

void test_update_battery_charge(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint16_t half_charge_cell_value =
      (LIPO_CELL_MIN_VOLTAGE_MV + LIPO_CELL_MAX_VOLTAGE_MV) / 2;

  battery_state->cells.voltage[1] = half_charge_cell_value;

  update_battery_charge();

  uint16_t half_charge = BATTERY_CHARGE_100_PERCENT / 2;

  ASSERT(battery_state->charge == half_charge, "expected: %u, got: %u.",
         half_charge, battery_state->charge);
}

void test_update_battery_charge_no_cells(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  // Zero all cell voltages
  memset(battery_state->cells.voltage, 0, sizeof(battery_state->cells.voltage));

  update_battery_charge();

  ASSERT(battery_state->charge == BATTERY_CHARGE_100_PERCENT, "");
}

void test_update_battery_charge_overcharged(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cells.voltage[1] = LIPO_OVERVOLTAGE;

  update_battery_charge();

  ASSERT(battery_state->charge == BATTERY_CHARGE_100_PERCENT,
         "expected: %u, got: %u.", BATTERY_CHARGE_100_PERCENT,
         battery_state->charge);
}

void test_update_battery_charge_undercharged(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cells.voltage[1] = LIPO_UNDERVOLTAGE;

  update_battery_charge();

  ASSERT(battery_state->charge == 0, "expected: 0, got: %u.",
         battery_state->charge);
}

void test_update_battery_leds_full_charge(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->charge = BATTERY_CHARGE_100_PERCENT;

  update_battery_leds();

  ASSERT(set_led_color_fake.call_count == 2, "expected: 2, got: %u",
         set_led_color_fake.call_count);

  ASSERT(set_led_color_fake.arg0_history[0] == LED6, "expected: %d, got: %d",
         LED6, set_led_color_fake.arg0_history[0]);

  ASSERT(set_led_color_fake.arg1_history[0] == GREEN, "expected: %d, got: %d",
         GREEN, set_led_color_fake.arg1_history[0]);

  ASSERT(set_led_color_fake.arg0_history[1] == LED7, "expected: %d, got: %d",
         LED7, set_led_color_fake.arg0_history[1]);

  ASSERT(set_led_color_fake.arg1_history[1] == GREEN, "expected: %d, got: %d",
         GREEN, set_led_color_fake.arg1_history[1]);
}

void test_update_battery_leds_half_charge(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint8_t half_charge_percent = 49;
  battery_state->charge = half_charge_percent;

  update_battery_leds();

  ASSERT(set_led_color_fake.call_count == 2, "expected: 2, got: %u",
         set_led_color_fake.call_count);

  ASSERT(set_led_color_fake.arg0_history[0] == LED6, "expected: %d, got: %d",
         LED6, set_led_color_fake.arg0_history[0]);

  ASSERT(set_led_color_fake.arg1_history[0] == ORANGE, "expected: %d, got: %d",
         ORANGE, set_led_color_fake.arg1_history[0]);

  ASSERT(set_led_color_fake.arg0_history[1] == LED7, "expected: %d, got: %d",
         LED7, set_led_color_fake.arg0_history[1]);

  ASSERT(set_led_color_fake.arg1_history[1] == ORANGE, "expected: %d, got: %d",
         ORANGE, set_led_color_fake.arg1_history[1]);
}

void test_update_battery_leds_low_charge(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint8_t low_charge_percent = 10;
  battery_state->charge = low_charge_percent;

  update_battery_leds();

  ASSERT(set_led_color_fake.call_count == 2, "expected: 2, got: %u",
         set_led_color_fake.call_count);

  ASSERT(set_led_color_fake.arg0_history[0] == LED6, "expected: %d, got: %d",
         LED6, set_led_color_fake.arg0_history[0]);

  ASSERT(set_led_color_fake.arg1_history[0] == RED, "expected: %d, got: %d",
         RED, set_led_color_fake.arg1_history[0]);

  ASSERT(set_led_color_fake.arg0_history[1] == LED7, "expected: %d, got: %d",
         LED7, set_led_color_fake.arg0_history[1]);

  ASSERT(set_led_color_fake.arg1_history[1] == RED, "expected: %d, got: %d",
         RED, set_led_color_fake.arg1_history[1]);
}

void test_is_reg_out_voltage_stable(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint16_t target_voltage = 4000;
  battery_state->target_reg_out_voltage = target_voltage;

  const uint16_t voltage_low = 3900;
  battery_state->reg_out.voltage = voltage_low;
  ASSERT(!is_reg_out_voltage_stable(), "");

  const uint16_t voltage_high = 4100;
  battery_state->reg_out.voltage = voltage_high;
  ASSERT(!is_reg_out_voltage_stable(), "");

  const uint16_t voltage_ok = target_voltage + 1;
  battery_state->reg_out.voltage = voltage_ok;
  ASSERT(is_reg_out_voltage_stable(), "");
}

void test_is_low_voltage_fault_low_load(void) {
  setup_test();

  ASSERT(!is_low_voltage_fault(), "");

  battery_state_t* battery_state = get_battery_state();
  battery_state->vbat_out.current = 0;
  battery_state->cells.voltage[1] = LIPO_UNDERVOLTAGE;

  ASSERT(is_low_voltage_fault(), "");
}

void test_is_low_voltage_fault_high_load(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint32_t acceptable_voltage = 3500;
  battery_state->cells.voltage[1] = acceptable_voltage;
  battery_state->vbat_out.current = DEFAULT_HIGH_LOAD_THRESHOLD_MA;

  ASSERT(!is_low_voltage_fault(), "");

  battery_state->cells.voltage[1] = LIPO_UNDERVOLTAGE;

  ASSERT(is_low_voltage_fault(), "");
}

void test_get_lowest_cell_all_full(void) {
  setup_test();

  uint16_t* lowest_cell = get_lowest_cell();

  ASSERT(lowest_cell != NULL, "lowest_cell should point to something");
  ASSERT(*lowest_cell == LIPO_CELL_MAX_VOLTAGE_MV, "expected: %u, got: %u",
         LIPO_CELL_MAX_VOLTAGE_MV, *lowest_cell);
}

void test_get_lowest_cell_one_lowest(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cells.voltage[1] = LIPO_UNDERVOLTAGE;

  uint16_t* lowest_cell = get_lowest_cell();

  ASSERT(lowest_cell != NULL, "lowest_cell should point to something");
  ASSERT(*lowest_cell == LIPO_UNDERVOLTAGE, "expected: %u, got: %u",
         LIPO_UNDERVOLTAGE, *lowest_cell);
}

void test_get_lowest_cell_no_cells(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  // Zero all cell voltages
  memset(battery_state->cells.voltage, 0, sizeof(battery_state->cells.voltage));

  uint16_t* lowest_cell = get_lowest_cell();

  ASSERT(lowest_cell == NULL, "expected: 0, got: %p", (void*)lowest_cell);
}

void test_update_reg_out_voltage_controller_stable_voltage(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint16_t target_voltage = 4500;
  const uint16_t actual_voltage = target_voltage;
  battery_state->target_reg_out_voltage = target_voltage;
  battery_state->reg_out.voltage = actual_voltage;

  update_reg_out_voltage_controller();

  ASSERT(read_potentiometer_value_fake.call_count == 0, "expected: 0, got: %u",
         read_potentiometer_value_fake.call_count);
  ASSERT(write_potentiometer_value_fake.call_count == 0, "expected: 0, got: %u",
         write_potentiometer_value_fake.call_count);
}

void test_update_reg_out_voltage_controller_increase_voltage(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint16_t target_voltage = 4500;
  const uint16_t actual_voltage = 4000;
  battery_state->target_reg_out_voltage = target_voltage;
  battery_state->reg_out.voltage = actual_voltage;

  int (*custom_fakes[])(uint8_t*) = {
      read_potentiometer_value_returns_almost_max,
      read_potentiometer_value_returns_max,
  };
  SET_CUSTOM_FAKE_SEQ(read_potentiometer_value, custom_fakes, 2);

  update_reg_out_voltage_controller();

  ASSERT(read_potentiometer_value_fake.call_count == 1, "expected: 1, got: %u",
         read_potentiometer_value_fake.call_count);
  ASSERT(write_potentiometer_value_fake.call_count == 1, "expected: 1, got: %u",
         write_potentiometer_value_fake.call_count);

  ASSERT(write_potentiometer_value_fake.arg0_val == UINT8_MAX,
         "expected: %u, got: %u", UINT8_MAX,
         write_potentiometer_value_fake.arg0_val);

  update_reg_out_voltage_controller();

  ASSERT(read_potentiometer_value_fake.call_count == 2, "expected: 2, got: %u",
         read_potentiometer_value_fake.call_count);
  ASSERT(write_potentiometer_value_fake.call_count == 2, "expected: 2, got: %u",
         write_potentiometer_value_fake.call_count);

  // Shouldn't try to increase above UINT8_MAX even though the voltage is still
  // lower than the target votlage.
  ASSERT(write_potentiometer_value_fake.arg0_val == UINT8_MAX,
         "expected: %u, got: %u", UINT8_MAX,
         write_potentiometer_value_fake.arg0_val);
}

void test_update_reg_out_voltage_controller_decrease_voltage(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint16_t target_voltage = 3500;
  const uint16_t actual_voltage = 4000;
  battery_state->target_reg_out_voltage = target_voltage;
  battery_state->reg_out.voltage = actual_voltage;

  int (*custom_fakes[])(uint8_t*) = {
      read_potentiometer_value_returns_almost_min,
      read_potentiometer_value_returns_min,
  };
  SET_CUSTOM_FAKE_SEQ(read_potentiometer_value, custom_fakes, 2);

  update_reg_out_voltage_controller();

  ASSERT(write_potentiometer_value_fake.call_count == 1, "expected: 1, got: %u",
         write_potentiometer_value_fake.call_count);

  ASSERT(write_potentiometer_value_fake.arg0_val == 0, "expected: 0, got: %u",
         write_potentiometer_value_fake.arg0_val);

  update_reg_out_voltage_controller();

  ASSERT(write_potentiometer_value_fake.call_count == 2, "expected: 2, got: %u",
         write_potentiometer_value_fake.call_count);

  // Shouldn't try to decrease below 0 even though the voltage is still
  // higher than the target votlage.
  ASSERT(write_potentiometer_value_fake.arg0_val == 0, "expected: 0, got: %u",
         write_potentiometer_value_fake.arg0_val);
}

int read_potentiometer_value_returns_almost_max(uint8_t* pot_value) {
  *pot_value = UINT8_MAX - 1;
  return APP_OK;
}

int read_potentiometer_value_returns_max(uint8_t* pot_value) {
  *pot_value = UINT8_MAX;
  return APP_OK;
}

int read_potentiometer_value_returns_almost_min(uint8_t* pot_value) {
  *pot_value = 1;
  return APP_OK;
}

int read_potentiometer_value_returns_min(uint8_t* pot_value) {
  *pot_value = 0;
  return APP_OK;
}
