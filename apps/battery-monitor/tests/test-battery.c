#include <string.h>

#include "battery-internal.h"
#include "battery.h"
#include "led.h"
#include "power.h"

// Testing
#include "battery-test-utils.h"
#include "fff.h"
#include "test.h"

DEFINE_FFF_GLOBALS

// NOLINTBEGIN
FAKE_VOID_FUNC(set_vbat_power_on)
FAKE_VOID_FUNC(set_vbat_power_off)
FAKE_VOID_FUNC(set_reg_vout_power_on)
FAKE_VOID_FUNC(set_reg_vout_power_off)
FAKE_VOID_FUNC(led_init)
FAKE_VOID_FUNC(led_signal_fault)
FAKE_VOID_FUNC(led_stop_signal_fault)
FAKE_VOID_FUNC(set_led_color, led_t, led_color_t)
FAKE_VALUE_FUNC(int, read_potentiometer_value, uint8_t*)
FAKE_VALUE_FUNC(power_state_t, get_vbat_power_state)
FAKE_VALUE_FUNC(power_state_t, get_reg_vout_power_state)
// NOLINTEND

#define FFF_FAKES_LIST(FAKE)     \
  FAKE(set_vbat_power_on)        \
  FAKE(set_vbat_power_off)       \
  FAKE(set_reg_vout_power_on)    \
  FAKE(set_reg_vout_power_off)   \
  FAKE(led_init)                 \
  FAKE(led_signal_fault)         \
  FAKE(led_stop_signal_fault)    \
  FAKE(set_led_color)            \
  FAKE(read_potentiometer_value) \
  FAKE(get_vbat_power_state)     \
  FAKE(get_reg_vout_power_state)

#define LIPO_OVERVOLTAGE (LIPO_CELL_MAX_VOLTAGE + 100)
#define LIPO_UNDERVOLTAGE (LIPO_CELL_MIN_VOLTAGE - 100)

void test_handle_faults_over_current_fault(void);
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
void test_is_low_voltage_fault(void);
void test_is_over_current_fault(void);
void test_get_lowest_cell_all_full(void);
void test_get_lowest_cell_one_lowest(void);
void test_get_lowest_cell_no_cells(void);

int main(void) {
  test_handle_faults_over_current_fault();
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
  test_is_low_voltage_fault();
  test_is_over_current_fault();
  test_get_lowest_cell_all_full();
  test_get_lowest_cell_one_lowest();
  test_get_lowest_cell_no_cells();
}

void setup_test(void) {
  battery_state_init();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cell_min_voltage = LIPO_CELL_MIN_VOLTAGE;
  battery_state->cell_max_voltage = LIPO_CELL_MAX_VOLTAGE;
  // 4 cell setup
  for (int i = 0; i < 4; i++) {
    battery_state->cell_voltage[i] = LIPO_CELL_MAX_VOLTAGE;
  }
  battery_state->charge = BATTERY_CHARGE_100_PERCENT;

  // Reset fakes after setup to simplify test code
  FFF_FAKES_LIST(RESET_FAKE);
  FFF_RESET_HISTORY();
}

void test_handle_faults_over_current_fault(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->over_current_fault = true;

  handle_faults();

  ASSERT(set_vbat_power_off_fake.call_count == 1, "expected: 1, got: %u",
         set_vbat_power_off_fake.call_count);
  ASSERT(led_signal_fault_fake.call_count == 1, "expected: 1, got: %u",
         led_signal_fault_fake.call_count);
}

void test_handle_faults_low_voltage_fault(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->low_voltage_fault = true;

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
    measurement.actual_value = battery_state->cell_voltage[i];
    ASSERT(is_acceptable_measurement(&measurement),
           "cell %d: expected voltage: %u, got: %u", i, expected_voltage_mv,
           battery_state->cell_voltage[i]);
  }
}

void test_update_battery_charge(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  const uint16_t half_charge_cell_value =
      (LIPO_CELL_MIN_VOLTAGE + LIPO_CELL_MAX_VOLTAGE) / 2;

  battery_state->cell_voltage[1] = half_charge_cell_value;

  update_battery_charge();

  uint16_t half_charge = BATTERY_CHARGE_100_PERCENT / 2;

  ASSERT(battery_state->charge == half_charge, "expected: %u, got: %u.",
         half_charge, battery_state->charge);
}

void test_update_battery_charge_no_cells(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  // Zero all cell voltages
  memset(battery_state->cell_voltage, 0, sizeof(battery_state->cell_voltage));

  update_battery_charge();

  ASSERT(battery_state->charge == BATTERY_CHARGE_100_PERCENT, "");
}

void test_update_battery_charge_overcharged(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cell_voltage[1] = LIPO_OVERVOLTAGE;

  update_battery_charge();

  ASSERT(battery_state->charge == BATTERY_CHARGE_100_PERCENT,
         "expected: %u, got: %u.", BATTERY_CHARGE_100_PERCENT,
         battery_state->charge);
}

void test_update_battery_charge_undercharged(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cell_voltage[1] = LIPO_UNDERVOLTAGE;

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

void test_is_low_voltage_fault(void) {
  setup_test();

  ASSERT(!is_low_voltage_fault(), "");

  battery_state_t* battery_state = get_battery_state();
  battery_state->cell_voltage[1] = LIPO_UNDERVOLTAGE;

  ASSERT(is_low_voltage_fault(), "");
}

void test_is_over_current_fault(void) {
  setup_test();
  ASSERT(!is_over_current_fault(), "");

  battery_state_t* battery_state = get_battery_state();
  const uint32_t current_1a = 1000;
  battery_state->over_current_threshold = current_1a;
  battery_state->vbat_out_current = 2 * current_1a;

  ASSERT(is_over_current_fault(), "");

  const uint32_t current_100a = 100 * 1000;
  battery_state->over_current_threshold = current_100a;

  ASSERT(!is_over_current_fault(), "");
}

void test_get_lowest_cell_all_full(void) {
  setup_test();

  uint16_t* lowest_cell = get_lowest_cell();

  ASSERT(lowest_cell != NULL, "lowest_cell should point to something");
  ASSERT(*lowest_cell == LIPO_CELL_MAX_VOLTAGE, "expected: %u, got: %u",
         LIPO_CELL_MAX_VOLTAGE, *lowest_cell);
}

void test_get_lowest_cell_one_lowest(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  battery_state->cell_voltage[1] = LIPO_UNDERVOLTAGE;

  uint16_t* lowest_cell = get_lowest_cell();

  ASSERT(lowest_cell != NULL, "lowest_cell should point to something");
  ASSERT(*lowest_cell == LIPO_UNDERVOLTAGE, "expected: %u, got: %u",
         LIPO_UNDERVOLTAGE, *lowest_cell);
}

void test_get_lowest_cell_no_cells(void) {
  setup_test();
  battery_state_t* battery_state = get_battery_state();
  // Zero all cell voltages
  memset(battery_state->cell_voltage, 0, sizeof(battery_state->cell_voltage));

  uint16_t* lowest_cell = get_lowest_cell();

  ASSERT(lowest_cell == NULL, "expected: 0, got: %p", (void*)lowest_cell);
}
