#include "adc.h"
#include "battery.h"
#include "jumpers.h"

// Testing
#include "battery-test-utils.h"
#include "fff.h"
#include "test.h"

DEFINE_FFF_GLOBALS

// NOLINTBEGIN
// Mock these functions to be able to buiid jumpers.c
FAKE_VALUE_FUNC(uint8_t, read_potentiometer_value)
FAKE_VALUE_FUNC(battery_state_t *, get_battery_state)
// NOLINTEND

void test_adc_to_cell_voltage(void);
void test_adc_to_reg_out_current(void);
void test_adc_to_reg_out_voltage(void);
void test_adc_to_vbat_out_current_x11_on_x12_on(void);
void test_adc_to_vbat_out_current_x11_on(void);
void test_adc_to_vbat_out_current_x12_on(void);
void test_adc_to_vbat_out_current_x11_off_x12_off(void);
void test_adc_to_vbat_out_voltage(void);

int main(void) {
  test_adc_to_cell_voltage();
  test_adc_to_reg_out_current();
  test_adc_to_reg_out_voltage();
  test_adc_to_vbat_out_current_x11_on_x12_on();
  test_adc_to_vbat_out_current_x11_on();
  test_adc_to_vbat_out_current_x12_on();
  test_adc_to_vbat_out_current_x11_off_x12_off();
  test_adc_to_vbat_out_voltage();
}

void test_adc_to_cell_voltage(void) {
  const uint16_t adc_value_4000mv = 550;
  const uint16_t expected_voltage_mv = 4000;
  const uint16_t accepted_error_mv = 50;

  measurement_t measurement = {
      .actual_value = adc_to_cell_voltage(adc_value_4000mv),
      .expected_value = expected_voltage_mv,
      .accepted_error = accepted_error_mv,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_reg_out_current(void) {
  const uint16_t adc_value_1500ma = 931;
  const uint16_t expected_current_ma = 1500;
  const uint16_t accepted_error_ma = 30;

  measurement_t measurement = {
      .actual_value = adc_to_reg_out_current(adc_value_1500ma),
      .expected_value = expected_current_ma,
      .accepted_error = accepted_error_ma,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_reg_out_voltage(void) {
  const uint16_t adc_value_6100mv = 1335;
  const uint16_t expected_voltage_mv = 6100;
  const uint16_t accepted_error_mv = 50;

  measurement_t measurement = {
      .actual_value = adc_to_reg_out_voltage(adc_value_6100mv),
      .expected_value = expected_voltage_mv,
      .accepted_error = accepted_error_mv,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_vbat_out_current_x11_on_x12_on(void) {
  const uint16_t adc_value_66a = 2048;
  const int32_t expected_current_ma = 66 * 1000;
  const uint16_t accepted_error_ma = 300;

  set_current_measure_jumper_config(X11_ON_X12_ON);

  measurement_t measurement = {
      .actual_value = (int32_t)adc_to_vbat_out_current(adc_value_66a),
      .expected_value = expected_current_ma,
      .accepted_error = accepted_error_ma,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_vbat_out_current_x11_on(void) {
  const uint16_t adc_value_33a = 2048;
  const uint16_t expected_current_ma = 33 * 1000;
  const uint16_t accepted_error_ma = 300;

  set_current_measure_jumper_config(X11_ON_X12_OFF);

  measurement_t measurement = {
      .actual_value = (int32_t)adc_to_vbat_out_current(adc_value_33a),
      .expected_value = expected_current_ma,
      .accepted_error = accepted_error_ma,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_vbat_out_current_x12_on(void) {
  const uint16_t adc_value_49a = 2047;
  const uint16_t expected_current_ma = 49 * 1000;
  const uint16_t accepted_error_ma = 300;

  set_current_measure_jumper_config(X11_OFF_X12_ON);

  measurement_t measurement = {
      .actual_value = (int32_t)adc_to_vbat_out_current(adc_value_49a),
      .expected_value = expected_current_ma,
      .accepted_error = accepted_error_ma,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_vbat_out_current_x11_off_x12_off(void) {
  const uint16_t adc_value_16a = 2000;
  const uint16_t expected_current_ma = 16 * 1000;
  const uint16_t accepted_error_ma = 300;

  set_current_measure_jumper_config(X11_OFF_X12_OFF);

  measurement_t measurement = {
      .actual_value = (int32_t)adc_to_vbat_out_current(adc_value_16a),
      .expected_value = expected_current_ma,
      .accepted_error = accepted_error_ma,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_vbat_out_voltage(void) {
  const uint16_t adc_value_12000mv = 1637;
  const uint16_t expected_voltage_mv = 12000;
  const uint16_t accepted_error_mv = 50;

  measurement_t measurement = {
      .actual_value = adc_to_vbat_out_voltage(adc_value_12000mv),
      .expected_value = expected_voltage_mv,
      .accepted_error = accepted_error_mv,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %u, got: %u",
         measurement.expected_value, measurement.actual_value);
}
