#include <stdbool.h>

#include "adc.h"

// Testing
#include "test.h"

void test_adc_to_servo_position_neutral(void);
void test_adc_to_servo_position_min(void);
void test_adc_to_servo_position_max(void);
void test_adc_to_servo_current(void);
void test_adc_to_battery_voltage(void);
void test_adc_to_servo_voltage(void);

int main(void) {
  test_adc_to_servo_position_neutral();
  test_adc_to_servo_position_min();
  test_adc_to_servo_position_max();
  test_adc_to_servo_current();
  test_adc_to_battery_voltage();
  test_adc_to_servo_voltage();
}

void test_adc_to_servo_position_neutral(void) {
  const uint16_t adc_value_0deg = 2047;
  const int16_t expected_position_deg = 0;
  const int16_t accepted_error_deg = 0;

  measurement_t measurement = {
      .actual_value = adc_to_servo_position(adc_value_0deg),
      .expected_value = expected_position_deg,
      .accepted_error = accepted_error_deg,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %.0f, got: %.0f",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_servo_position_min(void) {
  const uint16_t adc_value_negative_180deg = 0;
  const int16_t expected_position_deg = -180;
  const int16_t accepted_error_deg = 0;

  measurement_t measurement = {
      .actual_value = adc_to_servo_position(adc_value_negative_180deg),
      .expected_value = expected_position_deg,
      .accepted_error = accepted_error_deg,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %.0f, got: %.0f",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_servo_position_max(void) {
  const uint16_t adc_value_180deg = 4095;
  const int16_t expected_position_deg = 180;
  const int16_t accepted_error_deg = 0;

  measurement_t measurement = {
      .actual_value = adc_to_servo_position(adc_value_180deg),
      .expected_value = expected_position_deg,
      .accepted_error = accepted_error_deg,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %.0f, got: %.0f",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_servo_current(void) {
  // Current measured using power supply
  const uint16_t adc_value_450ma = 130;
  const uint16_t expected_current_ma = 450;
  const uint16_t accepted_error_ma = 50;

  measurement_t measurement = {
      .actual_value = adc_to_servo_current(adc_value_450ma),
      .expected_value = expected_current_ma,
      .accepted_error = accepted_error_ma,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %.0f, got: %.0f",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_battery_voltage(void) {
  const uint16_t adc_value_12v = 1695;
  const int16_t expected_voltage_mv = 12000;
  const int16_t accepted_error_mv = 50;

  measurement_t measurement = {
      .actual_value = adc_to_battery_voltage(adc_value_12v),
      .expected_value = expected_voltage_mv,
      .accepted_error = accepted_error_mv,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %.0f, got: %.0f",
         measurement.expected_value, measurement.actual_value);
}

void test_adc_to_servo_voltage(void) {
  const uint16_t adc_value_7400mv = 2564;
  const int16_t expected_voltage_mv = 7400;
  const int16_t accepted_error_mv = 50;

  measurement_t measurement = {
      .actual_value = adc_to_servo_voltage(adc_value_7400mv),
      .expected_value = expected_voltage_mv,
      .accepted_error = accepted_error_mv,
  };

  ASSERT(is_acceptable_measurement(&measurement), "expected: %.0f, got: %.0f",
         measurement.expected_value, measurement.actual_value);
}
