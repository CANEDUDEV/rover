#include "servo.h"

#include "error.h"
#include "potentiometer.h"

void update_voltage_controller(void);
bool is_servo_voltage_stable(void);

static servo_state_t servo;

servo_state_t* get_servo_state(void) {
  return &servo;
}

void servo_init(void) {
  servo.target_voltage = DEFAULT_SERVO_VOLTAGE_MV;
  servo.voltage = 0;
  servo.current = 0;
  servo.position = 0;
  servo.reverse = false;
}

void update_servo_state(adc_reading_t* adc_reading) {
  servo.voltage = adc_to_servo_voltage(adc_reading->adc2_buf[0]);
  servo.current = adc_to_servo_current(adc_reading->adc1_buf[1]);
  servo.position = adc_to_servo_position(adc_reading->adc1_buf[0]);
  update_voltage_controller();
}

void update_voltage_controller(void) {
  if (is_servo_voltage_stable()) {
    return;
  }

  uint8_t current_pot_value = 0;
  if (read_servo_potentiometer(&current_pot_value) != APP_OK) {
    return;
  }

  int16_t next_pot_value = current_pot_value;

  if (servo.voltage > servo.target_voltage && next_pot_value < UINT8_MAX) {
    next_pot_value++;
  }

  if (servo.voltage < servo.target_voltage && next_pot_value > 0) {
    next_pot_value--;
  }

  write_servo_potentiometer(next_pot_value);
}

bool is_servo_voltage_stable(void) {
  const uint8_t accepted_error = 10;
  int32_t max_voltage = (int32_t)servo.target_voltage + accepted_error;
  int32_t min_voltage = (int32_t)servo.target_voltage - accepted_error;

  return servo.voltage < max_voltage && servo.voltage > min_voltage;
}
