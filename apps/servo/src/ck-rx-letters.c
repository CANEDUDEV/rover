#include "ck-rx-letters.h"

#include <math.h>
#include <string.h>

#include "ck-data.h"
#include "error.h"
#include "failsafe.h"
#include "freertos-tasks.h"
#include "pwm.h"
#include "servo.h"

// y = kx + m. 500 µs pulse corresponds to 45 degree movement,
// yielding k = 500/45. m is the neutral position.
static const float k_angle_to_pulse = 500 / 45.0F;
static const float m_angle_to_pulse = PWM_NEUTRAL_PULSE_MUS;

static int16_t steering_pulse = PWM_NEUTRAL_PULSE_MUS;

// 2 bytes in page
// bytes 1-2: desired servo voltage in mV.
int process_set_servo_voltage_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->set_servo_voltage_folder->dlc) {
    return APP_NOT_OK;
  }

  servo_t *servo = get_servo_state();

  memcpy(&servo->target_voltage, letter->page.lines,
         sizeof(servo->target_voltage));

  return APP_OK;
}

// 2 bytes in page, the PWM frequency in Hz. At most 333 Hz.
int process_pwm_conf_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->pwm_conf_folder->dlc) {
    return APP_NOT_OK;
  }

  uint16_t frequency = 0;
  memcpy(&frequency, letter->page.lines, sizeof(frequency));

  if (frequency == 0 || frequency > PWM_MAX_FREQUENCY_HZ) {
    return APP_NOT_OK;
  }

  pwm_set_frequency(frequency);

  return APP_OK;
}

// 5 bytes in page
// byte 0: can either be 0 or 1.
//
// If byte0 == 0, interpret bytes 1-2 as a pulse-width in microseconds.
// Bytes 3-4 are ignored.
//
// If byte0 == 1, interpret bytes 1-4 as a float representing a
// steering angle, with a range of -90 to 90 degrees.
//
// -90 degrees pulse width: 500 microseconds.
// +90 degrees pulse width: 2500 microseconds.
int process_steering_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->steering_folder->dlc) {
    return APP_NOT_OK;
  }

  int16_t pulse = 0;
  float pulse_float = 0;

  switch (letter->page.lines[0]) {
    case 0:
      memcpy(&pulse, &letter->page.lines[1], sizeof(pulse));
      pulse_float = pulse;
      break;

    case 1: {
      float angle = 0;
      memcpy(&angle, &letter->page.lines[1], sizeof(angle));
      if (angle < -90 || angle > 90) {  // NOLINT
        return APP_NOT_OK;
      }

      pulse_float = angle * k_angle_to_pulse + m_angle_to_pulse;

      break;
    }

    default:
      return APP_NOT_OK;
  }

  servo_t *servo_state = get_servo_state();
  if (servo_state->reverse) {
    pulse_float = 2 * m_angle_to_pulse - pulse_float;
  }

  steering_pulse = (int16_t)roundf(pulse_float);

  pwm_set_pulse(steering_pulse);
  failsafe_refresh();

  return APP_OK;
}

// 2 bytes in page.
// Signed integer representing a subtrim value as a pulse width in microseconds.
int process_subtrim_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->subtrim_folder->dlc) {
    return APP_NOT_OK;
  }

  int16_t trim_pulse = 0;
  memcpy(&trim_pulse, letter->page.lines, sizeof(trim_pulse));

  servo_t *servo_state = get_servo_state();
  if (servo_state->reverse) {
    trim_pulse = (int16_t)-trim_pulse;
  }

  pwm_set_subtrim_pulse(trim_pulse);

  return APP_OK;
}

// 2 bytes in page
//
// bytes 0-1: reporting period in ms, i.e. how often to send
//            measurements over CAN.
int process_report_freq_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->report_freq_folder->dlc) {
    return APP_NOT_OK;
  }

  task_periods_t periods;
  memcpy(&periods.report_period_ms, letter->page.lines,
         sizeof(periods.report_period_ms));

  set_task_periods(&periods);
  return APP_OK;
}

int process_reverse_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->reverse_folder->dlc) {
    return APP_NOT_OK;
  }

  servo_t *servo_state = get_servo_state();
  servo_state->reverse = !servo_state->reverse;

  return APP_OK;
}

// 5 bytes in page
//
// byte 0: Failsafe state, 0 = off, 1 = on, all others ignored
//
// bytes 1-2: Failsafe timeout period in ms, unsigned 16-bit int.
//
// bytes 3-4: Failsafe PWM pulse setting, unsigned 16-bit int.
//
int process_failsafe_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->failsafe_folder->dlc) {
    return APP_NOT_OK;
  }

  if (letter->page.lines[0] == 0) {
    failsafe_off();
  } else if (letter->page.lines[0] == 1) {
    failsafe_on();
  }

  // 0 = keep current setting
  uint16_t timeout = 0;
  memcpy(&timeout, &letter->page.lines[1], sizeof(timeout));
  if (timeout) {
    failsafe_set_timeout(timeout);
  }

  // 0 = keep current setting
  uint16_t pulse = 0;
  memcpy(&pulse, &letter->page.lines[3], sizeof(pulse));
  if (pulse) {
    failsafe_set_pulse(pulse);
  }

  return APP_OK;
}
