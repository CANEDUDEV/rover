#include "ck-rx-letters.h"

#include <string.h>

#include "adc.h"
#include "error.h"
#include "freertos-tasks.h"
#include "peripherals.h"
#include "ports.h"
#include "potentiometer.h"

// y = kx + m. 500 µs pulse corresponds to 45 degree movement,
// yielding k = 500/45.
//
// m is the neutral position, i.e. 1500 µs pulse.
const float k_angle_to_pulse = 500 / 45.0F;
const float m_angle_to_pulse = 1500;

static int16_t steering_pulse = (int16_t)m_angle_to_pulse;
static int16_t steering_trim_pulse = 0;

// 2 bytes in page
int process_set_servo_voltage_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 2) {
    return APP_NOT_OK;
  }

  configure_servo_potentiometer(letter->page.lines[0]);
  return APP_OK;
}

// 2 bytes in page, the PWM frequency in Hz. At most 333 Hz.
int process_pwm_conf_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 2) {  // NOLINT
    return APP_NOT_OK;
  }

  uint16_t frequency = 0;
  memcpy(&frequency, letter->page.lines, sizeof(frequency));

  if (frequency == 0 || frequency > 333) {  // NOLINT
    return APP_NOT_OK;
  }

  // Timer is prescaled to 1MHz, so a reload value of 1 equals 1 microsecond.
  // Frequency is defined as f = 1/s => s = 1/f => µs = 1 000 000 / f.
  // STM32 timer driver requires value to be subtracted by 1.
  const uint32_t reload_value = (1000 * 1000) / frequency - 1;

  peripherals_t *peripherals = get_peripherals();
  __HAL_TIM_SET_AUTORELOAD(&peripherals->htim1, reload_value);

  return APP_OK;
}

// 3 bytes in page
// byte 0: can either be 0 or 1.
//
// If 0, interpret bytes 1-2 as a pulse-width in microseconds. If 1, interpret
// bytes 1-2 as a signed value representing a steering angle, with a range of
// -90 to 90 degrees.
//
// -90 degrees pulse width: 500 microseconds.
// +90 degrees pulse width: 2500 microseconds.
int process_steering_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 3) {
    return APP_NOT_OK;
  }

  peripherals_t *peripherals = get_peripherals();
  int16_t pulse = 0;

  switch (letter->page.lines[0]) {
    case 0:
      memcpy(&pulse, &letter->page.lines[1], sizeof(pulse));
      break;

    case 1: {
      int16_t angle = 0;
      memcpy(&angle, &letter->page.lines[1], sizeof(angle));
      if (angle < -90 || angle > 90) {  // NOLINT
        return APP_NOT_OK;
      }

      float pulse_float = (float)angle * k_angle_to_pulse + m_angle_to_pulse;

      // Round it
      pulse = (uint16_t)(pulse_float + 0.5);  // NOLINT
      break;
    }

    default:
      return APP_NOT_OK;
  }

  steering_pulse = pulse;
  // Write new pulse to CCR register
  __HAL_TIM_SET_COMPARE(&peripherals->htim1, TIM_CHANNEL_4,
                        (uint32_t)(steering_pulse + steering_trim_pulse));

  return APP_OK;
}

// 3 bytes in page
// byte 0: can either be 0 or 1.
//
// bytes 1-2: a signed integer representing a steering trim either as a
// pulse-width in microseconds or an angle in degrees. If byte 0 is 0, interpret
// bytes 1-2 as a pulse-width. If byte 0 is 1, interpret bytes 1-2 as an angle
// with a range of -45 to 45 degrees.
//
// -45 degrees pulse width: -500 microseconds.
// +45 degrees pulse width: +500 microseconds.
int process_steering_trim_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 3) {
    return APP_NOT_OK;
  }

  peripherals_t *peripherals = get_peripherals();
  int16_t trim_pulse = 0;

  switch (letter->page.lines[0]) {
    case 0:
      memcpy(&trim_pulse, &letter->page.lines[1], sizeof(trim_pulse));
      break;

    case 1: {
      int16_t angle = 0;
      memcpy(&angle, &letter->page.lines[1], sizeof(angle));
      if (angle < -45 || angle > 45) {  // NOLINT
        return APP_NOT_OK;
      }

      // Ignore m since we're calculating an offset
      const float trim_pulse_float = (float)angle * k_angle_to_pulse;

      // Round it
      if (trim_pulse < 0) {
        trim_pulse = (int16_t)(trim_pulse_float - 0.5);  // NOLINT
      } else {
        trim_pulse = (int16_t)(trim_pulse_float + 0.5);  // NOLINT
      }
      break;
    }

    default:
      return APP_NOT_OK;
  }

  steering_trim_pulse = trim_pulse;

  // Write new pulse to CCR register
  __HAL_TIM_SET_COMPARE(&peripherals->htim1, TIM_CHANNEL_4,
                        (uint32_t)(steering_pulse + steering_trim_pulse));

  return APP_OK;
}

// 4 bytes in page
//
// bytes 0-1: measuring period in ms, i.e. how often to measure voltage and
//            current.
// bytes 2-3: reporting period in ms, i.e. how often to send
//            measurements over CAN.
int process_report_freq_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 4) {
    return APP_NOT_OK;
  }
  task_periods_t periods;
  memcpy(&periods.measure_period_ms, letter->page.lines,
         sizeof(periods.measure_period_ms));
  memcpy(&periods.report_period_ms, &letter->page.lines[2],
         sizeof(periods.report_period_ms));

  set_task_periods(&periods);
  return APP_OK;
}
