#include "ck-rx-letters.h"

#include <math.h>
#include <string.h>

#include "error.h"
#include "failsafe.h"
#include "freertos-tasks.h"
#include "potentiometer.h"
#include "pwm.h"

// y = kx + m. 500 µs pulse corresponds to 45 degree movement,
// yielding k = 500/45. m is the neutral position.
static const float k_angle_to_pulse = 500 / 45.0F;
static const float m_angle_to_pulse = PWM_NEUTRAL_PULSE_MUS;

static int16_t steering_pulse = PWM_NEUTRAL_PULSE_MUS;

// Whether to reverse steering direction
static bool reverse = false;

// 2 bytes in page
// bytes 1-2: desired servo voltage in mV. Allowed range: 2740-10800 mV
int process_set_servo_voltage_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 2) {
    return APP_NOT_OK;
  }

  uint16_t target_voltage = 0;

  memcpy(&target_voltage, letter->page.lines, sizeof(target_voltage));

  const uint16_t min_voltage = 2740;
  const uint16_t max_voltage = 10800;

  if (target_voltage < min_voltage || target_voltage > max_voltage) {
    return APP_NOT_OK;
  }

  // Values obtained by using VCC_SERVO measurements on the ADC for various
  // potentiometer values, then fitted to a curve.
  //
  // The relationship between voltage and potentiometer roughly follows this
  // equation:
  //
  // y = (a/x)^b + c,
  //
  // where x = votlage in V, y = potentiometer value, a = 187.32493879, b
  // = 1.35, and c = -47.26858055.

  // NOLINTBEGIN(readability-identifier-length)
  const float a = 187.32493879F;
  const float b = 1.35F;
  const float c = -47.26858055F;
  // NOLINTEND(readability-identifier-length)

  const float voltage_f = (float)target_voltage / 1000;  // Convert from mV to V
  float potentiometer_f = powf((a / voltage_f), b) + c;

  float potentiometer_f_rounded = roundf(potentiometer_f);
  uint8_t potentiometer = 0;
  const uint8_t potentiometer_max = 0xFF;
  if (potentiometer_f_rounded < 0) {
    potentiometer = 0;
  } else if (potentiometer_f_rounded > (float)potentiometer_max) {
    potentiometer = potentiometer_max;
  } else {
    potentiometer = (uint8_t)potentiometer_f_rounded;
  }

  configure_servo_potentiometer(potentiometer);

  return APP_OK;
}

// 2 bytes in page, the PWM frequency in Hz. At most 333 Hz.
int process_pwm_conf_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 2) {
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
  if (letter->page.line_count != 5) {  // NOLINT
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

  if (reverse) {
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
  if (letter->page.line_count != 2) {
    return APP_NOT_OK;
  }

  int16_t trim_pulse = 0;
  memcpy(&trim_pulse, letter->page.lines, sizeof(trim_pulse));

  if (reverse) {
    trim_pulse = (int16_t)-trim_pulse;
  }

  pwm_set_subtrim_pulse(trim_pulse);

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

int process_reverse_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 0) {
    return APP_NOT_OK;
  }
  reverse = !reverse;
  return APP_OK;
}

int process_failsafe_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 5) {  // NOLINT
    return APP_NOT_OK;
  }

  // byte 0: on/off/keep current. 0 = off, 1 = on, all others = keep current
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
  memcpy(&pulse, &letter->page.lines[3], sizeof(timeout));
  if (pulse) {
    failsafe_set_pulse(pulse);
  }

  return APP_OK;
}
