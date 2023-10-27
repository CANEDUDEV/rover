#include "ck-rx-letters.h"

#include <math.h>
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

      pulse = (int16_t)roundf(pulse_float);
      break;
    }

    default:
      return APP_NOT_OK;
  }

  if (reverse) {
    steering_pulse = (int16_t)((int16_t)(2 * m_angle_to_pulse) - pulse);
  } else {
    steering_pulse = pulse;
  }

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

      trim_pulse = (int16_t)roundf(trim_pulse_float);
      break;
    }

    default:
      return APP_NOT_OK;
  }

  if (reverse) {
    steering_trim_pulse = (int16_t)-trim_pulse;
  } else {
    steering_trim_pulse = trim_pulse;
  }

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

int process_reverse_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 0) {
    return APP_NOT_OK;
  }
  reverse = !reverse;
  return APP_OK;
}
