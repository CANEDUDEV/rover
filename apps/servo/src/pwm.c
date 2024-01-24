#include "pwm.h"

#include <stdio.h>

#include "error.h"
#include "lfs-wrapper.h"
#include "peripherals.h"

static int32_t subtrim;
static const char *subtrim_filename = "/subtrim";
static file_t subtrim_file;

void pwm_init(void) {
  lfs_init();

  subtrim_file.name = subtrim_filename;
  subtrim_file.data = &subtrim;
  subtrim_file.size = sizeof(subtrim);

  if (read_file(&subtrim_file) != APP_OK) {
    printf("Couldn't read subtrim file %s, using default subtrim.\r\n",
           subtrim_filename);
    subtrim = 0;
  }

  peripherals_t *peripherals = get_peripherals();
  if (HAL_TIM_PWM_Start(&peripherals->htim1, TIM_CHANNEL_4) != HAL_OK) {
    error();
  }
}

void pwm_set_pulse(uint32_t pulse_mus) {
  peripherals_t *peripherals = get_peripherals();

  // Write new pulse to timer CCR register
  __HAL_TIM_SET_COMPARE(&peripherals->htim1, TIM_CHANNEL_4,
                        pulse_mus + subtrim);
}

void pwm_set_subtrim_pulse(int16_t pulse_mus) {
  int32_t new_pulse = pulse_mus;
  if (new_pulse > PWM_SUBTRIM_MAX_PULSE) {
    new_pulse = PWM_SUBTRIM_MAX_PULSE;
  }
  if (new_pulse < PWM_SUBTRIM_MIN_PULSE) {
    new_pulse = PWM_SUBTRIM_MIN_PULSE;
  }

  subtrim = new_pulse;

  write_file_async(&subtrim_file);
}

void pwm_set_frequency(uint16_t frequency_hz) {
  // Timer is prescaled to 1MHz, so a reload value of 1 equals 1 microsecond.
  // Frequency is defined as f = 1/s => s = 1/f => µs = 1 000 000 / f.
  // STM32 timer driver requires value to be subtracted by 1.
  const uint32_t reload_value = (1000 * 1000) / frequency_hz - 1;

  peripherals_t *peripherals = get_peripherals();
  __HAL_TIM_SET_AUTORELOAD(&peripherals->htim1, reload_value);
}
