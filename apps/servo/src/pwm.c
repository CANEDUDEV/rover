#include "error.h"
#include "peripherals.h"

void pwm_init(void) {
  peripherals_t *peripherals = get_peripherals();
  if (HAL_TIM_PWM_Start(&peripherals->htim1, TIM_CHANNEL_4) != HAL_OK) {
    error();
  }
}

void pwm_set_pulse(uint32_t pulse_mus) {
  peripherals_t *peripherals = get_peripherals();

  // Write new pulse to timer CCR register
  __HAL_TIM_SET_COMPARE(&peripherals->htim1, TIM_CHANNEL_4, pulse_mus);
}

void pwm_set_frequency(uint16_t frequency_hz) {
  // Timer is prescaled to 1MHz, so a reload value of 1 equals 1 microsecond.
  // Frequency is defined as f = 1/s => s = 1/f => µs = 1 000 000 / f.
  // STM32 timer driver requires value to be subtracted by 1.
  const uint32_t reload_value = (1000 * 1000) / frequency_hz - 1;

  peripherals_t *peripherals = get_peripherals();
  __HAL_TIM_SET_AUTORELOAD(&peripherals->htim1, reload_value);
}
