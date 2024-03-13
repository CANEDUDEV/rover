#include "failsafe.h"

#include <stdbool.h>
#include <stdio.h>

#include "pwm.h"
#include "servo.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

static StaticTimer_t timer_buf;
static TimerHandle_t timer;
static bool failsafe_active = false;
static uint16_t failsafe_timeout = FAILSAFE_DEFAULT_TIMEOUT_MS;
static uint16_t failsafe_pulse = PWM_NEUTRAL_PULSE_MUS;

void failsafe_timer_callback(TimerHandle_t timer) {
  (void)timer;
  servo_state_t *servo_state = get_servo_state();
  if (servo_state->steering_pulse != failsafe_pulse) {
    update_servo_pulse(failsafe_pulse);
    printf("Failsafe triggered.\r\n");
  }
}

void failsafe_init(void) {
  timer = xTimerCreateStatic("failsafe timer", pdMS_TO_TICKS(failsafe_timeout),
                             pdFALSE,  // Don't auto reload timer
                             NULL,     // Timer ID, unused
                             failsafe_timer_callback, &timer_buf);
}

void failsafe_refresh(void) {
  if (!failsafe_active) {
    return;
  }

  if (xTimerIsTimerActive(timer)) {
    xTimerStop(timer, portMAX_DELAY);
  }

  // Start the timer with the most recent timeout value.
  xTimerChangePeriod(timer, pdMS_TO_TICKS(failsafe_timeout), portMAX_DELAY);
}

void failsafe_on(void) {
  failsafe_active = true;

  // Start the timer with the most recent timeout value.
  xTimerChangePeriod(timer, pdMS_TO_TICKS(failsafe_timeout), portMAX_DELAY);
}

void failsafe_off(void) {
  failsafe_active = false;
  xTimerStop(timer, portMAX_DELAY);
}

void failsafe_set_timeout(uint16_t timeout_ms) {
  failsafe_timeout = timeout_ms;
}

void failsafe_set_pulse(uint16_t pulse_mus) {
  failsafe_pulse = pulse_mus;
}
