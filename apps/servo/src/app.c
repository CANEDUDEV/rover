#include "app.h"

#include <string.h>

#include "peripherals.h"

/* Going towards the minimum pulse gives steering to the right. Going towards
 * maximum pulse gives steering to the left.
 */
void UpdatePWMDutyCycle(uint32_t *pulse, STEERING_DIRECTION *direction) {
  peripherals_t *peripherals = get_peripherals();

  const uint32_t minPulse =
      peripherals->htim1.Init.Period / 20;  // 5% duty cycle

  const uint32_t maxPulse =
      peripherals->htim1.Init.Period / 10;  // 10% duty cycle

  const uint8_t pulseStep = 10;

  if (*pulse <= minPulse) {
    *pulse = minPulse;
    *direction = LEFT;
  } else if (*pulse >= maxPulse) {
    *pulse = maxPulse;
    *direction = RIGHT;
  }

  if (*direction == LEFT) {
    *pulse += pulseStep;
  } else {
    *pulse -= pulseStep;
  }

  // Write new pulse to CCR register
  __HAL_TIM_SET_COMPARE(&peripherals->htim1, TIM_CHANNEL_4, *pulse);
}
