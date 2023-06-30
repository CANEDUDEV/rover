#include "app.h"

#include <string.h>

#include "peripherals.h"

#define POT_ADDR (0x50 << 1)  // Shift left to match STM32 specification
#define POT_IVRA_ADDR 0x0     // VCC_Servo potentiometer
#define POT_IVRB_ADDR 0x1     // VDD_Sensor potentiometer

#define POT_IVRA_DEFAULT 40
#define POT_IVRB_DEFAULT 40

/* Set up potentiomenter terminals with default values.
 * TODO: split into two functions, one for servo and one for sensor.
 *       Make potentiometer values configurable over CAN as well.
 *
 * The TPL0102 potentiometer's end-to-end resistance is 100kOhm +/- 20%.
 * According to the schematic, VCC_Servo will have a maximum voltage output
 * of 10.80A, and a minimum of 2.47V - 3.11V depending on the actual maximum
 * resistance in the potentiometer (higher resistance => lower minimum).
 * The actual voltage will depend on the battery voltage, so make sure to
 * measure VCC_Servo with a multimeter before connecting a servo, to not burn
 * it. The corresponding range for VDD_Sensor is 5.10V maximum, and 3.22V-3.28V
 * minimum.
 *
 * The default value of POT_IVRA_DEFAULT is based on a servo with an operating
 * voltage of 6.0-8.4V.
 */
void InitPotentiometers(void) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t ivraWrite[2] = {POT_IVRA_ADDR, POT_IVRA_DEFAULT};
  uint8_t ivrbWrite[2] = {POT_IVRB_ADDR, POT_IVRB_DEFAULT};
  HAL_I2C_Master_Transmit(&peripherals->hi2c1, POT_ADDR, ivraWrite,
                          sizeof(ivraWrite), HAL_MAX_DELAY);
  HAL_I2C_Master_Transmit(&peripherals->hi2c1, POT_ADDR, ivrbWrite,
                          sizeof(ivrbWrite), HAL_MAX_DELAY);
}

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
