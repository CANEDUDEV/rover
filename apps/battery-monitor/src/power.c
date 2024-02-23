#include "power.h"

#include "ports.h"
#include "stm32f3xx_hal_gpio.h"

power_state_t get_vbat_power_state(void) {
  GPIO_PinState pin = HAL_GPIO_ReadPin(nPOWER_OFF_GPIO_PORT, nPOWER_OFF_PIN);

  if (pin == GPIO_PIN_SET) {
    return POWER_ON;
  }
  return POWER_OFF;
}

power_state_t get_reg_out_power_state(void) {
  GPIO_PinState pin =
      HAL_GPIO_ReadPin(REG_POWER_ON_GPIO_PORT, REG_POWER_ON_PIN);

  if (pin == GPIO_PIN_SET) {
    return POWER_ON;
  }
  return POWER_OFF;
}

void set_vbat_power_on(void) {
  HAL_GPIO_WritePin(nPOWER_OFF_GPIO_PORT, nPOWER_OFF_PIN, GPIO_PIN_SET);
}

void set_reg_out_power_on(void) {
  HAL_GPIO_WritePin(REG_POWER_ON_GPIO_PORT, REG_POWER_ON_PIN, GPIO_PIN_SET);
}

void set_vbat_power_off(void) {
  HAL_GPIO_WritePin(nPOWER_OFF_GPIO_PORT, nPOWER_OFF_PIN, GPIO_PIN_RESET);
}

void set_reg_out_power_off(void) {
  HAL_GPIO_WritePin(REG_POWER_ON_GPIO_PORT, REG_POWER_ON_PIN, GPIO_PIN_RESET);
}
