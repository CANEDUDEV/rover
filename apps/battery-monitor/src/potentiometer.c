#include "potentiometer.h"

#include <stdio.h>

#include "error.h"
#include "peripherals.h"

// Shift left to match STM32 specification
#define POTENTIOMETER_ADDRESS (0x53 << 1)
// Potentiometer terminal A i2c address
#define POTENTIOMETER_IVRA_ADDRESS 0x0

static uint8_t last_pot_value;

int configure_potentiometer(uint8_t pot_value) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t ivra_write[2] = {POTENTIOMETER_IVRA_ADDRESS, pot_value};
  HAL_StatusTypeDef err =
      HAL_I2C_Master_Transmit(&peripherals->hi2c1, POTENTIOMETER_ADDRESS,
                              ivra_write, sizeof(ivra_write), HAL_MAX_DELAY);

  if (err != HAL_OK) {
    printf("Error: failed to set pot value: %d\r\n", err);
    return APP_NOT_OK;
  }

  last_pot_value = pot_value;
  return APP_OK;
}

uint8_t get_potentiometer_value(void) {
  return last_pot_value;
}
