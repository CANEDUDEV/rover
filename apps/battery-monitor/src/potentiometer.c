#include "potentiometer.h"

#include <stdio.h>

#include "error.h"
#include "peripherals.h"

// Shift left to match STM32 specification
#define POTENTIOMETER_ADDRESS (0x53 << 1)

// Potentiometer terminal A voltatile i2c address
#define POTENTIOMETER_WRA_ADDRESS 0x00

// Access control register
#define POTENTIOMETER_ACR_ADDRESS 0x10

int init_potentiometer(void) {
  peripherals_t *peripherals = get_peripherals();
  // Volatile RW enable, Shutdown disable
  const uint8_t acr_contents = 0xC0;
  uint8_t acr_write[2] = {POTENTIOMETER_ACR_ADDRESS, acr_contents};
  HAL_StatusTypeDef err =
      HAL_I2C_Master_Transmit(&peripherals->hi2c1, POTENTIOMETER_ADDRESS,
                              acr_write, sizeof(acr_write), HAL_MAX_DELAY);
  if (err != HAL_OK) {
    printf("Error: failed to set up potentiometer: %lu\r\n",
           (unsigned long)HAL_I2C_GetError(&peripherals->hi2c1));
    return APP_NOT_OK;
  }

  return APP_OK;
}

int write_potentiometer_value(uint8_t pot_value) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t wra_write_cmd[2] = {POTENTIOMETER_WRA_ADDRESS, pot_value};

  HAL_StatusTypeDef err = HAL_I2C_Master_Transmit(
      &peripherals->hi2c1, POTENTIOMETER_ADDRESS, wra_write_cmd,
      sizeof(wra_write_cmd), HAL_MAX_DELAY);

  if (err != HAL_OK) {
    printf("Error: failed to set pot value: %lu\r\n",
           (unsigned long)HAL_I2C_GetError(&peripherals->hi2c1));
    return APP_NOT_OK;
  }

  return APP_OK;
}

int read_potentiometer_value(uint8_t *pot_value) {
  if (pot_value == NULL) {
    return APP_NOT_OK;
  }

  peripherals_t *peripherals = get_peripherals();
  uint8_t wra_read_cmd = POTENTIOMETER_WRA_ADDRESS;

  HAL_StatusTypeDef err = HAL_I2C_Master_Transmit(
      &peripherals->hi2c1, POTENTIOMETER_ADDRESS, &wra_read_cmd,
      sizeof(wra_read_cmd), HAL_MAX_DELAY);

  if (err != HAL_OK) {
    printf("Error: failed to transmit read cmd: %lu\r\n",
           (unsigned long)HAL_I2C_GetError(&peripherals->hi2c1));
    return APP_NOT_OK;
  }

  err = HAL_I2C_Master_Receive(&peripherals->hi2c1, POTENTIOMETER_ADDRESS,
                               pot_value, sizeof(uint8_t), HAL_MAX_DELAY);
  if (err != HAL_OK) {
    printf("Error: failed to read pot value: %lu\r\n",
           (unsigned long)HAL_I2C_GetError(&peripherals->hi2c1));
    return APP_NOT_OK;
  }

  return APP_OK;
}
