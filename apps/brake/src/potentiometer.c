#include "potentiometer.h"

#include <stdio.h>

#include "error.h"
#include "peripherals.h"

// Shift left to match STM32 specification
#define POTENTIOMETER_ADDRESS (0x50 << 1)

#define POTENTIOMETER_WRA_ADDRESS 0x00  // VCC_Servo potentiometer i2c address
#define POTENTIOMETER_WRB_ADDRESS 0x01  // VDD_Sensor potentiometer i2c address
#define POTENTIOMETER_ACR_ADDRESS 0x10  // Access control register

int init_potentiometers(void) {
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

int write_potentiometer_value(uint8_t terminal_address, uint8_t pot_value) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t write_cmd[2] = {terminal_address, pot_value};

  HAL_StatusTypeDef err =
      HAL_I2C_Master_Transmit(&peripherals->hi2c1, POTENTIOMETER_ADDRESS,
                              write_cmd, sizeof(write_cmd), HAL_MAX_DELAY);

  if (err != HAL_OK) {
    printf("Error: failed to set pot value: %lu\r\n",
           (unsigned long)HAL_I2C_GetError(&peripherals->hi2c1));
    return APP_NOT_OK;
  }

  return APP_OK;
}

int write_servo_potentiometer(uint8_t pot_value) {
  return write_potentiometer_value(POTENTIOMETER_WRA_ADDRESS, pot_value);
}

int write_sensor_potentiometer(uint8_t pot_value) {
  return write_potentiometer_value(POTENTIOMETER_WRB_ADDRESS, pot_value);
}
