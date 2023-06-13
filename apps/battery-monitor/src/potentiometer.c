#include "potentiometer.h"

void configure_potentiometer(I2C_HandleTypeDef *hi2c, uint8_t pot_value) {
  uint8_t ivra_write[2] = {POTENTIOMETER_IVRA_ADDRESS, pot_value};
  HAL_I2C_Master_Transmit(hi2c, POTENTIOMETER_ADDRESS, ivra_write,
                          sizeof(ivra_write), HAL_MAX_DELAY);
}
