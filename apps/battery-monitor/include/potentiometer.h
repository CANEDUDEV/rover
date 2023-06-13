#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

#define POTENTIOMETER_ADDRESS \
  (0x53 << 1)  // Shift left to match STM32 specification
#define POTENTIOMETER_IVRA_ADDRESS 0x0
#define POTENTIOMETER_IVRA_DEFAULT 40  // Default potentiometer value

void configure_potentiometer(I2C_HandleTypeDef *hi2c, uint8_t pot_value);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
