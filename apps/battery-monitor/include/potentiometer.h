#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

#define POTENTIOMETER_IVRA_DEFAULT 40  // Default potentiometer value

void configure_potentiometer(uint8_t pot_value);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
