#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

/* Default potentiometer values
 *
 * The default value of POT_IVRA_DEFAULT is based on a servo with an operating
 * voltage of 6.0-8.4V.
 */
#define POT_SERVO_DEFAULT 40
#define POT_SENSOR_DEFAULT 40

void configure_servo_potentiometer(uint8_t pot_value);
void configure_sensor_potentiometer(uint8_t pot_value);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
