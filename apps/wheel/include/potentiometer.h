#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define POTENTIOMETER_SERVO_DEFAULT 185   // Gives 3.3V
#define POTENTIOMETER_SENSOR_DEFAULT 185  // Gives 3.3V

int init_potentiometers(void);
int write_servo_potentiometer(uint8_t pot_value);
int write_sensor_potentiometer(uint8_t pot_value);
int read_servo_potentiometer(uint8_t *pot_value);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
