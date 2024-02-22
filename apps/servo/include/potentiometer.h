#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Default potentiometer values
 *
 * The default value of POT_IVRA_DEFAULT is based on a servo with an operating
 * voltage of 6.0-8.4V.
 */
#define POTENTIOMETER_SERVO_DEFAULT 32  // Gives ~7.4V, standard servo voltage.
#define POTENTIOMETER_SENSOR_DEFAULT 180  // Gives 3.3V

int init_potentiometers(void);
int write_servo_potentiometer(uint8_t pot_value);
int write_sensor_potentiometer(uint8_t pot_value);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
