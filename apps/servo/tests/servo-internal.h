#ifndef SERVO_INTERNAL_H
#define SERVO_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

void update_voltage_controller(void);
bool is_servo_voltage_stable(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_INTERNAL_H */
