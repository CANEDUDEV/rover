#ifndef ROVER_H
#define ROVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// City IDs
#define ROVER_BATTERY_MONITOR_ID 1
#define ROVER_SERVO_ID 2
#define ROVER_MOTOR_ID 3
#define ROVER_SBUS_RECEIVER_ID 4

// Kingdom base number
#define ROVER_BASE_NUMBER 0x400

void init_king_task(uint32_t priority);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_H */
