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

// Envelope IDs
// Control messages
#define ROVER_STEERING_ENVELOPE 0x100
#define ROVER_THROTTLE_ENVELOPE 0x101

// Report messages
#define ROVER_BATTERY_CELL_VOLTAGES_ENVELOPE 0x200
#define ROVER_BATTERY_REGULATED_OUTPUT_ENVELOPE 0x201
#define ROVER_BATTERY_BATTERY_OUTPUT_ENVELOPE 0x202
#define ROVER_SERVO_VOLTAGE_ENVELOPE 0x203
#define ROVER_SERVO_CURRENT_ENVELOPE 0x204
#define ROVER_BATTERY_VOLTAGE_ENVELOPE 0x205
#define ROVER_SERVO_POSITION_ENVELOPE 0x206

// Settings messages
#define ROVER_BATTERY_JUMPER_CONFIG_ENVELOPE 0x300
#define ROVER_BATTERY_REG_OUT_VOLTAGE_ENVELOPE 0x301
#define ROVER_BATTERY_OUTPUT_ON_OFF_ENVELOPE 0x302
#define ROVER_BATTERY_REPORT_FREQUENCY_ENVELOPE 0x303
#define ROVER_BATTERY_LOW_VOLTAGE_CUTOFF_ENVELOPE 0x304
#define ROVER_SERVO_SET_VOLTAGE_ENVELOPE 0x305
#define ROVER_SERVO_PWM_CONFIG_ENVELOPE 0x306
#define ROVER_SERVO_REPORT_FREQUENCY_ENVELOPE 0x307
#define ROVER_MOTOR_PWM_CONFIG_ENVELOPE 0x308
#define ROVER_SERVO_REVERSE_ENVELOPE 0x309
#define ROVER_MOTOR_REVERSE_ENVELOPE 0x30A
#define ROVER_SERVO_FAILSAFE_ENVELOPE 0x30B
#define ROVER_MOTOR_FAILSAFE_ENVELOPE 0x30C
#define ROVER_SERVO_SET_SUBTRIM_ENVELOPE 0x30D
#define ROVER_MOTOR_SET_SUBTRIM_ENVELOPE 0x30E
#define ROVER_BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD_ENVELOPE 0x30F
#define ROVER_BATTERY_REG_OUT_OVERCURRENT_THRESHOLD_ENVELOPE 0x310

void init_king_task(uint32_t priority);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_H */
