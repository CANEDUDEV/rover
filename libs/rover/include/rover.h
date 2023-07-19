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

#define ROVER_BASE_NUMBER 0x400

// Envelope IDs
// Control messages
#define ROVER_STEERING_ENVELOPE 0x100
#define ROVER_THROTTLE_ENVELOPE 0x101
#define ROVER_STEERING_TRIM_ENVELOPE 0x102
#define ROVER_THROTTLE_TRIM_ENVELOPE 0x103

// Report messages
#define ROVER_BATTERY_CELL_VOLTAGES_ENVELOPE 0x200
#define ROVER_BATTERY_REG_OUT_CURRENT_ENVELOPE 0x201
#define ROVER_BATTERY_VBAT_OUT_CURRENT_ENVELOPE 0x202

#define ROVER_SERVO_VOLTAGE_ENVELOPE 0x203
#define ROVER_SERVO_CURRENT_ENVELOPE 0x204

// Settings messages
#define ROVER_BATTERY_JUMPER_AND_FUSE_CONF_ENVELOPE 0x300
#define ROVER_BATTERY_REG_OUT_VOLTAGE_ENVELOPE 0x301
#define ROVER_BATTERY_OUTPUT_ON_OFF_ENVELOPE 0x302
#define ROVER_BATTERY_REPORT_FREQUENCY_ENVELOPE 0x303
#define ROVER_BATTERY_LOW_VOLTAGE_CUTOFF_ENVELOPE 0x304

#define ROVER_SERVO_SET_VOLTAGE_ENVELOPE 0x305
#define ROVER_SERVO_PWM_CONFIG_ENVELOPE 0x306
#define ROVER_SERVO_REPORT_FREQUENCY_ENVELOPE 0x307

#define ROVER_MOTOR_PWM_CONFIG_ENVELOPE 0x308

typedef struct {
  uint8_t city;
  uint16_t envelope;
  uint8_t folder;
} rover_assignment_t;

typedef struct {
  rover_assignment_t *assignments;
  uint8_t assignment_count;
} rover_kingdom_t;

rover_kingdom_t *get_rover_kingdom(void);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_H */
