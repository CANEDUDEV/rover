#include "rover.h"

#include <stdbool.h>

#define ASSIGNMENT_COUNT 27

static rover_assignment_t assignments[ASSIGNMENT_COUNT];
static rover_kingdom_t kingdom = {
    .assignments = assignments,
    .assignment_count = ASSIGNMENT_COUNT,
};

// NOLINTBEGIN(*-magic-numbers)
void init_assignments(void) {
  int index = 0;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_STEERING_ENVELOPE;
  assignments[index].folder = 9;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_THROTTLE_ENVELOPE;
  assignments[index].folder = 9;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_STEERING_ENVELOPE;
  assignments[index].folder = 2;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_THROTTLE_ENVELOPE;
  assignments[index].folder = 3;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_CELL_VOLTAGES_ENVELOPE;
  assignments[index].folder = 2;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_REG_OUT_CURRENT_ENVELOPE;
  assignments[index].folder = 3;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_VBAT_OUT_CURRENT_ENVELOPE;
  assignments[index].folder = 4;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_VOLTAGE_ENVELOPE;
  assignments[index].folder = 5;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_CURRENT_ENVELOPE;
  assignments[index].folder = 3;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_JUMPER_AND_FUSE_CONF_ENVELOPE;
  assignments[index].folder = 5;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_REG_OUT_VOLTAGE_ENVELOPE;
  assignments[index].folder = 6;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_OUTPUT_ON_OFF_ENVELOPE;
  assignments[index].folder = 7;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_REPORT_FREQUENCY_ENVELOPE;
  assignments[index].folder = 8;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_LOW_VOLTAGE_CUTOFF_ENVELOPE;
  assignments[index].folder = 9;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_SET_VOLTAGE_ENVELOPE;
  assignments[index].folder = 7;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_PWM_CONFIG_ENVELOPE;
  assignments[index].folder = 8;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_REPORT_FREQUENCY_ENVELOPE;
  assignments[index].folder = 11;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_PWM_CONFIG_ENVELOPE;
  assignments[index].folder = 8;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_REVERSE_ENVELOPE;
  assignments[index].folder = 12;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_REVERSE_ENVELOPE;
  assignments[index].folder = 12;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_FAILSAFE_ENVELOPE;
  assignments[index].folder = 13;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_FAILSAFE_ENVELOPE;
  assignments[index].folder = 13;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_SERVO_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 4;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_MOTOR_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 5;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 10;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 10;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_POSITION_ENVELOPE;
  assignments[index].folder = 2;
  index++;
}
// NOLINTEND(*-magic-numbers)

rover_kingdom_t *get_rover_kingdom(void) {
  static bool init_complete = false;
  if (!init_complete) {
    init_assignments();
    init_complete = true;
  }

  return &kingdom;
}
