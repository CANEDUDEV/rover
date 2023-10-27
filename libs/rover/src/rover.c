#include "rover.h"

#include <stdbool.h>

#define ASSIGNMENT_COUNT 24

static rover_assignment_t assignments[ASSIGNMENT_COUNT];
static rover_kingdom_t kingdom = {
    .assignments = assignments,
    .assignment_count = ASSIGNMENT_COUNT,
};

// NOLINTBEGIN(*-magic-numbers)
void init_assignments(void) {
  assignments[0].city = ROVER_SERVO_ID;
  assignments[0].envelope = ROVER_STEERING_ENVELOPE;
  assignments[0].folder = 9;

  assignments[1].city = ROVER_SERVO_ID;
  assignments[1].envelope = ROVER_STEERING_TRIM_ENVELOPE;
  assignments[1].folder = 10;

  assignments[2].city = ROVER_MOTOR_ID;
  assignments[2].envelope = ROVER_THROTTLE_ENVELOPE;
  assignments[2].folder = 9;

  assignments[3].city = ROVER_MOTOR_ID;
  assignments[3].envelope = ROVER_THROTTLE_TRIM_ENVELOPE;
  assignments[3].folder = 10;

  assignments[4].city = ROVER_SBUS_RECEIVER_ID;
  assignments[4].envelope = ROVER_STEERING_ENVELOPE;
  assignments[4].folder = 2;

  assignments[5].city = ROVER_SBUS_RECEIVER_ID;
  assignments[5].envelope = ROVER_STEERING_TRIM_ENVELOPE;
  assignments[5].folder = 3;

  assignments[6].city = ROVER_SBUS_RECEIVER_ID;
  assignments[6].envelope = ROVER_THROTTLE_ENVELOPE;
  assignments[6].folder = 4;

  assignments[7].city = ROVER_SBUS_RECEIVER_ID;
  assignments[7].envelope = ROVER_THROTTLE_TRIM_ENVELOPE;
  assignments[7].folder = 5;

  assignments[8].city = ROVER_BATTERY_MONITOR_ID;
  assignments[8].envelope = ROVER_BATTERY_CELL_VOLTAGES_ENVELOPE;
  assignments[8].folder = 2;

  assignments[9].city = ROVER_BATTERY_MONITOR_ID;
  assignments[9].envelope = ROVER_BATTERY_REG_OUT_CURRENT_ENVELOPE;
  assignments[9].folder = 3;

  assignments[10].city = ROVER_BATTERY_MONITOR_ID;
  assignments[10].envelope = ROVER_BATTERY_VBAT_OUT_CURRENT_ENVELOPE;
  assignments[10].folder = 4;

  assignments[11].city = ROVER_SERVO_ID;
  assignments[11].envelope = ROVER_SERVO_VOLTAGE_ENVELOPE;
  assignments[11].folder = 5;

  assignments[12].city = ROVER_SERVO_ID;
  assignments[12].envelope = ROVER_SERVO_CURRENT_ENVELOPE;
  assignments[12].folder = 3;

  assignments[13].city = ROVER_BATTERY_MONITOR_ID;
  assignments[13].envelope = ROVER_BATTERY_JUMPER_AND_FUSE_CONF_ENVELOPE;
  assignments[13].folder = 5;

  assignments[14].city = ROVER_BATTERY_MONITOR_ID;
  assignments[14].envelope = ROVER_BATTERY_REG_OUT_VOLTAGE_ENVELOPE;
  assignments[14].folder = 6;

  assignments[15].city = ROVER_BATTERY_MONITOR_ID;
  assignments[15].envelope = ROVER_BATTERY_OUTPUT_ON_OFF_ENVELOPE;
  assignments[15].folder = 7;

  assignments[16].city = ROVER_BATTERY_MONITOR_ID;
  assignments[16].envelope = ROVER_BATTERY_REPORT_FREQUENCY_ENVELOPE;
  assignments[16].folder = 8;

  assignments[17].city = ROVER_BATTERY_MONITOR_ID;
  assignments[17].envelope = ROVER_BATTERY_LOW_VOLTAGE_CUTOFF_ENVELOPE;
  assignments[17].folder = 9;

  assignments[18].city = ROVER_SERVO_ID;
  assignments[18].envelope = ROVER_SERVO_SET_VOLTAGE_ENVELOPE;
  assignments[18].folder = 7;

  assignments[19].city = ROVER_SERVO_ID;
  assignments[19].envelope = ROVER_SERVO_PWM_CONFIG_ENVELOPE;
  assignments[19].folder = 8;

  assignments[20].city = ROVER_SERVO_ID;
  assignments[20].envelope = ROVER_SERVO_REPORT_FREQUENCY_ENVELOPE;
  assignments[20].folder = 11;

  assignments[21].city = ROVER_MOTOR_ID;
  assignments[21].envelope = ROVER_MOTOR_PWM_CONFIG_ENVELOPE;
  assignments[21].folder = 8;

  assignments[22].city = ROVER_SERVO_ID;
  assignments[22].envelope = ROVER_SERVO_REVERSE_ENVELOPE;
  assignments[22].folder = 12;

  assignments[23].city = ROVER_MOTOR_ID;
  assignments[23].envelope = ROVER_MOTOR_REVERSE_ENVELOPE;
  assignments[23].folder = 12;
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
