#include "steering.h"

#include <math.h>

#define STEERING_CHANNEL 0
#define THROTTLE_CHANNEL 1

#define SBUS_MID 1023
#define SBUS_MAX_OFFSET 800

static int16_t sbus_to_steering_angle(float sbus_value);
static int16_t sbus_to_throttle(float sbus_value);

steering_command_t sbus_packet_to_steering_command(
    const sbus_packet_t *sbus_packet) {
  int16_t steering = (int16_t)sbus_packet->channels[STEERING_CHANNEL];
  int16_t throttle = (int16_t)sbus_packet->channels[THROTTLE_CHANNEL];

  steering_command_t command = {
      .steering_angle = sbus_to_steering_angle(steering),
      .throttle = sbus_to_throttle(throttle),
  };

  return command;
}

steering_command_t neutral_steering_command(void) {
  const steering_command_t command = {
      .steering_angle = 0,
      .throttle = 1500,
  };
  return command;
}

// Steering range varies between -45 and 45 deg.
static int16_t sbus_to_steering_angle(float sbus_value) {
  static float sbus_min = SBUS_MID - SBUS_MAX_OFFSET;
  static float sbus_max = SBUS_MID + SBUS_MAX_OFFSET;

  if (sbus_value < sbus_min) {
    sbus_min = sbus_value;
  }
  if (sbus_value > sbus_max) {
    sbus_max = sbus_value;
  }

  float sbus_range = sbus_max + sbus_min;
  const float angle = 90 * sbus_value / sbus_range - 45;
  return (int16_t)roundf(angle);
}

// Throttle pulse width ranges between 1000 and 2000.
// TODO: Switch to a throttle protocol instead of pulse width.
static int16_t sbus_to_throttle(float sbus_value) {
  static float sbus_min = SBUS_MID - SBUS_MAX_OFFSET;
  static float sbus_max = SBUS_MID + SBUS_MAX_OFFSET;

  if (sbus_value < sbus_min) {
    sbus_min = sbus_value;
  }
  if (sbus_value > sbus_max) {
    sbus_max = sbus_value;
  }

  float sbus_range = sbus_max + sbus_min;
  const float pwm = 1000 * sbus_value / sbus_range + 1000;
  return (int16_t)roundf(pwm);
}
