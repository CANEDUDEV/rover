#include "steering.h"

#define STEERING_CHANNEL 0
#define THROTTLE_CHANNEL 1

int16_t sbus_to_pwm(int16_t sbus_value);

steering_command_t sbus_packet_to_steering_command(
    const sbus_packet_t *sbus_packet) {
  const int16_t neutral_pulse = 1500;  // 1500 µs pulse
  // Don't interpret value as trim if it's above or below these values.
  const int32_t max_trim_pulse = 250;
  const int32_t min_trim_pulse = -250;
  // Receiving this many messages in a row indicates there's been some trim.
  const uint16_t counter_threshold = 50;

  static int16_t steering_trim = 0;
  static int16_t throttle_trim = 0;

  static uint8_t steering_count = 0;
  static uint8_t throttle_count = 0;

  static int16_t previous_steering = 0;
  static int16_t previous_throttle = 0;

  int16_t steering = (int16_t)sbus_packet->channels[STEERING_CHANNEL];
  int16_t throttle = (int16_t)sbus_packet->channels[THROTTLE_CHANNEL];

  // Count how many same messages were received in a row
  if (steering == previous_steering) {
    steering_count++;
  } else {
    steering_count = 0;
  }
  if (throttle == previous_throttle) {
    throttle_count++;
  } else {
    throttle_count = 0;
  }

  previous_steering = steering;
  previous_throttle = throttle;

  if (steering_count >= counter_threshold) {
    int16_t trim = (int16_t)(sbus_to_pwm(steering) - neutral_pulse);
    if (trim < max_trim_pulse && trim > min_trim_pulse) {
      steering_trim = trim;
    }
  }

  if (throttle_count >= counter_threshold) {
    int16_t trim = (int16_t)(sbus_to_pwm(throttle) - neutral_pulse);
    if (trim < max_trim_pulse && trim > min_trim_pulse) {
      throttle_trim = trim;
    }
  }

  steering_command_t command = {
      .steering = (uint16_t)(sbus_to_pwm(steering) - steering_trim),
      .steering_trim = steering_trim,
      .throttle = (uint16_t)(sbus_to_pwm(throttle) - throttle_trim),
      .throttle_trim = throttle_trim,
  };

  return command;
}

// y = kx + m
// m = 1000 // min PWM pulse
// x = sbus_value
// k = 1000/2047 ~ 1/2
// y = x/2 + 1000
// pwm = sbus_value / 2 + 1000
int16_t sbus_to_pwm(int16_t sbus_value) {
  const int16_t pwm = (int16_t)(sbus_value / 2 + 1000);
  return pwm;
}