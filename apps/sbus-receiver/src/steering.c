#include "steering.h"

#define STEERING_CHANNEL 0
#define THROTTLE_CHANNEL 1
#define TRIM_CHANNEL 6

static const int16_t neutral_pulse = 1500;  // 1500 µs pulse
static const uint16_t switch_on_threshold = 1700;

static int16_t steering_trim = 0;
static int16_t throttle_trim = 0;

static int16_t calculate_trim(int16_t pulse);
static int16_t sbus_to_pwm(int16_t sbus_value);

steering_command_t sbus_packet_to_steering_command(
    const sbus_packet_t *sbus_packet) {
  int16_t steering =
      sbus_to_pwm((int16_t)sbus_packet->channels[STEERING_CHANNEL]);
  int16_t throttle =
      sbus_to_pwm((int16_t)sbus_packet->channels[THROTTLE_CHANNEL]);

  if (sbus_packet->channels[TRIM_CHANNEL] > switch_on_threshold) {
    steering_trim = calculate_trim(steering);
    throttle_trim = calculate_trim(throttle);
  }

  steering_command_t command = {
      .steering = (uint16_t)(steering - steering_trim),
      .steering_trim = steering_trim,
      .throttle = (uint16_t)(throttle - throttle_trim),
      .throttle_trim = throttle_trim,
  };

  return command;
}

steering_command_t neutral_steering_command(void) {
  // Keep trims, send neutral pulse
  steering_command_t command = {
      .steering = neutral_pulse,
      .steering_trim = steering_trim,
      .throttle = neutral_pulse,
      .throttle_trim = throttle_trim,
  };
  return command;
}

static int16_t calculate_trim(int16_t pulse) {
  // Don't interpret value as trim if it's above or below these values.
  const int16_t max_trim_pulse = 250;
  const int16_t min_trim_pulse = -250;

  int32_t pulse_offset = pulse - neutral_pulse;

  if (pulse_offset < min_trim_pulse) {
    return min_trim_pulse;
  }

  if (pulse_offset > max_trim_pulse) {
    return max_trim_pulse;
  }

  return (int16_t)pulse_offset;
}

// y = kx + m
// m = 1000 // min PWM pulse
// x = sbus_value
// k = 1000/2047 ~ 1/2
// y = x/2 + 1000
// pwm = sbus_value / 2 + 1000
static int16_t sbus_to_pwm(int16_t sbus_value) {
  const int16_t pwm = (int16_t)(sbus_value / 2 + 1000);
  return pwm;
}
