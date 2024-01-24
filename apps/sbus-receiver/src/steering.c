#include "steering.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ck-data.h"
#include "error.h"
#include "lfs-wrapper.h"
#include "mayor.h"

// Radio channels in use. Most radios have 7 or more channels.
#define STEERING_CHANNEL 0
#define THROTTLE_CHANNEL 1
#define STEERING_SWITCH_CHANNEL 4
#define CALIBRATION_SWITCH_CHANNEL 6

#define SBUS_MID 1023
#define SBUS_MAX_OFFSET 400

#define PWM_MID_PULSE 1500

static float sbus_to_steering_angle(float sbus_value);
static int16_t sbus_to_throttle(float sbus_value);
static void update_calibration(const sbus_message_t *sbus_packet);
static void send_subtrim_commands(void);
static bool switch_is_active(uint16_t switch_value);
static void save_calibration_data(void);
static uint16_t get_press_time_in_seconds(uint16_t switch_message_count);
static int16_t sbus_to_pwm(float value, float range);

static int16_t steering_subtrim;
static int16_t throttle_subtrim;

typedef struct {
  float steering_min;
  float steering_max;
  float throttle_min;
  float throttle_max;
} calibration_values_t;

static const calibration_values_t default_calibration_values = {
    .steering_min = SBUS_MID - SBUS_MAX_OFFSET,
    .steering_max = SBUS_MID + SBUS_MAX_OFFSET,
    .throttle_min = SBUS_MID - SBUS_MAX_OFFSET,
    .throttle_max = SBUS_MID + SBUS_MAX_OFFSET,
};

static calibration_values_t calibration_values;

static const char *calibration_file = "/calibration";

void init_steering(void) {
  lfs_init();

  calibration_values = default_calibration_values;

  calibration_values_t read_calibration_values;
  file_t file = {
      .name = calibration_file,
      .data = &read_calibration_values,
      .size = sizeof(calibration_values_t),
  };

  if (read_file(&file) != APP_OK) {
    printf("Radio not calibrated, proceeding with default values.\r\n");
  } else {
    calibration_values = read_calibration_values;
  }
}

steering_command_t sbus_message_to_steering_command(
    const sbus_message_t *sbus_packet) {
  int16_t steering = (int16_t)sbus_packet->channels[STEERING_CHANNEL];
  int16_t throttle = (int16_t)sbus_packet->channels[THROTTLE_CHANNEL];
  uint16_t steering_switch = sbus_packet->channels[STEERING_SWITCH_CHANNEL];

  steering_command_t command = {
      .steering_is_on = switch_is_active(steering_switch),
      .steering_angle = sbus_to_steering_angle(steering),
      .throttle = sbus_to_throttle(throttle),
  };

  update_calibration(sbus_packet);

  return command;
}

steering_command_t neutral_steering_command(void) {
  const steering_command_t command = {
      .steering_is_on = true,
      .steering_angle = 0,
      .throttle = 1500,
  };
  return command;
}

// Steering range varies between -45 and 45 deg.
static float sbus_to_steering_angle(float sbus_value) {
  if (sbus_value < calibration_values.steering_min) {
    calibration_values.steering_min = sbus_value;
  }
  if (sbus_value > calibration_values.steering_max) {
    calibration_values.steering_max = sbus_value;
  }

  float sbus_range =
      calibration_values.steering_max + calibration_values.steering_min;

  // Always update subtrim and assume it will be correct when user triggers
  // calibration save.
  steering_subtrim =
      (int16_t)(sbus_to_pwm(sbus_value, sbus_range) - PWM_MID_PULSE);

  const float angle = 90 * sbus_value / sbus_range - 45;
  return angle;
}

// Throttle pulse width ranges between 1000 and 2000.
// TODO: Switch to a throttle protocol instead of pulse width.
static int16_t sbus_to_throttle(float sbus_value) {
  if (sbus_value < calibration_values.throttle_min) {
    calibration_values.throttle_min = sbus_value;
  }
  if (sbus_value > calibration_values.throttle_max) {
    calibration_values.throttle_max = sbus_value;
  }

  float sbus_range =
      calibration_values.throttle_max + calibration_values.throttle_min;

  int16_t pwm = sbus_to_pwm(sbus_value, sbus_range);

  // Always update subtrim and assume it will be correct when user triggers
  // calibration save.
  throttle_subtrim = (int16_t)(pwm - PWM_MID_PULSE);

  return pwm;
}

static void update_calibration(const sbus_message_t *sbus_packet) {
  static uint16_t switch_pressed_count = 0;

  if (switch_is_active(sbus_packet->channels[CALIBRATION_SWITCH_CHANNEL])) {
    switch_pressed_count++;
    return;
  }

  // Switch released
  const uint16_t save_calibration_time = 1;
  const uint16_t reset_calibration_time = 5;
  uint16_t switch_pressed_time =
      get_press_time_in_seconds(switch_pressed_count);

  if (switch_pressed_time >= reset_calibration_time) {
    printf("Resetting calibration data...\r\n");
    calibration_values = default_calibration_values;
    steering_subtrim = 0;
    throttle_subtrim = 0;
  }

  if (switch_pressed_time >= save_calibration_time) {
    printf("Saving calibration data...\r\n");
    send_subtrim_commands();
    save_calibration_data();
  }

  switch_pressed_count = 0;
}

static void send_subtrim_commands(void) {
  ck_data_t *ck_data = get_ck_data();

  memcpy(ck_data->steering_subtrim_page->lines, &steering_subtrim,
         sizeof(steering_subtrim));
  memcpy(ck_data->throttle_subtrim_page->lines, &throttle_subtrim,
         sizeof(throttle_subtrim));

  if (ck_send_document(ck_data->steering_subtrim_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->throttle_subtrim_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
}

static bool switch_is_active(uint16_t switch_value) {
  const uint16_t switch_active_min_sbus_value = 1600;
  return switch_value > switch_active_min_sbus_value;
}

static void save_calibration_data(void) {
  file_t file = {
      .name = calibration_file,
      .data = &calibration_values,
      .size = sizeof(calibration_values_t),
  };

  if (write_file_async(&file) != APP_OK) {
    printf("Error: couldn't write calibration data to flash.\r\n");
  }
}

static uint16_t get_press_time_in_seconds(uint16_t switch_message_count) {
  const uint16_t switch_messages_per_second = 100;
  return switch_message_count / switch_messages_per_second;
}

static int16_t sbus_to_pwm(float value, float range) {
  const float pwm = 1000 * value / range + 1000;
  return (int16_t)roundf(pwm);
}
