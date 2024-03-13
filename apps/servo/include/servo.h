#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"

#define DEFAULT_SERVO_VOLTAGE_MV 7400

typedef struct {
  uint16_t target_voltage;
  uint16_t voltage;
  uint16_t current;
  float steering_angle;
  uint16_t steering_pulse;
  int16_t position;  // Reported position
  bool reverse;      // Reverse steering direction
} servo_state_t;

typedef enum {
  STEERING_MODE_PULSE = 0,
  STEERING_MODE_ANGLE = 1,
} steering_mode_t;

servo_state_t* get_servo_state(void);
void servo_init(void);
void update_servo_state(adc_reading_t* adc_reading);
int update_servo_pulse(uint16_t pulse);
int update_servo_angle(float angle);

#ifdef __cplusplus
}
#endif

#endif  // SERVO_H
