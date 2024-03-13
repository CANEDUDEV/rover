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
  int16_t position;  // Reported position
  bool reverse;      // Reverse steering direction
} servo_t;

servo_t* get_servo_state(void);
void servo_init(void);
void update_servo_state(adc_reading_t* adc_reading);

#ifdef __cplusplus
}
#endif

#endif  // SERVO_H
