#ifndef BATTERY_TEST_UTILS_H
#define BATTERY_TEST_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int32_t actual_value;
  int32_t expected_value;
  int32_t accepted_error;
} measurement_t;

bool is_acceptable_measurement(measurement_t *measurement);

#ifdef __cplusplus
}
#endif

#endif  // BATTERY_TEST_UTILS_H
