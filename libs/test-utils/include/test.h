#ifndef TEST_H
#define TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

typedef enum {
  TEST_PASS = 0,
  TEST_FAIL = 1,

  // GNU-specific exit codes used by meson
  TEST_SKIP = 77,
  TEST_SETUP_FAIL = 99,
} test_err_t;

#define ASSERT(condition, message, ...)                                       \
  do {                                                                        \
    if (!(condition)) {                                                       \
      printf("Assertion failed in func %s, line %d: " message "\n", __func__, \
             __LINE__, ##__VA_ARGS__);                                        \
      exit(TEST_FAIL);                                                        \
    }                                                                         \
  } while (0)

typedef struct {
  float actual_value;
  float expected_value;
  float accepted_error;
} measurement_t;

bool is_acceptable_measurement(measurement_t *measurement);

#ifdef __cplusplus
}
#endif

#endif /* TEST_H */
