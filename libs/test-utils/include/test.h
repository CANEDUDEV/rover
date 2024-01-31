#ifndef TEST_H
#define TEST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TEST_PASS = 0,
  TEST_FAIL = 1,

  // GNU-specific exit codes used by meson
  TEST_SKIP = 77,
  TEST_SETUP_FAIL = 99,
} test_err_t;

#ifdef __cplusplus
}
#endif

#endif /* TEST_H */
