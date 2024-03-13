#ifndef BATTERY_TEST_UTILS_H
#define BATTERY_TEST_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "led.h"
#include "power.h"

// Testing
#include "fff.h"

DECLARE_FAKE_VOID_FUNC(set_vbat_power_on)
DECLARE_FAKE_VOID_FUNC(set_vbat_power_off)
DECLARE_FAKE_VOID_FUNC(set_reg_out_power_on)
DECLARE_FAKE_VOID_FUNC(set_reg_out_power_off)
DECLARE_FAKE_VOID_FUNC(led_init)
DECLARE_FAKE_VOID_FUNC(led_signal_fault)
DECLARE_FAKE_VOID_FUNC(led_stop_signal_fault)
DECLARE_FAKE_VOID_FUNC(set_led_color, led_t, led_color_t)
DECLARE_FAKE_VALUE_FUNC(int, read_potentiometer_value, uint8_t *)
DECLARE_FAKE_VALUE_FUNC(int, write_potentiometer_value, uint8_t)
DECLARE_FAKE_VALUE_FUNC(power_state_t, get_vbat_power_state)
DECLARE_FAKE_VALUE_FUNC(power_state_t, get_reg_out_power_state)

#define FFF_FAKES_LIST(FAKE)      \
  FAKE(set_vbat_power_on)         \
  FAKE(set_vbat_power_off)        \
  FAKE(set_reg_out_power_on)      \
  FAKE(set_reg_out_power_off)     \
  FAKE(led_init)                  \
  FAKE(led_signal_fault)          \
  FAKE(led_stop_signal_fault)     \
  FAKE(set_led_color)             \
  FAKE(read_potentiometer_value)  \
  FAKE(write_potentiometer_value) \
  FAKE(get_vbat_power_state)      \
  FAKE(get_reg_out_power_state)

void reset_fakes(void);

#ifdef __cplusplus
}
#endif

#endif  // BATTERY_TEST_UTILS_H
