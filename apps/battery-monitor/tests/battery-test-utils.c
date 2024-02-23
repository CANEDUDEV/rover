#include "battery-test-utils.h"

#include "led.h"
#include "power.h"

// Testing
#include "fff.h"

DEFINE_FFF_GLOBALS

// NOLINTBEGIN
DEFINE_FAKE_VOID_FUNC(set_vbat_power_on)
DEFINE_FAKE_VOID_FUNC(set_vbat_power_off)
DEFINE_FAKE_VOID_FUNC(set_reg_out_power_on)
DEFINE_FAKE_VOID_FUNC(set_reg_out_power_off)
DEFINE_FAKE_VOID_FUNC(led_init)
DEFINE_FAKE_VOID_FUNC(led_signal_fault)
DEFINE_FAKE_VOID_FUNC(led_stop_signal_fault)
DEFINE_FAKE_VOID_FUNC(set_led_color, led_t, led_color_t)
DEFINE_FAKE_VALUE_FUNC(int, read_potentiometer_value, uint8_t *)
DEFINE_FAKE_VALUE_FUNC(int, write_potentiometer_value, uint8_t)
DEFINE_FAKE_VALUE_FUNC(power_state_t, get_vbat_power_state)
DEFINE_FAKE_VALUE_FUNC(power_state_t, get_reg_out_power_state)
// NOLINTEND

void reset_fakes(void) {
  FFF_FAKES_LIST(RESET_FAKE);
  FFF_RESET_HISTORY();
}

bool is_acceptable_measurement(measurement_t *measurement) {
  int32_t min_accepted_measurement =
      measurement->expected_value - measurement->accepted_error;
  int32_t max_accepted_measurement =
      measurement->expected_value + measurement->accepted_error;

  return (measurement->actual_value >= min_accepted_measurement &&
          measurement->actual_value <= max_accepted_measurement);
}
