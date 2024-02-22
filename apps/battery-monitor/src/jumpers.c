#include "jumpers.h"

#include "battery.h"
#include "potentiometer.h"

/*
 * X11 and X12 jumper configuration
 *
 * | X11 | X12 | Rout      |
 * |-----|-----|-----------|
 * | OFF | OFF | 10.2 kOhm |
 * | ON  | OFF | 5.1 kOhm  |
 * | OFF | ON  | 3.4 kOhm  |
 * | ON  | ON  | 2.55 kOhm |
 */
#define X11_OFF_X12_OFF_R_OUT 10200
#define X11_ON_X12_OFF_R_OUT 5100
#define X11_OFF_X12_ON_R_OUT 3400
#define X11_ON_X12_ON_R_OUT 2550

static current_measure_jumper_config_t current_measure_jumper_config;
static voltage_regulator_jumper_config_t voltage_regulator_jumper_config;

void set_current_measure_jumper_config(
    current_measure_jumper_config_t jumper_config) {
  switch (jumper_config) {
    case X11_OFF_X12_OFF:
    case X11_ON_X12_OFF:
    case X11_OFF_X12_ON:
    case X11_ON_X12_ON:
      current_measure_jumper_config = jumper_config;
  }
}

uint16_t get_current_measure_jumper_r_out(void) {
  switch (current_measure_jumper_config) {
    case X11_ON_X12_OFF:
      return X11_ON_X12_OFF_R_OUT;

    case X11_OFF_X12_ON:
      return X11_OFF_X12_ON_R_OUT;

    case X11_ON_X12_ON:
      return X11_ON_X12_ON_R_OUT;

    case X11_OFF_X12_OFF:
    default:
      return X11_OFF_X12_OFF_R_OUT;
  }
}

void update_voltage_regulator_jumper_state(void) {
  battery_state_t *battery_state = get_battery_state();
  uint8_t pot_val = get_potentiometer_value();

  // There is overlap between the voltage ranges around 6-6.5V. Figure out which
  // jumper it is based on the potentiometer value in that case.
  const uint16_t max_6v_voltage = 6500;
  const uint16_t min_16v_voltage = 6000;
  const uint8_t high_voltage_pot_threshold = 200;

  // If voltage is above 6.5V, or between 6-6.5V with a low potentiometer
  // value, set to 6-16V range
  if (battery_state->reg_out_voltage > max_6v_voltage ||
      (battery_state->reg_out_voltage >= min_16v_voltage &&
       pot_val < high_voltage_pot_threshold)) {
    voltage_regulator_jumper_config = VOUT_6_TO_16V;

  } else {
    voltage_regulator_jumper_config = VOUT_0_TO_6V;
  }
}

voltage_regulator_jumper_config_t get_voltage_regulator_jumper_state(void) {
  return voltage_regulator_jumper_config;
}
