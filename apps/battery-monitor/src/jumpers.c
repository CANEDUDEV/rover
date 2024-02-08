#include "jumpers.h"

static current_measure_jumper_config_t current_measure_jumper_config;

void set_current_measure_jumper_config(
    current_measure_jumper_config_t jumper_config) {
  switch (jumper_config) {
    case X11_OFF_X12_OFF:
    case X11_ON:
    case X12_ON:
    case X11_ON_X12_ON:
      current_measure_jumper_config = jumper_config;
  }
}

current_measure_jumper_config_t get_current_measure_jumper_config(void) {
  return current_measure_jumper_config;
}
