#ifndef JUMPERS_H
#define JUMPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * X11 and X12 jumper configuration
 *
 * | X11 | X12 | Measurable current range |
 * |-----|-----|--------------------------|
 * | OFF | OFF | 0-33 A                   |
 * | ON  | OFF | 0-66 A                   |
 * | OFF | ON  | 0-99 A                   |
 * | ON  | ON  | 0-132 A                  |
 */
typedef enum {
  X11_OFF_X12_OFF,
  X11_ON_X12_OFF,
  X11_OFF_X12_ON,
  X11_ON_X12_ON,
} current_measure_jumper_config_t;

typedef enum {
  VOUT_0_TO_6V,
  VOUT_6_TO_16V,
} voltage_regulator_jumper_config_t;

void set_current_measure_jumper_config(
    current_measure_jumper_config_t jumper_config);

uint16_t get_current_measure_jumper_r_out(void);

void update_voltage_regulator_jumper_state(void);

voltage_regulator_jumper_config_t get_voltage_regulator_jumper_state(void);

#ifdef __cplusplus
}
#endif

#endif /* JUMPERS_H */
