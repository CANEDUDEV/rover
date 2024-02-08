#ifndef JUMPERS_H
#define JUMPERS_H

#ifdef __cplusplus
extern "C" {
#endif

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
typedef enum {
  X11_OFF_X12_OFF = 10200,
  X11_ON = 5100,
  X12_ON = 3400,
  X11_ON_X12_ON = 2550,
} current_measure_jumper_config_t;

void set_current_measure_jumper_config(
    current_measure_jumper_config_t jumper_config);

current_measure_jumper_config_t get_current_measure_jumper_config(void);

#ifdef __cplusplus
}
#endif

#endif /* JUMPERS_H */
