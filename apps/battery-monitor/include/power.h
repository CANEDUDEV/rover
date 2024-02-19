#ifndef POWER_H
#define POWER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  POWER_OFF,
  POWER_ON,
} power_state_t;

power_state_t get_vbat_power_state(void);
power_state_t get_reg_vout_power_state(void);

void set_vbat_power_on(void);
void set_reg_vout_power_on(void);
void set_vbat_power_off(void);
void set_reg_vout_power_off(void);

#ifdef __cplusplus
}
#endif

#endif  // POWER_H
