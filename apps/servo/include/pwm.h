#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PWM_NEUTRAL_PULSE_MUS 1500
#define PWM_MAX_FREQUENCY_HZ 333
#define PWM_SUBTRIM_MAX_PULSE 500
#define PWM_SUBTRIM_MIN_PULSE (-500)

void pwm_init(void);
void pwm_set_pulse(uint32_t pulse_mus);
void pwm_set_subtrim_pulse(int16_t pulse_mus);
void pwm_set_frequency(uint16_t frequency_hz);

#ifdef __cplusplus
}
#endif

#endif  // PWM_H
