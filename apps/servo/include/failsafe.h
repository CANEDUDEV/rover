#ifndef FAILSAFE_H
#define FAILSAFE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FAILSAFE_DEFAULT_TIMEOUT_MS 100

void failsafe_init(void);
void failsafe_refresh(void);
void failsafe_on(void);
void failsafe_off(void);
void failsafe_set_timeout(uint16_t timeout_ms);
void failsafe_set_pulse(uint16_t pulse_mus);

#ifdef __cplusplus
}
#endif

#endif  // FAILSAFE_H
