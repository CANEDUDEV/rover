#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// CK
#include "ck-types.h"

void init_wheel_speed_task(uint8_t priority);
int process_set_wheel_parameters_letter(const ck_letter_t *letter);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: WHEEL_SPEED_H */
