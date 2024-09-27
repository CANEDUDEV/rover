#ifndef REPORT_H
#define REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ck-types.h"

void init_report_task(uint8_t priority);
int process_report_freq_letter(const ck_letter_t *letter);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: REPORT_H */
