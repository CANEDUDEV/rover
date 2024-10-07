#ifndef ADC_INTERNAL_H
#define ADC_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void adc_update_vdda(uint16_t vrefint_cal, uint16_t vrefint);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: ADC_INTERNAL_H */
