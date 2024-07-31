/*******************************************************************************
 * @file peripherals.h
 *
 * Provides peripheral initialization functions.
 ******************************************************************************/
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common-peripherals.h"
#include "stm32f3xx_hal.h"

typedef struct {
  // Provided by CPU board
  common_peripherals_t *common_peripherals;

  ADC_HandleTypeDef hadc1;
  DMA_HandleTypeDef hdma_adc1;

} peripherals_t;

peripherals_t *get_peripherals(void);

void peripherals_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
