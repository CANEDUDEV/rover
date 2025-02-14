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

  TIM_HandleTypeDef htim1;
  DMA_HandleTypeDef hdma_tim1_ch1;
  DMA_HandleTypeDef hdma_tim1_ch2;
  DMA_HandleTypeDef hdma_tim1_ch3;
  DMA_HandleTypeDef hdma_tim1_ch4;

  UART_HandleTypeDef huart2;
} peripherals_t;

peripherals_t *get_peripherals(void);

void peripherals_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
