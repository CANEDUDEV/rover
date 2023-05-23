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

#include "stm32f3xx_hal.h"

typedef struct {
  CAN_HandleTypeDef hcan;
  SPI_HandleTypeDef hspi1;
  UART_HandleTypeDef huart1;
} peripherals_t;

peripherals_t *get_peripherals(void);
void can_init(void);
void spi1_init(void);
void uart1_init(void);
void gpio_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
