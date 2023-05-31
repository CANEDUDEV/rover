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
  ADC_HandleTypeDef hadc1;
  ADC_HandleTypeDef hadc2;
  DMA_HandleTypeDef hdma_adc1;
  DMA_HandleTypeDef hdma_adc2;
  CAN_HandleTypeDef hcan;
  I2C_HandleTypeDef hi2c1;
  SPI_HandleTypeDef hspi1;
  UART_HandleTypeDef huart1;
} peripherals_t;

peripherals_t *get_peripherals(void);
void adc1_init(void);
void adc2_init(void);
void can_init(void);
void i2c1_init(void);
void spi1_init(void);
void uart1_init(void);
void dma_init(void);
void gpio_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
