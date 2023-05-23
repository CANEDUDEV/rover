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

// Some definitions for PWM
#define PWM_PSC_1MHZ (72 - 1)
#define PWM_PERIOD_50HZ (20000 - 1)
#define PWM_MID_POS_PULSE ((PWM_PERIOD_50HZ / 10 + PWM_PERIOD_50HZ / 20) / 2)

#include "stm32f3xx_hal.h"

typedef struct {
  ADC_HandleTypeDef hadc1;
  ADC_HandleTypeDef hadc2;
  DMA_HandleTypeDef hdma_adc1;
  DMA_HandleTypeDef hdma_adc2;
  CAN_HandleTypeDef hcan;
  I2C_HandleTypeDef hi2c1;
  I2C_HandleTypeDef hi2c3;
  SPI_HandleTypeDef hspi1;
  SPI_HandleTypeDef hspi3;
  TIM_HandleTypeDef htim1;
  UART_HandleTypeDef huart1;
} peripherals_t;

peripherals_t *get_peripherals(void);

void gpio_init(void);
void dma_init(void);
void can_init(void);
void uart1_init(void);
void spi1_init(void);
void adc1_init(void);
void adc2_init(void);
void i2c1_init(void);
void i2c3_init(void);
void spi3_init(void);
void tim1_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
