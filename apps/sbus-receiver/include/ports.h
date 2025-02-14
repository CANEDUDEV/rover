/*******************************************************************************
 * @file ports.h
 *
 * Contains preprocessor definitions for the various GPIO ports.
 ******************************************************************************/
#ifndef PORTS_H
#define PORTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

#define TIM1_GPIO_PORT GPIOC
#define TIM1_CH1_PIN GPIO_PIN_0
#define TIM1_CH2_PIN GPIO_PIN_1
#define TIM1_CH3_PIN GPIO_PIN_2
#define TIM1_CH4_PIN GPIO_PIN_3

#define GPIO_PWRON_1_2_GPIO_PORT GPIOC
#define GPIO_PWRON_1_PIN GPIO_PIN_4
#define GPIO_PWRON_2_PIN GPIO_PIN_5

#define GPIO_PWRON_3_4_GPIO_PORT GPIOB
#define GPIO_PWRON_3_PIN GPIO_PIN_0
#define GPIO_PWRON_4_PIN GPIO_PIN_1

#define UART2_GPIO_PORT GPIOA
#define UART2_RX_PIN GPIO_PIN_3

#define VDD_IO_LEVEL_GPIO_PORT GPIOB
#define VDD_IO_LEVEL_PIN GPIO_PIN_2

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
