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

#define OVER_CURRENT_Pin GPIO_PIN_13
#define OVER_CURRENT_GPIO_Port GPIOC
#define OVER_CURRENT_EXTI_IRQn EXTI15_10_IRQn
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOC
#define POWER_OFF_Pin GPIO_PIN_3
#define POWER_OFF_GPIO_Port GPIOC
#define CELL1_MEASURE_Pin GPIO_PIN_0
#define CELL1_MEASURE_GPIO_Port GPIOA
#define CELL2_MEASURE_Pin GPIO_PIN_1
#define CELL2_MEASURE_GPIO_Port GPIOA
#define CELL3_MEASURE_Pin GPIO_PIN_2
#define CELL3_MEASURE_GPIO_Port GPIOA
#define CELL4_MEASURE_Pin GPIO_PIN_3
#define CELL4_MEASURE_GPIO_Port GPIOA
#define CELL5_MEASURE_Pin GPIO_PIN_4
#define CELL5_MEASURE_GPIO_Port GPIOA
#define CELL6_MEASURE_Pin GPIO_PIN_5
#define CELL6_MEASURE_GPIO_Port GPIOA
#define I_PWR_A_MEASURE_Pin GPIO_PIN_6
#define I_PWR_A_MEASURE_GPIO_Port GPIOA
#define VBAT_I_MEASURE_Pin GPIO_PIN_5
#define VBAT_I_MEASURE_GPIO_Port GPIOC
#define REG_PWR_ON_Pin GPIO_PIN_2
#define REG_PWR_ON_GPIO_Port GPIOD

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
