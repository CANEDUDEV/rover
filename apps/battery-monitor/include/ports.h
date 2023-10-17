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

#define OVER_CURRENT_PIN GPIO_PIN_13
#define OVER_CURRENT_GPIO_PORT GPIOC
#define OVER_CURRENT_EXTI_IRQn EXTI15_10_IRQn
#define LED4_PIN GPIO_PIN_15
#define LED4_GPIO_PORT GPIOC
#define LED3_PIN GPIO_PIN_0
#define LED3_GPIO_PORT GPIOC
#define LED2_PIN GPIO_PIN_1
#define LED2_GPIO_PORT GPIOC
#define LED1_PIN GPIO_PIN_2
#define LED1_GPIO_PORT GPIOC
#define nPOWER_OFF_PIN GPIO_PIN_3
#define nPOWER_OFF_GPIO_PORT GPIOC
#define CELL1_MEASURE_PIN GPIO_PIN_0
#define CELL1_MEASURE_GPIO_PORT GPIOA
#define CELL2_MEASURE_PIN GPIO_PIN_1
#define CELL2_MEASURE_GPIO_PORT GPIOA
#define CELL3_MEASURE_PIN GPIO_PIN_2
#define CELL3_MEASURE_GPIO_PORT GPIOA
#define CELL4_MEASURE_PIN GPIO_PIN_3
#define CELL4_MEASURE_GPIO_PORT GPIOA
#define CELL5_MEASURE_PIN GPIO_PIN_4
#define CELL5_MEASURE_GPIO_PORT GPIOA
#define CELL6_MEASURE_PIN GPIO_PIN_5
#define CELL6_MEASURE_GPIO_PORT GPIOA
#define I_PWR_A_MEASURE_PIN GPIO_PIN_6
#define I_PWR_A_MEASURE_GPIO_PORT GPIOA
#define VBAT_I_MEASURE_PIN GPIO_PIN_5
#define VBAT_I_MEASURE_GPIO_PORT GPIOC
#define nREG_POWER_ON_PIN GPIO_PIN_2
#define nREG_POWER_ON_GPIO_PORT GPIOD
#define I2C1_SCL_GPIO_PORT GPIOB
#define I2C1_SCL_PIN GPIO_PIN_6
#define I2C1_SDA_PIN GPIO_PIN_7

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
