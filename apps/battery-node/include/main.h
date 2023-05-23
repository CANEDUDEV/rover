
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

// Disable warnings for generated code
// NOLINTBEGIN(bugprone-reserved-identifier)

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
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
#define CAN_FD_INT_Pin GPIO_PIN_7
#define CAN_FD_INT_GPIO_Port GPIOC
#define CAN_FD_SOF_Pin GPIO_PIN_8
#define CAN_FD_SOF_GPIO_Port GPIOC
#define CAN_FD_INT1_Pin GPIO_PIN_11
#define CAN_FD_INT1_GPIO_Port GPIOA
#define CAN_FD_INT0_Pin GPIO_PIN_12
#define CAN_FD_INT0_GPIO_Port GPIOA
#define CAN_FD_SPI_CS_Pin GPIO_PIN_15
#define CAN_FD_SPI_CS_GPIO_Port GPIOA
#define REG_PWR_ON_Pin GPIO_PIN_2
#define REG_PWR_ON_GPIO_Port GPIOD
#define CAN_FD_SPI_SCK_Pin GPIO_PIN_3
#define CAN_FD_SPI_SCK_GPIO_Port GPIOB
#define CAN_FD_SPI_MISO_Pin GPIO_PIN_4
#define CAN_FD_SPI_MISO_GPIO_Port GPIOB
#define CAN_FD_SPI_MOSI_Pin GPIO_PIN_5
#define CAN_FD_SPI_MOSI_GPIO_Port GPIOB

// NOLINTEND(bugprone-reserved-identifier)

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
