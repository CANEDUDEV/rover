
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
#define SWITCH4_PIN2_Pin GPIO_PIN_13
#define SWITCH4_PIN2_GPIO_Port GPIOC
#define SWITCH2_PIN1_Pin GPIO_PIN_15
#define SWITCH2_PIN1_GPIO_Port GPIOC
#define GPIO0_Pin GPIO_PIN_0
#define GPIO0_GPIO_Port GPIOC
#define GPIO1_Pin GPIO_PIN_1
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_2
#define GPIO2_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_3
#define GPIO3_GPIO_Port GPIOC
#define AN1_Pin GPIO_PIN_0
#define AN1_GPIO_Port GPIOA
#define AN2_Pin GPIO_PIN_1
#define AN2_GPIO_Port GPIOA
#define AN3_Pin GPIO_PIN_2
#define AN3_GPIO_Port GPIOA
#define AN4_Pin GPIO_PIN_3
#define AN4_GPIO_Port GPIOA
#define SWITCH3_PIN1_Pin GPIO_PIN_4
#define SWITCH3_PIN1_GPIO_Port GPIOA
#define SWITCH3_PIN2_Pin GPIO_PIN_5
#define SWITCH3_PIN2_GPIO_Port GPIOA
#define SWITCH4_PIN1_Pin GPIO_PIN_6
#define SWITCH4_PIN1_GPIO_Port GPIOA
#define GPIO_PWRON_1_Pin GPIO_PIN_4
#define GPIO_PWRON_1_GPIO_Port GPIOC
#define GPIO_PWRON_2_Pin GPIO_PIN_5
#define GPIO_PWRON_2_GPIO_Port GPIOC
#define GPIO_PWRON_3_Pin GPIO_PIN_0
#define GPIO_PWRON_3_GPIO_Port GPIOB
#define GPIO_PWRON_4_Pin GPIO_PIN_1
#define GPIO_PWRON_4_GPIO_Port GPIOB
#define VDD_IO_LEVEL_Pin GPIO_PIN_2
#define VDD_IO_LEVEL_GPIO_Port GPIOB
#define SWITCH1_PIN1_Pin GPIO_PIN_10
#define SWITCH1_PIN1_GPIO_Port GPIOB
#define SWITCH1_PIN2_Pin GPIO_PIN_11
#define SWITCH1_PIN2_GPIO_Port GPIOB
#define SWITCH2_PIN2_Pin GPIO_PIN_6
#define SWITCH2_PIN2_GPIO_Port GPIOC
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
#define SPI3_NSS_Pin GPIO_PIN_2
#define SPI3_NSS_GPIO_Port GPIOD
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
