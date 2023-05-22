
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
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
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
