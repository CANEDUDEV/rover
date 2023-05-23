
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define PWM_PSC_1MHZ (72 - 1)
#define PWM_PERIOD_50HZ (20000 - 1)
#define PWM_MID_POS_PULSE ((PWM_PERIOD_50HZ / 10 + PWM_PERIOD_50HZ / 20) / 2)
#define SERVO_PWM_Pin GPIO_PIN_3
#define SERVO_PWM_GPIO_Port GPIOC
#define SENSOR_POWER_Pin GPIO_PIN_0
#define SENSOR_POWER_GPIO_Port GPIOA
#define DEBUG_LED_Pin GPIO_PIN_1
#define DEBUG_LED_GPIO_Port GPIOA
#define SERVO_CURRENT_Pin GPIO_PIN_2
#define SERVO_CURRENT_GPIO_Port GPIOA
#define BAT_VOLTAGE_Pin GPIO_PIN_3
#define BAT_VOLTAGE_GPIO_Port GPIOA
#define VCC_SERVO_VOLTAGE_Pin GPIO_PIN_4
#define VCC_SERVO_VOLTAGE_GPIO_Port GPIOA
#define H_BRIDGE_VPROP_Pin GPIO_PIN_5
#define H_BRIDGE_VPROP_GPIO_Port GPIOA
#define H_BRIDGE_ENABLE_Pin GPIO_PIN_6
#define H_BRIDGE_ENABLE_GPIO_Port GPIOA
#define H_BRIDGE_MODE1_Pin GPIO_PIN_7
#define H_BRIDGE_MODE1_GPIO_Port GPIOA
#define H_BRIDDGE_MODE2_Pin GPIO_PIN_4
#define H_BRIDDGE_MODE2_GPIO_Port GPIOC
#define H_BRIDGE_PHASE_Pin GPIO_PIN_5
#define H_BRIDGE_PHASE_GPIO_Port GPIOC
#define H_BRIDGE_nSLEEP_Pin GPIO_PIN_0
#define H_BRIDGE_nSLEEP_GPIO_Port GPIOB
#define H_BRIDGE_nFAULT_Pin GPIO_PIN_1
#define H_BRIDGE_nFAULT_GPIO_Port GPIOB
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
