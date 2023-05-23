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

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
