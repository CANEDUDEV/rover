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

#define UART2_GPIO_PORT GPIOA
#define UART2_RX_PIN GPIO_PIN_3
#define VDD_IO_LEVEL_GPIO_PORT GPIOB
#define VDD_IO_LEVEL_PIN GPIO_PIN_2

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
