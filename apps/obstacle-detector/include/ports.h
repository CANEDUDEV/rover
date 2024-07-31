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

#define VDD_IO_LEVEL_GPIO_PORT GPIOB
#define VDD_IO_LEVEL_PIN GPIO_PIN_2

#define AN_GPIO_PORT GPIOA
#define AN0_GPIO_PIN GPIO_PIN_0
#define AN1_GPIO_PIN GPIO_PIN_1
#define AN2_GPIO_PIN GPIO_PIN_2
#define AN3_GPIO_PIN GPIO_PIN_3

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
