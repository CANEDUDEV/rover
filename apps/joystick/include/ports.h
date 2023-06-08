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

#define SWITCH4_PIN2_PIN GPIO_PIN_13
#define SWITCH4_PIN2_GPIO_PORT GPIOC
#define SWITCH2_PIN1_PIN GPIO_PIN_15
#define SWITCH2_PIN1_GPIO_PORT GPIOC
#define GPIO0_PIN GPIO_PIN_0
#define GPIO0_GPIO_PORT GPIOC
#define GPIO1_PIN GPIO_PIN_1
#define GPIO1_GPIO_PORT GPIOC
#define GPIO2_PIN GPIO_PIN_2
#define GPIO2_GPIO_PORT GPIOC
#define GPIO3_PIN GPIO_PIN_3
#define GPIO3_GPIO_PORT GPIOC
#define AN1_PIN GPIO_PIN_0
#define AN1_GPIO_PORT GPIOA
#define AN2_PIN GPIO_PIN_1
#define AN2_GPIO_PORT GPIOA
#define AN3_PIN GPIO_PIN_2
#define AN3_GPIO_PORT GPIOA
#define AN4_PIN GPIO_PIN_3
#define AN4_GPIO_PORT GPIOA
#define SWITCH3_PIN1_PIN GPIO_PIN_4
#define SWITCH3_PIN1_GPIO_PORT GPIOA
#define SWITCH3_PIN2_PIN GPIO_PIN_5
#define SWITCH3_PIN2_GPIO_PORT GPIOA
#define SWITCH4_PIN1_PIN GPIO_PIN_6
#define SWITCH4_PIN1_GPIO_PORT GPIOA
#define GPIO_PWRON_1_PIN GPIO_PIN_4
#define GPIO_PWRON_1_GPIO_PORT GPIOC
#define GPIO_PWRON_2_PIN GPIO_PIN_5
#define GPIO_PWRON_2_GPIO_PORT GPIOC
#define GPIO_PWRON_3_PIN GPIO_PIN_0
#define GPIO_PWRON_3_GPIO_PORT GPIOB
#define GPIO_PWRON_4_PIN GPIO_PIN_1
#define GPIO_PWRON_4_GPIO_PORT GPIOB
#define VDD_IO_LEVEL_PIN GPIO_PIN_2
#define VDD_IO_LEVEL_GPIO_PORT GPIOB
#define SWITCH1_PIN1_PIN GPIO_PIN_10
#define SWITCH1_PIN1_GPIO_PORT GPIOB
#define SWITCH1_PIN2_PIN GPIO_PIN_11
#define SWITCH1_PIN2_GPIO_PORT GPIOB
#define SWITCH2_PIN2_PIN GPIO_PIN_6
#define SWITCH2_PIN2_GPIO_PORT GPIOC
#define SPI3_NSS_PIN GPIO_PIN_2
#define SPI3_NSS_GPIO_PORT GPIOD
#define SPI3_SCK_GPIO_PORT GPIOC
#define SPI3_SCK_PIN GPIO_PIN_10
#define SPI3_MISO_PIN GPIO_PIN_11
#define SPI3_MOSI_PIN GPIO_PIN_12
#define I2C1_SCL_GPIO_PORT GPIOB
#define I2C1_SCL_PIN GPIO_PIN_6
#define I2C1_SDA_PIN GPIO_PIN_7

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
