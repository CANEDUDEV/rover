#ifndef COMMON_PORTS_H
#define COMMON_PORTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

#define CAN_GPIO_PORT GPIOB
#define CAN_RX_PIN GPIO_PIN_8
#define CAN_TX_PIN GPIO_PIN_9

#define UART1_GPIO_PORT GPIOA
#define UART1_RX_PIN GPIO_PIN_10
#define UART1_TX_PIN GPIO_PIN_9

#define CAN_FD_INT_PIN GPIO_PIN_7
#define CAN_FD_INT_GPIO_PORT GPIOC
#define CAN_FD_SOF_PIN GPIO_PIN_8
#define CAN_FD_SOF_GPIO_PORT GPIOC
#define CAN_FD_INT1_PIN GPIO_PIN_11
#define CAN_FD_INT1_GPIO_PORT GPIOA
#define CAN_FD_INT0_PIN GPIO_PIN_12
#define CAN_FD_INT0_GPIO_PORT GPIOA
#define CAN_FD_SPI_CS_PIN GPIO_PIN_15
#define CAN_FD_SPI_CS_GPIO_PORT GPIOA
#define CAN_FD_SPI_SCK_PIN GPIO_PIN_3
#define CAN_FD_SPI_SCK_GPIO_PORT GPIOB
#define CAN_FD_SPI_MISO_PIN GPIO_PIN_4
#define CAN_FD_SPI_MISO_GPIO_PORT GPIOB
#define CAN_FD_SPI_MOSI_PIN GPIO_PIN_5
#define CAN_FD_SPI_MOSI_GPIO_PORT GPIOB

#define SPI_FLASH_GPIO_PORT GPIOB
#define SPI_FLASH_NSS_PIN GPIO_PIN_12
#define SPI_FLASH_SCK_PIN GPIO_PIN_13
#define SPI_FLASH_SO_IO1_PIN GPIO_PIN_14
#define SPI_FLASH_SI_IO0_PIN GPIO_PIN_15

#ifdef __cplusplus
}
#endif

#endif /* COMMON_PORTS_H */
