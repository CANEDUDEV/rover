#ifndef COMMON_PERIPHERALS_H
#define COMMON_PERIPHERALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

typedef struct {
  CAN_HandleTypeDef hcan;
  SPI_HandleTypeDef hcanfd;
  CRC_HandleTypeDef hcrc;
  SPI_HandleTypeDef hspi_flash;
  UART_HandleTypeDef huart1;
} common_peripherals_t;

common_peripherals_t* get_common_peripherals(void);

void common_peripherals_init(void);

// MSP init functions required by STM32 HAL
void can_msp_init(void);
void can_msp_deinit(void);
void crc_msp_init(void);
void crc_msp_deinit(void);
void spi1_msp_init(void);
void spi1_msp_deinit(void);
void spi2_msp_init(void);
void spi2_msp_deinit(void);
void uart1_msp_init(void);
void uart1_msp_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_PERIPHERALS_H */
