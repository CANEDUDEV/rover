#ifndef PRINT_H
#define PRINT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

void print(UART_HandleTypeDef *huart, char *str);

#ifdef __cplusplus
}
#endif

#endif /* PRINT_H */
