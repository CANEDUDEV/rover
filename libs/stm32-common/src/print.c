#include "print.h"

#include <string.h>

void print(UART_HandleTypeDef *huart, char *str) {
  HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
