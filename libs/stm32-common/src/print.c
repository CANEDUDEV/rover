#include "print.h"

#include <string.h>

#include "common-peripherals.h"

void print(char *str) {
  common_peripherals_t *common_peripherals = get_common_peripherals();
  HAL_UART_Transmit(&common_peripherals->huart1, (uint8_t *)str, strlen(str),
                    HAL_MAX_DELAY);
}
