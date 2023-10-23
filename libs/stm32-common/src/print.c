#include "common-peripherals.h"

// Enable printf usage with uart

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {  // NOLINT
  common_peripherals_t *common_peripherals = get_common_peripherals();
  HAL_UART_Transmit(&common_peripherals->huart1, (uint8_t *)&ch, 1,
                    HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char *ptr, int len) {  // NOLINT
  (void)file;
  for (int i = 0; i < len; i++) {
    __io_putchar(*ptr++);
  }
  return len;
}
