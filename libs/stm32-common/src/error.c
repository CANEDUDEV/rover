#include "error.h"

#include "stm32f3xx_hal.h"

void error(void) {
  __disable_irq();
  while (1) {
  }
}
