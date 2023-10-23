#include <stdio.h>

#include "error.h"
#include "stm32f3xx_hal.h"

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  printf("Assertion failed. File: %s, Line: %u\r\n", (char *)file, line);
  error();
}
#endif /* USE_FULL_ASSERT */

// NOLINTBEGIN(bugprone-reserved-identifier)
void __assert_fail(const char *__assertion, const char *__file,
                   unsigned int __line, const char *__function) {
  printf("Assertion %s failed. File: %s, Function: %s, Line: %u\r\n",
         __assertion, __file, __function, __line);
  error();
}
// NOLINTEND(bugprone-reserved-identifier)
