#include "flash.h"

#include <string.h>

#include "error.h"
#include "stm32f3xx_hal.h"

#define FLASH_ERASE_OK 0xFFFFFFFF  // Specified by STM32 HAL
#define FLASH_ERASED_SYMBOL 0xDEADBEEF

/* Check if we have erased the flash by checking for existence of
 * FLASH_ERASED_SYMBOL. If we haven't, we erase it then write the
 * FLASH_ERASED_SYMBOL to end of writable area. Erasing flash is only needed
 * before first time use, which is why we write the FLASH_ERASED_SYMBOL.
 */
int flash_init(void) {
  uint32_t flashErased = 0;
  if (flash_read(FLASH_RW_END - sizeof(flashErased), &flashErased,
                 sizeof(flashErased)) != APP_OK) {
    return APP_NOT_OK;
  }
  // Check if flash is already erased
  if (flashErased == FLASH_ERASED_SYMBOL) {
    return APP_OK;
  }
  flash_erase();
  flashErased = FLASH_ERASED_SYMBOL;
  if (flash_write(FLASH_RW_END - sizeof(uint32_t), &flashErased,
                  sizeof(flashErased)) != APP_OK) {
    return APP_NOT_OK;
  }
  return APP_OK;
}

int flash_erase(void) {
  // 1 page = 2KB
  FLASH_EraseInitTypeDef eraseConf = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = FLASH_RW_START,
      .NbPages = 1,
  };
  HAL_FLASH_Unlock();
  uint32_t err = 0;
  HAL_FLASHEx_Erase(&eraseConf, &err);
  HAL_FLASH_Lock();
  if (err != FLASH_ERASE_OK) {
    return APP_NOT_OK;
  }
  return APP_OK;
}

int flash_read(uint32_t addr, void *data, size_t len) {
  // Check if within writeable memory bounds
  if (addr < FLASH_RW_START) {
    return APP_NOT_OK;
  }
  if (addr + len > FLASH_RW_END) {
    return APP_NOT_OK;
  }
  memcpy(data, (const void *)addr, len);
  return APP_OK;
}

int flash_write(uint32_t addr, const void *data, size_t len) {
  // Check if within writeable memory bounds
  if (addr < FLASH_RW_START) {
    return APP_NOT_OK;
  }
  if (addr + len > FLASH_RW_END) {
    return APP_NOT_OK;
  }
  // Check if data is word-aligned
  if (len % 4 > 0) {
    return APP_NOT_OK;
  }

  const size_t wordLength = 4;
  uint32_t word = 0;
  HAL_FLASH_Unlock();
  for (size_t i = 0; i < len; i += wordLength) {
    memcpy(&word, (uint8_t *)data + i, wordLength);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, (uint64_t)word);
  }
  HAL_FLASH_Lock();

  return APP_OK;
}
