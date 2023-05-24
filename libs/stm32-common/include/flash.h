#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

// 1 page (2KiB) is reserved for persistent storage
#define FLASH_RW_START 0x0807F800
#define FLASH_RW_END (FLASH_RW_START + 2 * 1024)

// Setup the writeable flash memory area. Return APP_OK on success, APP_NOT_OK
// otherwise.
int flash_init(void);

// Erase the persistent storage. This needs to be done prior to first time
// programming of the flash. Return APP_OK on success, APP_NOT_OK otherwise.
int flash_erase(void);

// Read from FLASH_RW area. Return APP_OK on success, APP_NOT_OK otherwise.
int flash_read(uint32_t addr, void *data, size_t len);

// Write to FLASH_RW area. The len parameter needs to be word-aligned (32-bit)
// since we program in word mode. Return APP_OK on success, APP_NOT_OK
// otherwise.
int flash_write(uint32_t addr, const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
