#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define SPI_FLASH_SECTOR_SIZE (4 * 1024)
#define SPI_FLASH_SECTOR_COUNT 512
#define SPI_FLASH_PAGE_SIZE 256
#define SPI_FLASH_SIZE (SPI_FLASH_SECTOR_SIZE * SPI_FLASH_SECTOR_COUNT)

// SPI flash driver interface
int erase(uint32_t sector_address);
int program(uint32_t address, uint8_t *bytes, size_t size);
int read(uint32_t address, uint8_t *data, size_t size);

// Tries to write the whole flash and read back. Useful for debugging.
void test_spi_flash(void);
// Hexdump of flash
void dump_flash(uint32_t start_address, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* SPI_FLASH_H */
