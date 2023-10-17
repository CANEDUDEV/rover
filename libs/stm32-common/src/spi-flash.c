#include "spi-flash.h"

#include <stdint.h>
#include <stdio.h>

#include "common-peripherals.h"
#include "common-ports.h"
#include "error.h"
#include "stm32f3xx_hal.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// littlefs
#include "lfs.h"

#define READ_DATA_COMMAND 0x03
#define READ_SR1_COMMAND 0x05
#define WRITE_ENABLE_COMMAND 0x06
#define SECTOR_ERASE_COMMAND 0x20
#define PAGE_PROGRAM_COMMAND 0x02

#define SR1_BUSY 0x01
#define SR1_WEL 0x02

#define MAX_BLOCK_TIME_MS 100

#define SECTOR_SIZE (4 * 1024)
#define SECTOR_COUNT 512
#define PAGE_SIZE 256
#define MEMORY_SIZE (SECTOR_SIZE * SECTOR_COUNT)

// SPI flash driver interface
static int erase(uint32_t sector_address);
static int program(uint32_t page_address, uint8_t *bytes, size_t size);
static int read(uint32_t address, uint8_t *data, size_t size);

// Helpers
static int wait_until_ready(void);
static int write_enable(void);

// Statically allocate buffers for littelfs
static uint8_t lfs_read_buf[PAGE_SIZE];
static uint8_t lfs_prog_buf[PAGE_SIZE];
static uint8_t lfs_lookahead_buf[PAGE_SIZE];

// Functions required by littlefs
static int spi_flash_read(const struct lfs_config *config, lfs_block_t block,
                          lfs_off_t offset, void *buffer, lfs_size_t size) {
  return read(block * config->block_size + offset, buffer, size);
}

static int spi_flash_prog(const struct lfs_config *config, lfs_block_t block,
                          lfs_off_t offset, const void *buffer,
                          lfs_size_t size) {
  return program(block * config->block_size + offset, (uint8_t *)buffer, size);
}

static int spi_flash_erase(const struct lfs_config *config, lfs_block_t block) {
  return erase(block * config->block_size);
}

static int spi_flash_sync(const struct lfs_config *config) {
  (void)config;
  return APP_OK;
}

static const struct lfs_config lfs_cfg = {
    // Operations
    .read = spi_flash_read,
    .prog = spi_flash_prog,
    .erase = spi_flash_erase,
    .sync = spi_flash_sync,

    // Configuration
    .read_size = 1,
    .prog_size = 1,
    .block_size = SECTOR_SIZE,  // Flash is sector-erasable
    .block_count = SECTOR_COUNT,
    .block_cycles = 100,  // Optimize for wear leveling

    .cache_size = sizeof(lfs_read_buf),
    .read_buffer = lfs_read_buf,
    .prog_buffer = lfs_prog_buf,

    .lookahead_size = sizeof(lfs_lookahead_buf),
    .lookahead_buffer = lfs_lookahead_buf,
};

const struct lfs_config *get_spi_flash_lfs_config(void) { return &lfs_cfg; }

static int wait_until_ready(void) {
  common_peripherals_t *peripherals = get_common_peripherals();
  uint8_t cmd = READ_SR1_COMMAND;

  uint8_t response = SR1_BUSY;
  while (response & SR1_BUSY) {
    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                         MAX_BLOCK_TIME_MS) != HAL_OK) {
      HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
      taskEXIT_CRITICAL();
      printf("Failed to transmit READ_SR1_COMMAND\r\n");
      return APP_NOT_OK;
    }

    if (HAL_SPI_Receive(&peripherals->hspi_flash, &response, sizeof(response),
                        MAX_BLOCK_TIME_MS) != HAL_OK) {
      HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
      taskEXIT_CRITICAL();
      printf("Failed to receive response to READ_SR1_COMMAND\r\n");
      return APP_NOT_OK;
    }
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
  }

  return APP_OK;
}

static int write_enable(void) {
  common_peripherals_t *peripherals = get_common_peripherals();
  uint8_t cmd = WRITE_ENABLE_COMMAND;

  // Set write enable flag
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to transmit WRITE_ENABLE_COMMAND\r\n");
    return APP_NOT_OK;
  }

  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
  taskEXIT_CRITICAL();

  // Check if flag is set
  cmd = READ_SR1_COMMAND;

  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to transmit READ_SR1_COMMAND\r\n");
    return APP_NOT_OK;
  }

  uint8_t response = 0;
  if (HAL_SPI_Receive(&peripherals->hspi_flash, &response, sizeof(response),
                      MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to receive response to READ_SR1_COMMAND\r\n");
    return APP_NOT_OK;
  }

  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
  taskEXIT_CRITICAL();

  if ((response & SR1_WEL) == 0) {
    printf("Failed to set WRITE_ENABLE flag\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

static int erase(uint32_t sector_address) {
  // sector_address bounds and alignment check
  if (sector_address > SECTOR_SIZE * (SECTOR_COUNT - 1) ||
      sector_address % SECTOR_SIZE != 0) {
    printf("error: sector_address out of bounds\r\n");
    return APP_NOT_OK;
  }

  // Command consists of command number followed by sector address to erase in
  // big endian format.

  // NOLINTBEGIN(*-magic-numbers)
  uint8_t cmd[4];
  cmd[0] = SECTOR_ERASE_COMMAND;
  cmd[1] = (sector_address >> 2 * 8) & 0xFF;
  cmd[2] = (sector_address >> 1 * 8) & 0xFF;
  cmd[3] = (sector_address >> 0 * 8) & 0xFF;
  // NOLINTEND(*-magic-numbers)

  // Wait for flash
  if (wait_until_ready() != APP_OK) {
    return APP_NOT_OK;
  }

  // Set write enable flag before erase
  if (write_enable() != APP_OK) {
    return APP_NOT_OK;
  }

  // Erase
  common_peripherals_t *peripherals = get_common_peripherals();

  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to transmit SECTOR_ERASE_COMMAND\r\n");
    return APP_NOT_OK;
  }

  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
  taskEXIT_CRITICAL();

  return APP_OK;
}

// Can program at most 256 bytes at once.
static int program(uint32_t page_address, uint8_t *bytes, size_t size) {
  // Bounds check
  if (size > PAGE_SIZE || page_address + size > MEMORY_SIZE) {
    printf("error: page out of bounds\r\n");
    return APP_NOT_OK;
  }

  // Command consists of command number followed by page address in big
  // endian format. If an entire page should be programmed the last address byte
  // should be set to 0.

  // NOLINTBEGIN(*-magic-numbers)
  uint8_t cmd[4];
  cmd[0] = PAGE_PROGRAM_COMMAND;
  cmd[1] = (page_address >> 2 * 8) & 0xFF;
  cmd[2] = (page_address >> 1 * 8) & 0xFF;
  cmd[3] = (page_address >> 0 * 8) & 0xFF;
  if (size == PAGE_SIZE) {
    cmd[3] = 0;
  }
  // NOLINTEND(*-magic-numbers)

  // Wait for flash
  if (wait_until_ready() != APP_OK) {
    return APP_NOT_OK;
  }

  // Set write enable flag before page program
  if (write_enable() != APP_OK) {
    return APP_NOT_OK;
  }

  common_peripherals_t *peripherals = get_common_peripherals();

  // Transmit command
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to transmit PAGE_PROGRAM_COMMAND\r\n");
    return APP_NOT_OK;
  }

  // Transmit data bytes
  if (HAL_SPI_Transmit(&peripherals->hspi_flash, bytes, size,
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to transmit data bytes\r\n");
    return APP_NOT_OK;
  }

  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
  taskEXIT_CRITICAL();

  return APP_OK;
}

// Can read whole flash using one command
static int read(uint32_t address, uint8_t *data, size_t size) {
  // Bounds check
  if (size > MEMORY_SIZE) {
    printf("error: size too big\r\n");
    return APP_NOT_OK;
  }

  // Command consists of command number followed by page address in big
  // endian format.

  // NOLINTBEGIN(*-magic-numbers)
  uint8_t cmd[4];
  cmd[0] = READ_DATA_COMMAND;
  cmd[1] = (address >> 2 * 8) & 0xFF;
  cmd[2] = (address >> 1 * 8) & 0xFF;
  cmd[3] = (address >> 0 * 8) & 0xFF;
  // NOLINTEND(*-magic-numbers)

  // Wait for flash
  if (wait_until_ready() != APP_OK) {
    return APP_NOT_OK;
  }

  // Read data
  common_peripherals_t *peripherals = get_common_peripherals();

  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to transmit READ_DATA_COMMAND\r\n");
    return APP_NOT_OK;
  }

  // If the requested data is larger than what HAL_SPI_Receive can receive at a
  // time, we need to loop
  uint32_t bytes_left = size;
  uint32_t bytes_read = 0;

  while (bytes_left > UINT16_MAX) {
    if (HAL_SPI_Receive(&peripherals->hspi_flash, data + bytes_read, UINT16_MAX,
                        MAX_BLOCK_TIME_MS) != HAL_OK) {
      HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
      taskEXIT_CRITICAL();
      printf("Failed to receive bytes\r\n");
      return APP_NOT_OK;
    }

    bytes_read += UINT16_MAX;
    bytes_left -= UINT16_MAX;
  }

  // Read remaining bytes
  if (HAL_SPI_Receive(&peripherals->hspi_flash, data + bytes_read,
                      size - bytes_read, MAX_BLOCK_TIME_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    printf("Failed to receive bytes\r\n");
    return APP_NOT_OK;
  }

  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
  taskEXIT_CRITICAL();

  return APP_OK;
}
