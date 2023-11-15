#include "spi-flash.h"

#include <stdio.h>

#include "common-peripherals.h"
#include "common-ports.h"
#include "error.h"
#include "stm32f3xx_hal.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define READ_DATA_COMMAND 0x03
#define READ_SR1_COMMAND 0x05
#define WRITE_ENABLE_COMMAND 0x06
#define SECTOR_ERASE_COMMAND 0x20
#define PAGE_PROGRAM_COMMAND 0x02

#define SR1_BUSY 0x01
#define SR1_WEL 0x02

#define MAX_BLOCK_TIME_MS 100

// Helpers
static int program_page(uint32_t page_address, uint8_t *bytes, size_t size);
static int wait_until_ready(void);
static int write_enable(void);
static void start_cmd(void);
static void end_cmd(void);

int erase(uint32_t sector_address) {
  // sector_address bounds and alignment check
  if (sector_address > SPI_FLASH_SECTOR_SIZE * (SPI_FLASH_SECTOR_COUNT - 1) ||
      sector_address % SPI_FLASH_SECTOR_SIZE != 0) {
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

  // Set write enable flag before erase
  if (write_enable() != APP_OK) {
    return APP_NOT_OK;
  }

  // Erase
  common_peripherals_t *peripherals = get_common_peripherals();

  start_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
    printf("Failed to transmit SECTOR_ERASE_COMMAND\r\n");
    return APP_NOT_OK;
  }

  end_cmd();

  // Wait for flash
  return wait_until_ready();
}

int program(uint32_t address, uint8_t *bytes, size_t size) {
  // Bounds check
  if (address + size > SPI_FLASH_SIZE) {
    printf("error: page out of bounds\r\n");
    return APP_NOT_OK;
  }

  // We can program at most 1 page at a time, so we have to program in chunks.
  size_t bytes_left = size;
  size_t bytes_written = 0;

  while (bytes_left > 0) {
    uint32_t page_address = address + bytes_written;
    uint8_t *chunk = &bytes[bytes_written];
    size_t chunk_size = bytes_left;

    if (chunk_size > SPI_FLASH_PAGE_SIZE) {
      chunk_size = SPI_FLASH_PAGE_SIZE;
    }

    int err = program_page(page_address, chunk, chunk_size);
    if (err != APP_OK) {
      return err;
    }

    bytes_written += chunk_size;
    bytes_left -= chunk_size;
  }

  return APP_OK;
}

// Can read whole flash using one command
int read(uint32_t address, uint8_t *data, size_t size) {
  // Bounds check
  if (size > SPI_FLASH_SIZE) {
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

  // Read data
  common_peripherals_t *peripherals = get_common_peripherals();

  start_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
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
      end_cmd();
      printf("Failed to receive bytes\r\n");
      return APP_NOT_OK;
    }

    bytes_read += UINT16_MAX;
    bytes_left -= UINT16_MAX;
  }

  // Read remaining bytes
  if (HAL_SPI_Receive(&peripherals->hspi_flash, data + bytes_read,
                      size - bytes_read, MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
    printf("Failed to receive bytes\r\n");
    return APP_NOT_OK;
  }

  end_cmd();

  return APP_OK;
}

static int program_page(uint32_t page_address, uint8_t *bytes, size_t size) {
  // Command consists of command number followed by page address in big
  // endian format. If an entire page should be programmed the last address byte
  // should be set to 0.

  // NOLINTBEGIN(*-magic-numbers)
  uint8_t cmd[4];
  cmd[0] = PAGE_PROGRAM_COMMAND;
  cmd[1] = (page_address >> 2 * 8) & 0xFF;
  cmd[2] = (page_address >> 1 * 8) & 0xFF;
  cmd[3] = (page_address >> 0 * 8) & 0xFF;
  if (size == SPI_FLASH_PAGE_SIZE) {
    cmd[3] = 0;
  }
  // NOLINTEND(*-magic-numbers)

  // Set write enable flag before page program
  if (write_enable() != APP_OK) {
    return APP_NOT_OK;
  }

  common_peripherals_t *peripherals = get_common_peripherals();

  // Transmit command
  start_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
    printf("Failed to transmit PAGE_PROGRAM_COMMAND\r\n");
    return APP_NOT_OK;
  }

  // Transmit data bytes
  if (HAL_SPI_Transmit(&peripherals->hspi_flash, bytes, size,
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
    printf("Failed to transmit data bytes\r\n");
    return APP_NOT_OK;
  }

  end_cmd();

  // Wait for flash
  return wait_until_ready();
}

static int wait_until_ready(void) {
  common_peripherals_t *peripherals = get_common_peripherals();
  uint8_t cmd = READ_SR1_COMMAND;

  uint8_t response = SR1_BUSY;
  while (response & SR1_BUSY) {
    start_cmd();
    if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                         MAX_BLOCK_TIME_MS) != HAL_OK) {
      end_cmd();
      printf("Failed to transmit READ_SR1_COMMAND\r\n");
      return APP_NOT_OK;
    }

    if (HAL_SPI_Receive(&peripherals->hspi_flash, &response, sizeof(response),
                        MAX_BLOCK_TIME_MS) != HAL_OK) {
      end_cmd();
      printf("Failed to receive response to READ_SR1_COMMAND\r\n");
      return APP_NOT_OK;
    }
    end_cmd();
  }

  return APP_OK;
}

static int write_enable(void) {
  common_peripherals_t *peripherals = get_common_peripherals();
  uint8_t cmd = WRITE_ENABLE_COMMAND;

  // Set write enable flag
  start_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
    printf("Failed to transmit WRITE_ENABLE_COMMAND\r\n");
    return APP_NOT_OK;
  }

  end_cmd();

  // Check if flag is set
  cmd = READ_SR1_COMMAND;

  start_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
    printf("Failed to transmit READ_SR1_COMMAND\r\n");
    return APP_NOT_OK;
  }

  uint8_t response = 0;
  if (HAL_SPI_Receive(&peripherals->hspi_flash, &response, sizeof(response),
                      MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_cmd();
    printf("Failed to receive response to READ_SR1_COMMAND\r\n");
    return APP_NOT_OK;
  }

  end_cmd();

  if ((response & SR1_WEL) == 0) {
    printf("Failed to set WRITE_ENABLE flag\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

static void start_cmd(void) {
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);
}

static void end_cmd(void) {
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
  taskEXIT_CRITICAL();
}
