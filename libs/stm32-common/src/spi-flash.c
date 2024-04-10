#include "spi-flash.h"

#include <stdio.h>
#include <string.h>

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
int program_page(uint32_t page_address, uint8_t *bytes, size_t size);
int wait_until_ready(void);
int set_write_enable_flag(void);
void start_spi_cmd(void);
void end_spi_cmd(void);

int spi_flash_workaround_init(void) {
  // There is a timing issue on cold reset where the flash cannot be read
  // properly without first writing or erasing some part of the flash. We work
  // around this by writing the last byte in the flash.

  uint8_t data = 0xFF;  // NOLINT(*-magic-numbers)
  if (program_page(SPI_FLASH_SIZE - 1, &data, sizeof(data)) != APP_OK) {
    printf("Program didn't work, trying erase instead");
    return erase(SPI_FLASH_SIZE - SPI_FLASH_SECTOR_SIZE);
  }
  return APP_OK;
}

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
  if (set_write_enable_flag() != APP_OK) {
    return APP_NOT_OK;
  }

  // Erase
  common_peripherals_t *peripherals = get_common_peripherals();

  start_spi_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
    printf("Failed to transmit SECTOR_ERASE_COMMAND\r\n");
    return APP_NOT_OK;
  }

  end_spi_cmd();

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
  if (address + size > SPI_FLASH_SIZE) {
    printf("error: size out of bounds\r\n");
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

  start_spi_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
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
      end_spi_cmd();
      printf("Failed to receive bytes\r\n");
      return APP_NOT_OK;
    }

    bytes_read += UINT16_MAX;
    bytes_left -= UINT16_MAX;
  }

  // Read remaining bytes
  if (HAL_SPI_Receive(&peripherals->hspi_flash, data + bytes_read,
                      size - bytes_read, MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
    printf("Failed to receive bytes\r\n");
    return APP_NOT_OK;
  }

  end_spi_cmd();

  return APP_OK;
}

int program_page(uint32_t page_address, uint8_t *bytes, size_t size) {
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
  if (set_write_enable_flag() != APP_OK) {
    return APP_NOT_OK;
  }

  common_peripherals_t *peripherals = get_common_peripherals();

  // Transmit command
  start_spi_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
    printf("Failed to transmit PAGE_PROGRAM_COMMAND\r\n");
    return APP_NOT_OK;
  }

  // Transmit data bytes
  if (HAL_SPI_Transmit(&peripherals->hspi_flash, bytes, size,
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
    printf("Failed to transmit data bytes\r\n");
    return APP_NOT_OK;
  }

  end_spi_cmd();

  // Wait for flash
  return wait_until_ready();
}

int wait_until_ready(void) {
  common_peripherals_t *peripherals = get_common_peripherals();
  uint8_t cmd = READ_SR1_COMMAND;

  uint8_t response = SR1_BUSY;
  while (response & SR1_BUSY) {
    start_spi_cmd();
    if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                         MAX_BLOCK_TIME_MS) != HAL_OK) {
      end_spi_cmd();
      printf("Failed to transmit READ_SR1_COMMAND\r\n");
      return APP_NOT_OK;
    }

    if (HAL_SPI_Receive(&peripherals->hspi_flash, &response, sizeof(response),
                        MAX_BLOCK_TIME_MS) != HAL_OK) {
      end_spi_cmd();
      printf("Failed to receive response to READ_SR1_COMMAND\r\n");
      return APP_NOT_OK;
    }
    end_spi_cmd();
  }

  return APP_OK;
}

int set_write_enable_flag(void) {
  common_peripherals_t *peripherals = get_common_peripherals();
  uint8_t cmd = WRITE_ENABLE_COMMAND;

  // Set write enable flag
  start_spi_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
    printf("Failed to transmit WRITE_ENABLE_COMMAND\r\n");
    return APP_NOT_OK;
  }

  end_spi_cmd();

  // Check if flag is set
  cmd = READ_SR1_COMMAND;

  start_spi_cmd();

  if (HAL_SPI_Transmit(&peripherals->hspi_flash, &cmd, sizeof(cmd),
                       MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
    printf("Failed to transmit READ_SR1_COMMAND\r\n");
    return APP_NOT_OK;
  }

  uint8_t response = 0;
  if (HAL_SPI_Receive(&peripherals->hspi_flash, &response, sizeof(response),
                      MAX_BLOCK_TIME_MS) != HAL_OK) {
    end_spi_cmd();
    printf("Failed to receive response to READ_SR1_COMMAND\r\n");
    return APP_NOT_OK;
  }

  end_spi_cmd();

  if ((response & SR1_WEL) == 0) {
    printf("Failed to set WRITE_ENABLE flag\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

void start_spi_cmd(void) {
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_RESET);
}

void end_spi_cmd(void) {
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);
  taskEXIT_CRITICAL();
}

void dump_flash(uint32_t start_address, uint32_t size) {
  if (start_address + size > SPI_FLASH_SIZE) {
    printf("Error: out of bounds\r\n");
    return;
  }

  const uint8_t row_length = 16;
  uint8_t read_buf[row_length];

  for (uint32_t addr = start_address; addr < start_address + size;
       addr += row_length) {
    int err = read(addr, read_buf, row_length);
    if (err != APP_OK) {
      printf("Failed to read, exiting.\r\n");
      error();
    }

    printf("%08x ", addr);
    for (uint8_t byte = 0; byte < row_length; byte++) {
      printf("%02x ", read_buf[byte]);
      if (byte == 7) {  // NOLINT
        printf(" ");
      }
    }
    printf(" |%.16s|\r\n", &read_buf[addr]);
  }
}

void test_spi_flash(void) {
  for (int i = 0; i < SPI_FLASH_SECTOR_COUNT; i++) {
    printf("Erasing sector %d...\r\n", i);
    int err = erase(i * SPI_FLASH_SECTOR_SIZE);
    if (err != APP_OK) {
      printf("Failed to erase, exiting.\r\n");
      error();
    }
  }

  uint8_t write_buf[SPI_FLASH_SECTOR_SIZE];
  for (int i = 0; i < SPI_FLASH_SECTOR_SIZE; i++) {
    write_buf[i] = 0xAA;  // NOLINT
  }

  for (int i = 0; i < SPI_FLASH_SECTOR_COUNT; i++) {
    printf("programming sector %d...\r\n", i);
    int err = program(i * SPI_FLASH_SECTOR_SIZE, write_buf, sizeof(write_buf));
    if (err != APP_OK) {
      printf("Failed to program, exiting.\r\n");
      error();
    }
  }

  uint8_t read_buf[SPI_FLASH_SECTOR_SIZE];
  for (int i = 0; i < SPI_FLASH_SECTOR_COUNT; i++) {
    memset(read_buf, 0, sizeof(read_buf));
    printf("reading sector %d...\r\n", i);
    int err = read(i * SPI_FLASH_SECTOR_SIZE, read_buf, sizeof(read_buf));
    if (err != APP_OK) {
      printf("Failed to read, exiting.\r\n");
      error();
    }
    if (memcmp(read_buf, write_buf, sizeof(write_buf)) != 0) {
      printf("Sector %d was not written properly. Expected: %u, got: %u\r\n", i,
             write_buf[i], read_buf[i]);
    }
  }
}
