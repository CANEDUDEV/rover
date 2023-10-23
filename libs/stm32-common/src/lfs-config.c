#include "lfs-config.h"

#include "error.h"
#include "lfs.h"
#include "spi-flash.h"

// Statically allocate buffers for littelfs
static uint8_t lfs_read_buf[SPI_FLASH_PAGE_SIZE];
static uint8_t lfs_prog_buf[SPI_FLASH_PAGE_SIZE];
static uint8_t lfs_lookahead_buf[SPI_FLASH_PAGE_SIZE];

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
    .block_size = SPI_FLASH_SECTOR_SIZE,  // Flash is sector-erasable
    .block_count = SPI_FLASH_SECTOR_COUNT,
    .block_cycles = 100,  // Optimize for wear leveling

    .cache_size = sizeof(lfs_read_buf),
    .read_buffer = lfs_read_buf,
    .prog_buffer = lfs_prog_buf,

    .lookahead_size = sizeof(lfs_lookahead_buf),
    .lookahead_buffer = lfs_lookahead_buf,
};

static lfs_t lfs = {0};
static bool mounted = false;

lfs_config_t get_lfs_config(void) {
  lfs_config_t cfg = {.cfg = &lfs_cfg, .lfs = &lfs};
  return cfg;
}

int lfs_init(void) {
  if (mounted) {
    return APP_OK;
  }

  int err = lfs_mount(&lfs, &lfs_cfg);

  if (err < 0) {  // Should only happen on first boot
    printf("lfs_mount error: %d\r\n", err);
    err = lfs_format(&lfs, &lfs_cfg);
    if (err < 0) {
      printf("lfs_format error: %d\r\n", err);
      error();
    }
    err = lfs_mount(&lfs, &lfs_cfg);
  }

  if (err >= 0) {
    mounted = true;
  }

  return err;
}

int lfs_deinit(void) {
  int err = lfs_unmount(&lfs);
  if (err < 0) {
    printf("lfs_unmount error: %d\r\n", err);
  }
  return err;
}
