#include "lfs-config.h"

#include "error.h"
#include "lfs.h"
#include "spi-flash.h"

// Statically allocate buffers for littelfs
static uint8_t lfs_read_buf[SPI_FLASH_SECTOR_SIZE];
static uint8_t lfs_prog_buf[SPI_FLASH_SECTOR_SIZE];
static uint8_t lfs_lookahead_buf[SPI_FLASH_SECTOR_SIZE];

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
    .read_size = SPI_FLASH_SECTOR_SIZE,
    .prog_size = SPI_FLASH_SECTOR_SIZE,
    .block_size = SPI_FLASH_SECTOR_SIZE,
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

static void format_and_mount(void) {
  int err = lfs_format(&lfs, &lfs_cfg);
  if (err < 0) {
    printf("Fatal flash error, couldn't format flash: %d\r\n", err);
    error();
  }
  err = lfs_mount(&lfs, &lfs_cfg);
  if (err < 0) {
    printf("Fatal flash error, couldn't mount after format: %d\r\n", err);
    error();
  }
}

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
    printf("Formatting flash...\r\n");
    format_and_mount();
  }

  lfs_dir_t dir;
  err = lfs_dir_open(&lfs, &dir, "/");

  if (err == LFS_ERR_CORRUPT) {
    printf("Corrupt filesystem, reformatting flash...\r\n");
    format_and_mount();
  }

  if (err == LFS_ERR_OK) {
    err = lfs_dir_close(&lfs, &dir);
    if (err < 0) {
      printf("Fatal flash error, couldn't close root directory: %d\r\n", err);
      error();
    }
  }

  mounted = true;

  return APP_OK;
}

int lfs_deinit(void) {
  int err = lfs_unmount(&lfs);
  if (err < 0) {
    printf("lfs_unmount error: %d\r\n", err);
  }
  mounted = false;
  return err;
}
