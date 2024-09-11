#include "lfs-wrapper.h"

#include <stdio.h>
#include <string.h>

#include "error.h"
#include "lfs.h"
#include "spi-flash.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define WRITE_QUEUE_LENGTH 10
#define WRITE_QUEUE_ITEM_SIZE sizeof(file_t)

// Statically allocate buffers for littelfs
static uint8_t lfs_read_buf[SPI_FLASH_SECTOR_SIZE];
static uint8_t lfs_prog_buf[SPI_FLASH_SECTOR_SIZE];
static uint8_t lfs_lookahead_buf[SPI_FLASH_SECTOR_SIZE];
static lfs_t lfs = {0};
static bool lfs_is_mounted = false;

static QueueHandle_t write_queue;
static StaticQueue_t static_write_queue;
static uint8_t write_queue_storage[WRITE_QUEUE_LENGTH * WRITE_QUEUE_ITEM_SIZE];

static StaticTask_t write_file_task_buf;

// Need at least one page of stack for interacting with the SPI flash.
static StackType_t
    write_file_stack[SPI_FLASH_PAGE_SIZE + configMINIMAL_STACK_SIZE];

SemaphoreHandle_t mutex = NULL;
StaticSemaphore_t mutex_buf;

static void write_file_task(void *unused);

// Functions required by littlefs
int spi_flash_read(const struct lfs_config *config, lfs_block_t block,
                   lfs_off_t offset, void *buffer, lfs_size_t size);
int spi_flash_prog(const struct lfs_config *config, lfs_block_t block,
                   lfs_off_t offset, const void *buffer, lfs_size_t size);
int spi_flash_erase(const struct lfs_config *config, lfs_block_t block);
int spi_flash_sync(const struct lfs_config *config);

// Helpers
static void format_and_mount(void);
static void lfs_lock(void);
static void lfs_unlock(void);

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

    // Last sector reserved for SPI flash init
    .block_count = SPI_FLASH_SECTOR_COUNT - 1,
    .block_cycles = 100,  // Optimize for wear leveling

    .cache_size = sizeof(lfs_read_buf),
    .read_buffer = lfs_read_buf,
    .prog_buffer = lfs_prog_buf,

    .lookahead_size = sizeof(lfs_lookahead_buf),
    .lookahead_buffer = lfs_lookahead_buf,
};

void lfs_init(void) {
  if (lfs_is_mounted) {
    return;
  }

  if (spi_flash_workaround_init() != APP_OK) {
    printf("error: couldn't init SPI flash.");
    error();
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

  lfs_is_mounted = true;
}

void lfs_deinit(void) {
  int err = lfs_unmount(&lfs);
  if (err < 0) {
    printf("lfs_unmount error: %d\r\n", err);
    error();
  }

  lfs_is_mounted = false;
}

int init_lfs_task(uint32_t write_priority) {
  if (write_priority == 0) {
    printf("Error: failed to init lfs task, priority: %d\r\n.", write_priority);
    return APP_NOT_OK;
  }

  write_queue = xQueueCreateStatic(WRITE_QUEUE_LENGTH, WRITE_QUEUE_ITEM_SIZE,
                                   write_queue_storage, &static_write_queue);

  mutex = xSemaphoreCreateMutexStatic(&mutex_buf);

  xTaskCreateStatic(write_file_task, "file writer",
                    SPI_FLASH_PAGE_SIZE + configMINIMAL_STACK_SIZE, NULL,
                    write_priority, write_file_stack, &write_file_task_buf);
  lfs_init();

  return APP_OK;
}

static void write_file_task(void *unused) {
  (void)unused;

  for (;;) {
    file_t file;
    if (xQueueReceive(write_queue, &file, portMAX_DELAY) != pdPASS) {
      printf("Error: couldn't retrieve file from queue.\r\n");
      continue;
    }

    write_file(&file);
  }
}

int write_file(const file_t *file) {
  if (!lfs_is_mounted) {
    printf("Error: filesystem not initialized.\r\n");
    return APP_NOT_OK;
  }

  lfs_lock();

  lfs_file_t lfs_file;
  int err =
      lfs_file_open(&lfs, &lfs_file, file->name, LFS_O_CREAT | LFS_O_WRONLY);

  if (err < 0) {
    printf("lfs_file_open error: %d\r\n", err);
    lfs_unlock();
    return err;
  }

  err = lfs_file_write(&lfs, &lfs_file, file->data, file->size);
  if (err < 0) {
    printf("lfs_file_write error: %d\r\n", err);
    lfs_file_close(&lfs, &lfs_file);
    lfs_unlock();
    return err;
  }

  err = lfs_file_close(&lfs, &lfs_file);
  if (err < 0) {
    printf("lfs_file_close error: %d\r\n", err);
    lfs_unlock();
    return err;
  }

  lfs_unlock();
  return APP_OK;
}

int read_file(file_t *file) {
  if (!lfs_is_mounted) {
    printf("Error: filesystem not initialized.\r\n");
    return APP_NOT_OK;
  }

  lfs_lock();

  lfs_file_t lfs_file;
  int err = lfs_file_open(&lfs, &lfs_file, file->name, LFS_O_RDONLY);
  if (err < 0) {
    if (err == LFS_ERR_NOENT) {
      printf("Couldn't find file: %s.\r\n", file->name);
    } else {
      printf("lfs_file_open error: %d\r\n", err);
    }
    lfs_unlock();
    return err;
  }

  err = lfs_file_read(&lfs, &lfs_file, file->data, file->size);
  if (err < 0) {
    printf("lfs_file_read error: %d", err);
    lfs_file_close(&lfs, &lfs_file);
    lfs_unlock();
    return err;
  }

  err = lfs_file_close(&lfs, &lfs_file);
  if (err < 0) {
    printf("lfs_file_close error: %d\r\n", err);
    lfs_unlock();
    return err;
  }

  lfs_unlock();
  return APP_OK;
}

int write_file_async(const file_t *file) {
  if (!lfs_is_mounted) {
    printf("Error: filesystem not initialized.\r\n");
    return APP_NOT_OK;
  }

  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
    printf(
        "Error: can only use async write when FreeRTOS scheduler is "
        "running.\r\n");
    return APP_NOT_OK;
  }

  if (xQueueSend(write_queue, file, portMAX_DELAY) != pdPASS) {
    printf("Error: couldn't enqueue file.\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

int spi_flash_read(const struct lfs_config *config, lfs_block_t block,
                   lfs_off_t offset, void *buffer, lfs_size_t size) {
  return read(block * config->block_size + offset, buffer, size);
}

int spi_flash_prog(const struct lfs_config *config, lfs_block_t block,
                   lfs_off_t offset, const void *buffer, lfs_size_t size) {
  return program(block * config->block_size + offset, (uint8_t *)buffer, size);
}

int spi_flash_erase(const struct lfs_config *config, lfs_block_t block) {
  return erase(block * config->block_size);
}

int spi_flash_sync(const struct lfs_config *config) {
  (void)config;
  return APP_OK;
}

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

static void lfs_lock(void) {
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    xSemaphoreTake(mutex, portMAX_DELAY);
  }
}

static void lfs_unlock(void) {
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    xSemaphoreGive(mutex);
  }
}
