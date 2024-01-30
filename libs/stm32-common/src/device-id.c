#include "device-id.h"

#include <stdio.h>
#include <string.h>

#include "common-peripherals.h"
#include "error.h"
#include "lfs-wrapper.h"
#include "rover.h"
#include "stm32f3xx_ll_utils.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

static const char *ck_id_filename = "/ck_id";

static ck_id_t ck_id_storage;
static ck_id_t cached_ck_id;

ck_id_t get_default_ck_id(uint8_t city_address) {
  ck_id_t ck_id = {
      .city_address = city_address,
      .base_no = ROVER_BASE_NUMBER,
      .base_no_is_known = true,
      .base_no_has_extended_id = false,
  };
  memset(ck_id.group_addresses, 0, sizeof(ck_id.group_addresses));
  return ck_id;
}

ck_id_t *get_cached_ck_id(void) { return &cached_ck_id; }

int read_ck_id(ck_id_t *ck_id) {
  memset(ck_id, 0, sizeof(ck_id_t));

  file_t file = {
      .name = ck_id_filename,
      .data = ck_id,
      .size = sizeof(ck_id_t),
  };

  int err = read_file(&file);
  if (err < 0) {
    printf("Error: couldn't read file %s. error: %d\r\n", ck_id_filename, err);
    return err;
  }

  cached_ck_id = *ck_id;

  return APP_OK;
}

int write_ck_id(ck_id_t *ck_id) {
  ck_id_storage = *ck_id;  // Store it for async write

  file_t file = {
      .name = ck_id_filename,
      .data = &ck_id_storage,
      .size = sizeof(ck_id_t),
  };

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    return write_file_async(&file);
  }

  return write_file(&file);
}

uint32_t get_device_serial(void) {
  uint32_t uid[3] = {
      LL_GetUID_Word0(),
      LL_GetUID_Word1(),
      LL_GetUID_Word2(),
  };

  common_peripherals_t *common_peripherals = get_common_peripherals();
  return HAL_CRC_Calculate(&common_peripherals->hcrc, uid,
                           sizeof(uid) / sizeof(uint32_t));
}
