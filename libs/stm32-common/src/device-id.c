#include "device-id.h"

#include "common-peripherals.h"
#include "error.h"
#include "lfs-config.h"
#include "lfs.h"
#include "rover.h"
#include "stm32f3xx_ll_utils.h"

static const char *ck_settings_dir = "/ck";
static const char *ck_settings_file = "/ck/settings";

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

int read_ck_id(ck_id_t *ck_id) {
  lfs_config_t lfs = get_lfs_config();
  lfs_file_t file;

  memset(ck_id, 0, sizeof(ck_id_t));

  int err = lfs_file_open(lfs.lfs, &file, ck_settings_file, LFS_O_RDONLY);
  if (err < 0) {
    if (err == LFS_ERR_NOENT) {
      printf("Couldn't find CK settings file.\r\n");
    } else {
      printf("lfs_file_open error: %d\r\n", err);
    }
    return err;
  }

  err = lfs_file_read(lfs.lfs, &file, ck_id, sizeof(ck_id_t));
  if (err < 0) {
    printf("Couldn't read CK settings file, error: %d", err);
    lfs_file_close(lfs.lfs, &file);
    return err;
  }

  err = lfs_file_close(lfs.lfs, &file);
  if (err < 0) {
    printf("lfs_file_close error: %d\r\n", err);
    return err;
  }

  return APP_OK;
}

int write_ck_id(const ck_id_t *ck_id) {
  lfs_config_t lfs = get_lfs_config();

  int err = lfs_mkdir(lfs.lfs, ck_settings_dir);
  if (err < 0 && err != LFS_ERR_EXIST) {
    printf("lfs_mkdir error: %d\r\n", err);
    return err;
  }

  lfs_file_t file;
  err = lfs_file_open(lfs.lfs, &file, ck_settings_file,
                      LFS_O_CREAT | LFS_O_WRONLY);
  if (err < 0) {
    printf("lfs_file_open error: %d\r\n", err);
    return err;
  }

  err = lfs_file_write(lfs.lfs, &file, ck_id, sizeof(ck_id_t));
  if (err < 0) {
    printf("lfs_file_write error: %d\r\n", err);
    return err;
  }

  err = lfs_file_close(lfs.lfs, &file);
  if (err < 0) {
    printf("lfs_file_close error: %d\r\n", err);
    return err;
  }

  return APP_OK;
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
