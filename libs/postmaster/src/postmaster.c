#include "postmaster.h"

#include <stdio.h>

#include "lfs-wrapper.h"

static const char *bit_timing_file = "/ck_bit_timing";

static ck_can_bit_timing_t bit_timing_storage;

ck_err_t ck_save_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  bit_timing_storage = *bit_timing;

  file_t file = {
      .name = bit_timing_file,
      .data = &bit_timing_storage,
      .size = sizeof(ck_can_bit_timing_t),
  };

  int err = write_file_async(&file);
  if (err < 0) {
    printf("Error: couldn't write CK bit timing file: %d\r\n", err);
    return CK_ERR_PERIPHERAL;
  }

  return CK_OK;
}

ck_err_t ck_load_bit_timing(ck_can_bit_timing_t *bit_timing) {
  file_t file = {
      .name = bit_timing_file,
      .data = bit_timing,
      .size = sizeof(ck_can_bit_timing_t),
  };

  int err = read_file(&file);
  if (err < 0) {
    printf("Error: couldn't read CK bit timing file: %d\r\n", err);
    return CK_ERR_PERIPHERAL;
  }

  return CK_OK;
}
