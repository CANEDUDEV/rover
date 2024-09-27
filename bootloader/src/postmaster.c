#include "postmaster.h"

ck_err_t ck_save_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  (void)bit_timing;
  return CK_OK;
}

ck_err_t ck_load_bit_timing(ck_can_bit_timing_t *bit_timing) {
  *bit_timing = ck_default_bit_timing();
  return CK_OK;
}
