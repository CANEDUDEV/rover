#include "postmaster.h"

static ck_can_bit_timing_t current_bit_timing;
static ck_can_bit_timing_t saved_bit_timing;

ck_err_t ck_postmaster_init(void) {
  return CK_OK;
}

ck_err_t ck_send_letter(const ck_letter_t *letter) {
  (void)letter;
  return CK_OK;
}

ck_err_t ck_apply_comm_mode(ck_comm_mode_t mode) {
  if (ck_check_comm_mode(mode) != CK_OK) {
    return CK_ERR_INVALID_COMM_MODE;
  }
  return CK_OK;
}

uint8_t ck_get_125kbit_prescaler(void) {
  return 1;
}

ck_err_t ck_set_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  if (!bit_timing) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  current_bit_timing = *bit_timing;
  return CK_OK;
}

ck_err_t ck_save_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  if (!bit_timing) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  saved_bit_timing = *bit_timing;
  return CK_OK;
}

ck_err_t ck_load_bit_timing(ck_can_bit_timing_t *bit_timing) {
  if (!bit_timing) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  *bit_timing = saved_bit_timing;
  return CK_OK;
}
