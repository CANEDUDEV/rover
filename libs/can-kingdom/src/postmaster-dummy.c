#include "postmaster.h"

ck_err_t ck_send_letter(const ck_letter_t *letter, uint8_t dlc) {
  (void)letter;
  if (dlc > CK_CAN_MAX_DLC) {
    return CK_ERR_INVALID_CAN_DLC;
  }
  return CK_OK;
}

ck_err_t ck_set_comm_mode(ck_comm_mode_t mode) {
  if (ck_check_comm_mode(mode) != CK_OK) {
    return CK_ERR_INVALID_COMM_MODE;
  }
  return CK_OK;
}

ck_comm_mode_t ck_get_comm_mode(void) { return CK_COMM_MODE_COMMUNICATE; }
