#include "postmaster.h"

ck_err_t ck_send_letter(const ck_letter_t *letter, uint8_t dlc) {
  (void)letter;
  if (dlc > CK_CAN_MAX_DLC) {
    return CK_ERR_INVALID_CAN_DLC;
  }
  return CK_OK;
}
