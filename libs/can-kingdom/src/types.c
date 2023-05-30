#include "types.h"

ck_err_t ck_check_action_mode(ck_action_mode_t mode) {
  switch (mode) {
    case CK_ACTION_MODE_KEEP_CURRENT:
    case CK_ACTION_MODE_RUN:
    case CK_ACTION_MODE_FREEZE:
    case CK_ACTION_MODE_RESET:
      return CK_OK;
    default:
      return CK_ERR_INVALID_ACTION_MODE;
  }
}

ck_err_t ck_check_comm_mode(ck_comm_mode_t mode) {
  switch (mode) {
    case CK_COMM_MODE_KEEP_CURRENT:
    case CK_COMM_MODE_SILENT:
    case CK_COMM_MODE_LISTEN_ONLY:
    case CK_COMM_MODE_COMMUNICATE:
      return CK_OK;
    default:
      return CK_ERR_INVALID_COMM_MODE;
  }
}

ck_err_t ck_check_list_type(ck_list_type_t type) {
  switch (type) {
    case CK_LIST_BIT:
    case CK_LIST_LINE:
    case CK_LIST_PAGE:
    case CK_LIST_DOCUMENT:
      return CK_OK;
    default:
      return CK_ERR_INVALID_LIST_TYPE;
  }
}

ck_letter_t ck_default_letter(void) {
  ck_letter_t letter = {
      .envelope =
          {
              .envelope_no = CK_DEFAULT_LETTER_ENVELOPE,
          },
      .page =
          {
              .line_count = CK_MAX_LINES_PER_PAGE,
              // NOLINTNEXTLINE(*-magic-numbers)
              .lines = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA},
          },
  };
  return letter;
}
