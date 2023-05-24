#include "postmaster.h"

#include "postmaster-hal.h"

ck_err_t ck_send_letter(const ck_letter_t *letter, uint8_t dlc) {
  CAN_HandleTypeDef *hcan = get_can_handle();

  // If bus off, return error
  if (hcan->State != HAL_CAN_STATE_READY &&
      hcan->State != HAL_CAN_STATE_LISTENING) {
    return CK_ERR_SEND_FAILED;
  }

  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 1) {
    // Busy loop, wait for mailbox to be free
  }

  CAN_TxHeaderTypeDef header = {
      .DLC = dlc,
      .RTR = CAN_RTR_DATA,
      .IDE = CAN_ID_STD,
      .StdId = letter->envelope.envelope_no,
  };

  if (letter->envelope.is_remote) {
    header.RTR = CAN_RTR_REMOTE;
  }
  if (letter->envelope.has_extended_id) {
    header.IDE = CAN_ID_EXT;
    header.ExtId = letter->envelope.envelope_no;
  }

  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(hcan, &header, letter->page.lines, &mailbox) !=
      HAL_OK) {
    return CK_ERR_SEND_FAILED;
  }
  return CK_OK;
}

ck_err_t ck_set_comm_mode(ck_comm_mode_t mode) {
  ck_err_t ret = ck_check_comm_mode(mode);
  if (ret != CK_OK) {
    return ret;
  }

  CAN_HandleTypeDef *hcan = get_can_handle();
  switch (mode) {
    case CK_COMM_MODE_COMMUNICATE:
    case CK_COMM_MODE_LISTEN_ONLY:
      set_comm_mode(mode);

      if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING) {
        if (HAL_CAN_Stop(hcan) != HAL_OK) {
          return CK_ERR_SET_MODE_FAILED;
        }
      }

      hcan->Init.Mode = CAN_MODE_NORMAL;
      if (HAL_CAN_Init(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }

      if (HAL_CAN_Start(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }

      break;

    case CK_COMM_MODE_SILENT:
      set_comm_mode(mode);

      if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING) {
        if (HAL_CAN_Stop(hcan) != HAL_OK) {
          return CK_ERR_SET_MODE_FAILED;
        }
      }

      hcan->Init.Mode = CAN_MODE_SILENT;
      if (HAL_CAN_Init(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }

      if (HAL_CAN_Start(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }

      break;

    default:
      break;
  }
  return CK_OK;
}
