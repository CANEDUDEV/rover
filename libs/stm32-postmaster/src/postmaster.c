#include "postmaster.h"

#include "error.h"
#include "flash.h"
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

ck_err_t ck_apply_comm_mode(ck_comm_mode_t mode) {
  ck_err_t ret = ck_check_comm_mode(mode);
  if (ret != CK_OK) {
    return ret;
  }

  CAN_HandleTypeDef *hcan = get_can_handle();
  switch (mode) {
    case CK_COMM_MODE_COMMUNICATE:
    case CK_COMM_MODE_LISTEN_ONLY:
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

uint8_t ck_get_125kbit_prescaler(void) {
  return 18;  // NOLINT
}

ck_err_t ck_set_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  CAN_HandleTypeDef *hcan = get_can_handle();

  // NOLINTBEGIN(*-magic-numbers)
  if (bit_timing->prescaler < 1 || bit_timing->prescaler > 32) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  hcan->Init.Prescaler = bit_timing->prescaler;

  uint8_t tseg1 = bit_timing->time_quanta - 1 - bit_timing->phase_seg2;

  switch (tseg1) {
    case 1:
      hcan->Init.TimeSeg1 = CAN_BS1_1TQ;
      break;
    case 2:
      hcan->Init.TimeSeg1 = CAN_BS1_2TQ;
      break;
    case 3:
      hcan->Init.TimeSeg1 = CAN_BS1_3TQ;
      break;
    case 4:
      hcan->Init.TimeSeg1 = CAN_BS1_4TQ;
      break;
    case 5:
      hcan->Init.TimeSeg1 = CAN_BS1_5TQ;
      break;
    case 6:
      hcan->Init.TimeSeg1 = CAN_BS1_6TQ;
      break;
    case 7:
      hcan->Init.TimeSeg1 = CAN_BS1_7TQ;
      break;
    case 8:
      hcan->Init.TimeSeg1 = CAN_BS1_8TQ;
      break;
    case 9:
      hcan->Init.TimeSeg1 = CAN_BS1_9TQ;
      break;
    case 10:
      hcan->Init.TimeSeg1 = CAN_BS1_10TQ;
      break;
    case 11:
      hcan->Init.TimeSeg1 = CAN_BS1_11TQ;
      break;
    case 12:
      hcan->Init.TimeSeg1 = CAN_BS1_12TQ;
      break;
    case 13:
      hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
      break;
    case 14:
      hcan->Init.TimeSeg1 = CAN_BS1_14TQ;
      break;
    case 15:
      hcan->Init.TimeSeg1 = CAN_BS1_15TQ;
      break;
    case 16:
      hcan->Init.TimeSeg1 = CAN_BS1_16TQ;
      break;
    default:
      return CK_ERR_INVALID_CAN_BIT_TIMING;
  }

  switch (bit_timing->phase_seg2) {
    case 1:
      hcan->Init.TimeSeg2 = CAN_BS2_1TQ;
      break;
    case 2:
      hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
      break;
    case 3:
      hcan->Init.TimeSeg2 = CAN_BS2_3TQ;
      break;
    case 4:
      hcan->Init.TimeSeg2 = CAN_BS2_4TQ;
      break;
    case 5:
      hcan->Init.TimeSeg2 = CAN_BS2_5TQ;
      break;
    case 6:
      hcan->Init.TimeSeg2 = CAN_BS2_6TQ;
      break;
    case 7:
      hcan->Init.TimeSeg2 = CAN_BS2_7TQ;
      break;
    case 8:
      hcan->Init.TimeSeg2 = CAN_BS2_8TQ;
      break;
    default:
      return CK_ERR_INVALID_CAN_BIT_TIMING;
  }

  switch (bit_timing->sjw) {
    case 1:
      hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
      break;
    case 2:
      hcan->Init.SyncJumpWidth = CAN_SJW_2TQ;
      break;
    case 3:
      hcan->Init.SyncJumpWidth = CAN_SJW_3TQ;
      break;
    case 4:
      hcan->Init.SyncJumpWidth = CAN_SJW_4TQ;
      break;
    default:
      return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  // NOLINTEND(*-magic-numbers)

  // Go off bus
  if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING) {
    if (HAL_CAN_Stop(hcan) != HAL_OK) {
      return CK_ERR_PERIPHERAL;
    }
  }

  if (HAL_CAN_Init(hcan) != HAL_OK) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }

  return CK_OK;
}

ck_err_t ck_save_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  if (flash_write(FLASH_RW_START, bit_timing, sizeof(ck_can_bit_timing_t)) !=
      APP_OK) {
    return CK_ERR_PERIPHERAL;
  }
  return CK_OK;
}

ck_err_t ck_load_bit_timing(ck_can_bit_timing_t *bit_timing) {
  if (flash_read(FLASH_RW_START, bit_timing, sizeof(ck_can_bit_timing_t)) !=
      APP_OK) {
    return CK_ERR_PERIPHERAL;
  }
  return CK_OK;
}
