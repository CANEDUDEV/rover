#include "airbridge.h"

#include <stdio.h>

#include "common-peripherals.h"
#include "stm32f3xx_hal_can.h"

#define AIRBRIDGE_SBUS_CHANNEL 3

enum airbridge_status {
  DISCONNECTED,
  CONNECTED,
};

static enum airbridge_status status = DISCONNECTED;

void send_airbridge_connect(void);
bool switch_is_active(uint16_t switch_value);

void handle_airbridge_command(sbus_message_t *message) {
  if (switch_is_active(message->channels[AIRBRIDGE_SBUS_CHANNEL])) {
    send_airbridge_connect();
  }
}

void send_airbridge_connect(void) {
  if (status == CONNECTED) {
    return;
  }

  printf("connecting airbridges...\r\n");

  CAN_HandleTypeDef hcan = get_common_peripherals()->hcan;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < 1) {
    ;
  }

  // NOLINTBEGIN (*-magic-numbers)
  CAN_TxHeaderTypeDef header = {
      .DLC = 8,
      .StdId = 0x500,
      .IDE = CAN_ID_STD,
      .RTR = CAN_RTR_DATA,
  };
  uint8_t data[] = {0x07, 0x06, 0xCA, 0xD0, 0x00, 0x09, 0x01, 0x01};
  uint32_t mbox = 0;
  // NOLINTEND

  HAL_CAN_AddTxMessage(&hcan, &header, data, &mbox);

  status = CONNECTED;
}

void send_airbridge_disconnect(void) {
  printf("disconnecting airbridges...\r\n");

  CAN_HandleTypeDef hcan = get_common_peripherals()->hcan;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < 1) {
    ;
  }

  // NOLINTBEGIN (*-magic-numbers)
  CAN_TxHeaderTypeDef header = {
      .DLC = 8,
      .StdId = 0x500,
      .IDE = CAN_ID_STD,
      .RTR = CAN_RTR_DATA,
  };
  uint8_t data[] = {0x07, 0x06, 0xCA, 0xD0, 0x00, 0x00, 0x00, 0x00};
  uint32_t mbox = 0;
  // NOLINTEND

  HAL_CAN_AddTxMessage(&hcan, &header, data, &mbox);

  status = DISCONNECTED;
}

bool switch_is_active(uint16_t switch_value) {
  const uint16_t switch_active_min_sbus_value = 1600;
  return switch_value > switch_active_min_sbus_value;
}
