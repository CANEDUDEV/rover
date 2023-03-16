#include "app.h"

#include <string.h>

#include "main.h"

// When mounting a switch vertically, SWITCH_PRESS_LEFT should correspond to
// pressing the switch up, and SWITCH_PRESS_RIGHT should correspond to pressing
// the switch down.
enum SWITCH_STATE {
  SWITCH_NEUTRAL = 0U,
  SWITCH_PRESS_LEFT = 1U,
  SWITCH_PRESS_RIGHT = 2U,
};

#define ANALOG_PORT_MESSAGE_ID 100U
#define ANALOG_PORT_MESSAGE_DLC 4U

#define SWITCH_MESSAGE_ID 200U
#define SWITCH_MESSAGE_DLC 4U

// Handles defined in main.c by STM32CubeMX
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern CAN_HandleTypeDef hcan;

void SendAnalogPortMessage(const uint16_t *data) {
  CAN_TxHeaderTypeDef header = {
      .StdId = ANALOG_PORT_MESSAGE_ID,
      .DLC = ANALOG_PORT_MESSAGE_DLC,
  };
  uint32_t mailbox = CAN_TX_MAILBOX0;
  uint8_t canData[4];
  for (int i = 0; i < ANALOG_PORT_MESSAGE_DLC; i++) {
    canData[i] = data[i];
  }
  HAL_CAN_AddTxMessage(&hcan, &header, canData, &mailbox);
}

void SendSwitchPortMessage(const uint8_t *data) {
  CAN_TxHeaderTypeDef header = {
      .StdId = SWITCH_MESSAGE_ID,
      .DLC = SWITCH_MESSAGE_DLC,
  };
  uint32_t mailbox = CAN_TX_MAILBOX0;
  HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}

enum SWITCH_STATE getSwitchState(GPIO_PinState pin1, GPIO_PinState pin2) {
  // pin1 = 0 && pin2 = 1 => left
  // pin1 = 1 && pin2 = 0 => right
  // pin1 = 1 && pin2 = 1 => neutral
  if (!pin1 && pin2) {
    return SWITCH_PRESS_LEFT;
  }
  if (pin1 && !pin2) {
    return SWITCH_PRESS_RIGHT;
  }
  return SWITCH_NEUTRAL;
}

void ReadSwitches(uint8_t *data) {
  GPIO_PinState sw1p1 =
      HAL_GPIO_ReadPin(SWITCH1_PIN1_GPIO_Port, SWITCH1_PIN1_Pin);
  GPIO_PinState sw1p2 =
      HAL_GPIO_ReadPin(SWITCH1_PIN2_GPIO_Port, SWITCH1_PIN2_Pin);

  GPIO_PinState sw2p1 =
      HAL_GPIO_ReadPin(SWITCH2_PIN1_GPIO_Port, SWITCH2_PIN1_Pin);
  GPIO_PinState sw2p2 =
      HAL_GPIO_ReadPin(SWITCH2_PIN2_GPIO_Port, SWITCH2_PIN2_Pin);

  GPIO_PinState sw3p1 =
      HAL_GPIO_ReadPin(SWITCH3_PIN1_GPIO_Port, SWITCH3_PIN1_Pin);
  GPIO_PinState sw3p2 =
      HAL_GPIO_ReadPin(SWITCH3_PIN2_GPIO_Port, SWITCH3_PIN2_Pin);

  GPIO_PinState sw4p1 =
      HAL_GPIO_ReadPin(SWITCH4_PIN1_GPIO_Port, SWITCH4_PIN1_Pin);
  GPIO_PinState sw4p2 =
      HAL_GPIO_ReadPin(SWITCH4_PIN2_GPIO_Port, SWITCH4_PIN2_Pin);

  data[0] = getSwitchState(sw1p1, sw1p2);
  data[1] = getSwitchState(sw2p1, sw2p2);
  data[2] = getSwitchState(sw3p1, sw3p2);
  data[3] = getSwitchState(sw4p1, sw4p2);
}
