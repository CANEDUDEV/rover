#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "clock.h"
#include "common-peripherals.h"
#include "rover.h"

#define STEERING_ID 2
#define THROTTLE_ID 4

void process_letter(void *unused);
void set_sbus_silent(void);
void handle_steering_id(const uint8_t *data);
void handle_throttle_id(const uint8_t *data);
void update_trim(int16_t *trim, uint8_t trim_signal);
uint16_t joystick_to_pwm(uint16_t signal, bool reverse);

const uint8_t trim_mask = 0xFC;
const uint16_t joystick_signal_mask = 0x03FF;

int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  common_peripherals_init();

  printf("Starting application\r\n");

  while (1) {
    process_letter(NULL);
  }
}

void process_letter(void *unused) {
  (void)unused;
  common_peripherals_t *peripherals = get_common_peripherals();
  HAL_CAN_Start(&peripherals->hcan);

  CAN_RxHeaderTypeDef header;
  uint8_t data[8];  // NOLINT

  for (;;) {
    if (HAL_CAN_GetRxMessage(&peripherals->hcan, CAN_RX_FIFO0, &header, data) !=
        HAL_OK) {
      continue;
    }

    set_sbus_silent();

    switch (header.StdId) {
      case STEERING_ID:
        handle_steering_id(data);
        break;
      case THROTTLE_ID:
        handle_throttle_id(data);
        break;
      default:
        break;
    }
  }
}

void set_sbus_silent(void) {
  common_peripherals_t *peripherals = get_common_peripherals();

  uint8_t set_sbus_silent_data[] = {4, 0, 0, 1, 0, 0, 0, 0};
  CAN_TxHeaderTypeDef set_sbus_silent_header = {
      .StdId = 0,
      .DLC = 8,
  };

  uint32_t mailbox = 0;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&peripherals->hcan) < 1) {
    // Wait
  }

  HAL_CAN_AddTxMessage(&peripherals->hcan, &set_sbus_silent_header,
                       set_sbus_silent_data, &mailbox);
}

void handle_steering_id(const uint8_t *data) {
  common_peripherals_t *peripherals = get_common_peripherals();

  static int16_t trim = 0;

  uint8_t trim_signal = data[1] & trim_mask;
  uint16_t joystick_signal = *(uint16_t *)data & joystick_signal_mask;

  update_trim(&trim, trim_signal);

  CAN_TxHeaderTypeDef steering_header = {
      .StdId = ROVER_STEERING_ENVELOPE,
      .DLC = 5,
  };

  uint8_t steering_data[5];
  steering_data[0] = 0;
  *(uint16_t *)&steering_data[1] =
      joystick_to_pwm(joystick_signal, false) + trim;

  uint32_t mailbox = 0;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&peripherals->hcan) < 1) {
    // Wait
  }

  HAL_CAN_AddTxMessage(&peripherals->hcan, &steering_header, steering_data,
                       &mailbox);
}

void update_trim(int16_t *trim, uint8_t trim_signal) {
  const uint8_t trim_decrease = 4;
  const uint8_t trim_increase = 8;
  const int16_t trim_min = -500;
  const int16_t trim_max = 500;

  if (trim_signal == trim_decrease && *trim > trim_min) {
    *trim -= 2;  // NOLINT
  } else if (trim_signal == trim_increase && *trim < trim_max) {
    *trim += 2;  // NOLINT
  }
}

uint16_t joystick_to_pwm(uint16_t signal, bool reverse) {
  const float factor = 480.0F / 512;
  const uint16_t neutral_signal = 512;
  const float neutral_pulse = 1500;
  const float max_pulse = 2000;

  float pulse = 0;

  if (signal < neutral_signal) {
    pulse = neutral_pulse - (float)signal * factor;
  } else if (signal > neutral_signal) {
    pulse = max_pulse - ((float)(signal) - (float)neutral_signal) * factor;
  }

  if (reverse) {
    pulse = neutral_pulse - (pulse - neutral_pulse);
  }

  return (uint16_t)roundf(pulse);
}

void handle_throttle_id(const uint8_t *data) {
  common_peripherals_t *peripherals = get_common_peripherals();

  static int16_t trim = 0;

  uint8_t trim_signal = data[1] & trim_mask;
  uint16_t joystick_signal = *(uint16_t *)data & joystick_signal_mask;

  update_trim(&trim, trim_signal);

  CAN_TxHeaderTypeDef throttle_header = {
      .StdId = ROVER_THROTTLE_ENVELOPE,
      .DLC = 5,
  };

  uint8_t throttle_data[5];
  throttle_data[0] = 0;
  *(uint16_t *)&throttle_data[1] =
      joystick_to_pwm(joystick_signal, true) + trim;

  uint32_t mailbox = 0;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&peripherals->hcan) < 2) {
    // Wait
  }

  HAL_CAN_AddTxMessage(&peripherals->hcan, &throttle_header, throttle_data,
                       &mailbox);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN) {
    can_msp_init();
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN) {
    can_msp_deinit();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uart1_msp_init();
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uart1_msp_deinit();
  }
}
