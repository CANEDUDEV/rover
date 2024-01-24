#include "sbus.h"

#include <stdio.h>
#include <string.h>

#include "error.h"
#include "peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define SBUS_HEADER 0x0F
#define SBUS_PACKET_LENGTH 25
#define SBUS_PACKET_DATA_LENGTH (SBUS_PACKET_LENGTH - 2)

typedef struct {
  uint8_t header;
  uint8_t data[SBUS_PACKET_DATA_LENGTH];
  uint8_t footer;
} sbus_packet_t;

static const uint16_t sbus_timeout_ms = 10;
static TaskHandle_t sbus_task;

static int sbus_read_header(uint8_t *header);
static int sbus_read_data(uint8_t *sbus_data);
static int sbus_read_footer(uint8_t *footer);
static void sbus_parse_packet(const sbus_packet_t *packet,
                              sbus_message_t *message);

static int read_uart_stream(uint8_t *dest, size_t size);

void sbus_init(TaskHandle_t task_to_notify) { sbus_task = task_to_notify; }

int sbus_read_message(sbus_message_t *message) {
  sbus_task = xTaskGetCurrentTaskHandle();

  sbus_packet_t packet;

  int err = sbus_read_header(&packet.header);
  if (err != APP_OK) {
    return err;
  }

  err = sbus_read_data(packet.data);
  if (err != APP_OK) {
    return err;
  }

  err = sbus_read_footer(&packet.footer);
  if (err != APP_OK) {
    return err;
  }

  memset(message, 0, sizeof(sbus_message_t));
  sbus_parse_packet(&packet, message);

  return APP_OK;
}

// Try to read a header in a loop. Return error on timeout or bad UART read.
static int sbus_read_header(uint8_t *header) {
  int err = read_uart_stream(header, sizeof(uint8_t));
  if (err != APP_OK) {
    // Don't print anything here since it's a common failure mode and will
    // result in a lot of spam.
    return err;
  }

  if (*header != SBUS_HEADER) {
    // We're in the middle of a packet, so need to flush UART register
    printf("Error: invalid SBUS header.\r\n");
    peripherals_t *peripherals = get_peripherals();
    __HAL_UART_FLUSH_DRREGISTER(&peripherals->huart2);
    return APP_NOT_OK;
  }

  return APP_OK;
}

// Try to read an sbus packet.
static int sbus_read_data(uint8_t *sbus_data) {
  int err = read_uart_stream(sbus_data, SBUS_PACKET_DATA_LENGTH);
  if (err != APP_OK) {
    printf("Error: failed to read SBUS data: %d.\r\n", err);
  }
  return err;
}

// Try to read a header in a loop. Return error on timeout or bad UART read.
static int sbus_read_footer(uint8_t *footer) {
  int err = read_uart_stream(footer, sizeof(uint8_t));
  if (err != APP_OK) {
    printf("Error: failed to read SBUS footer: %d.\r\n", err);
  }
  return err;
}

/*
 * The SBUS packet is 25 bytes long consisting of:
 *
 * Byte[0]: SBUS header, 0x0F
 * Byte[1-22]: 16 servo channels, 11 bits each
 * Byte[23]
 *     Bit 0: channel 17 (0x01)
 *     Bit 1: channel 18 (0x02)
 *     Bit 2: frame lost (0x04)
 *     Bit 3: failsafe activated (0x08)
 * Byte[24]: SBUS footer
 *
 */
static void sbus_parse_packet(const sbus_packet_t *packet,
                              sbus_message_t *message) {
  const uint8_t *data = packet->data;

  // NOLINTBEGIN(*-magic-numbers)
  message->channels[0] = data[0] | ((data[1] << 8) & 0x07FF);
  message->channels[1] = (data[1] >> 3) | ((data[2] << 5) & 0x07FF);
  message->channels[2] =
      (data[2] >> 6) | (data[3] << 2) | ((data[4] << 10) & 0x07FF);

  message->channels[3] = (data[4] >> 1) | ((data[5] << 7) & 0x07FF);
  message->channels[4] = (data[5] >> 4) | ((data[6] << 4) & 0x07FF);
  message->channels[5] =
      (data[6] >> 7) | (data[7] << 1) | ((data[8] << 9) & 0x07FF);

  message->channels[6] = (data[8] >> 2) | ((data[9] << 6) & 0x07FF);
  message->channels[7] = (data[9] >> 5) | ((data[10] << 3) & 0x07FF);
  message->channels[8] = data[11] | ((data[12] << 8) & 0x07FF);
  message->channels[9] = (data[12] >> 3) | ((data[13] << 5) & 0x07FF);
  message->channels[10] =
      (data[13] >> 6) | (data[14] << 2) | ((data[15] << 10) & 0x07FF);

  message->channels[11] = (data[15] >> 1) | ((data[16] << 7) & 0x07FF);
  message->channels[12] = (data[16] >> 4) | ((data[17] << 4) & 0x07FF);
  message->channels[13] =
      (data[17] >> 7) | (data[18] << 1) | ((data[19] << 9) & 0x07FF);

  message->channels[14] = (data[19] >> 2) | ((data[20] << 6) & 0x07FF);
  message->channels[15] = (data[20] >> 5) | ((data[21] << 3) & 0x07FF);

  message->channels[16] = data[22] & 0x01;
  message->channels[17] = data[22] & 0x02;
  message->frame_lost = data[22] & 0x04;
  message->failsafe_activated = data[22] & 0x08;
  // NOLINTEND(*-magic-numbers)
}

static int read_uart_stream(uint8_t *dest, size_t size) {
  peripherals_t *peripherals = get_peripherals();

  HAL_UART_Receive_IT(&peripherals->huart2, dest, size);

  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(sbus_timeout_ms)) != pdPASS) {
    // Timeout
    return APP_NOT_OK;
  }

  uint32_t err = HAL_UART_GetError(&peripherals->huart2);
  if (err != HAL_UART_ERROR_NONE) {
    printf("UART error in SBUS header: 0x%x\r\n", err);
    __HAL_UART_FLUSH_DRREGISTER(&peripherals->huart2);
    return APP_NOT_OK;
  }

  return APP_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART2) {
    return;
  }
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(sbus_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
