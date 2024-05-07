#include "freertos-tasks.h"

#include <stdio.h>
#include <string.h>

#include "airbridge.h"
#include "ck-data.h"
#include "rover.h"
#include "sbus.h"
#include "steering.h"

// CK
#include "mayor.h"

// STM32Common
#include "error.h"
#include "letter-reader.h"
#include "lfs-wrapper.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

static TaskHandle_t steering_task;
static StaticTask_t steering_task_buf;
static StackType_t steering_task_stack[2 * configMINIMAL_STACK_SIZE];

void sbus_read_and_steer(void *unused);
void send_steering_command(steering_command_t *command);

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

void task_init(void) {
  uint8_t priority = LOWEST_TASK_PRIORITY;

  if (init_lfs_task(priority++) < 0) {
    error();
  }

  steering_task = xTaskCreateStatic(
      sbus_read_and_steer, "sbus read", 2 * configMINIMAL_STACK_SIZE, NULL,
      priority++, steering_task_stack, &steering_task_buf);

  letter_reader_cfg_t letter_reader_cfg = {
      .priority = priority++,
      .app_letter_handler_func = handle_letter,
  };

  if (init_letter_reader_task(letter_reader_cfg) != APP_OK) {
    error();
  }

  init_king_task(priority++);
}

void sbus_read_and_steer(void *unused) {
  (void)unused;

  init_steering();

  send_airbridge_disconnect();

  for (;;) {
    sbus_message_t message;
    steering_command_t steering_command = neutral_steering_command();

    if (sbus_read_message(&message) != APP_OK) {
      send_steering_command(&steering_command);
      continue;
    }

    // Failsafe usually triggers if many frames are lost in a row.
    // Indicates heavy connection loss. Frame lost indicateds slight connection
    // loss or issue with frame.
    bool read_failed = message.failsafe_activated || message.frame_lost;

    if (read_failed) {
      printf("Frame lost, sending neutral command.\r\n");
    } else {
      steering_command = sbus_message_to_steering_command(&message);
    }

    send_steering_command(&steering_command);

    handle_airbridge_command(&message);
  }
}

void send_steering_command(steering_command_t *command) {
  if (!command->steering_is_on) {
    return;
  }

  ck_data_t *ck_data = get_ck_data();

  memcpy(&ck_data->steering_page->lines[1], &command->steering_angle,
         sizeof(command->steering_angle));
  memcpy(&ck_data->throttle_page->lines[1], &command->throttle,
         sizeof(command->throttle));

  if (ck_send_document(ck_data->steering_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->throttle_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  (void)folder;
  if (letter->envelope.envelope_no == AIRBRIDGE_DISCONNECT_ID) {
    printf("hello\r\n");
    send_airbridge_disconnect();
  }
  return APP_OK;
}
