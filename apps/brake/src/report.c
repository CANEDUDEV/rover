#include <stdio.h>
#include <string.h>

#include "ck-data.h"

// STM32Common
#include "error.h"

// CK
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

static TaskHandle_t report_task;
static StaticTask_t report_task_buf;
static StackType_t report_task_stack[configMINIMAL_STACK_SIZE];

#define DEFAULT_REPORT_PERIOD_MS 200
static uint16_t report_period_ms = DEFAULT_REPORT_PERIOD_MS;

void report(void *unused);

void init_report_task(uint8_t priority) {
  report_task =
      xTaskCreateStatic(report, "report", configMINIMAL_STACK_SIZE, NULL,
                        priority++, report_task_stack, &report_task_buf);
}

// 2 bytes in page
//
// bytes 0-1: reporting period in ms, i.e. how often to send
//            measurements over CAN.
int process_report_freq_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->set_report_freq_folder->dlc) {
    return APP_NOT_OK;
  }

  uint16_t new_period = 0;
  memcpy(&new_period, letter->page.lines, sizeof(new_period));
  if (new_period != 0) {
    report_period_ms = new_period;
  }

  return APP_OK;
}

void report_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(report_task);
}

void report(void *unused) {
  (void)unused;

  ck_data_t *ck_data = get_ck_data();

  StaticTimer_t timer_buf;

  TimerHandle_t timer =
      xTimerCreateStatic("report timer", pdMS_TO_TICKS(report_period_ms),
                         pdFALSE,  // Don't auto reload timer
                         NULL,     // Timer ID, unused
                         report_timer, &timer_buf);

  while (1) {
    xTimerChangePeriod(timer, report_period_ms, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (ck_get_action_mode() == CK_ACTION_MODE_FREEZE) {
      continue;
    }

    ck_err_t ret = ck_send_document(ck_data->wheel_speed_folder->folder_no);
    if (ret != CK_OK && ret != CK_ERR_TIMEOUT) {
      printf("error: failed to wheel speed doc\r\n");
    }
  }
}
