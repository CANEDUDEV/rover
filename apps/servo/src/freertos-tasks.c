#include "freertos-tasks.h"

#include <string.h>

#include "adc.h"
#include "ck-data.h"
#include "ck-rx-letters.h"
#include "peripherals.h"
#include "potentiometer.h"

// STM32Common
#include "error.h"
#include "postmaster-hal.h"
#include "print.h"
#include "stm32f3xx_hal.h"

// CK
#include "mayor.h"
#include "types.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define MEASURE_DEFAULT_PERIOD_MS 20
#define REPORT_DEFAULT_PERIOD_MS 100
#define LOWEST_TASK_PRIORITY 24

task_periods_t task_periods = {
    .measure_period_ms = MEASURE_DEFAULT_PERIOD_MS,
    .report_period_ms = REPORT_DEFAULT_PERIOD_MS,
};

static TaskHandle_t measure_task;
static StaticTask_t measure_buffer;
static StackType_t measure_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t report_task;
static StaticTask_t report_buffer;
static StackType_t report_stack[configMINIMAL_STACK_SIZE];

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];

void measure(void *unused);
void report(void *unused);
void process_letter(void *unused);
void measure_timer(TimerHandle_t timer);
void report_timer(TimerHandle_t timer);

void send_docs(void);
void dispatch_letter(ck_letter_t *letter);
int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

void task_init(void) {
  measure_task = xTaskCreateStatic(measure, "measure", configMINIMAL_STACK_SIZE,
                                   NULL, LOWEST_TASK_PRIORITY + 1,
                                   measure_stack, &measure_buffer);

  report_task =
      xTaskCreateStatic(report, "report", configMINIMAL_STACK_SIZE, NULL,
                        LOWEST_TASK_PRIORITY, report_stack, &report_buffer);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 2, process_letter_stack, &process_letter_buf);
}

void set_task_periods(task_periods_t *periods) {
  if (periods->measure_period_ms != 0) {
    task_periods.measure_period_ms = periods->measure_period_ms;
  }
  if (periods->report_period_ms != 0) {
    task_periods.report_period_ms = periods->report_period_ms;
  }
}

void measure(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  TimerHandle_t timer = xTimerCreate(
      "measure timer", pdMS_TO_TICKS(task_periods.measure_period_ms),
      pdFALSE,  // Don't auto reload timer
      NULL,     // Timer ID, unused
      measure_timer);

  uint16_t adc1Buf[3];
  uint16_t adc2Buf[2];
  ck_data_t *ck_data = get_ck_data();

  uint16_t sensor_power = 0;
  uint16_t servo_current = 0;
  uint16_t battery_voltage = 0;
  uint16_t servo_voltage = 0;
  uint16_t h_bridge_current = 0;

  for (;;) {
    xTimerChangePeriod(timer, task_periods.measure_period_ms, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adc1Buf,
                      sizeof(adc1Buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adc2Buf,
                      sizeof(adc2Buf) / sizeof(uint16_t));

    // Wait for DMA
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    sensor_power = adc_to_sensor_power(adc1Buf[0]);
    servo_current = adc_to_servo_current(adc1Buf[1]);
    battery_voltage = adc_to_battery_voltage(adc1Buf[2]);
    servo_voltage = adc_to_servo_voltage(adc2Buf[0]);
    h_bridge_current = adc_to_h_bridge_current(adc2Buf[1]);
    memcpy(ck_data->sensor_power_page->lines, &sensor_power,
           sizeof(sensor_power));
    memcpy(ck_data->servo_current_page->lines, &servo_current,
           sizeof(servo_current));
    memcpy(ck_data->battery_voltage_page->lines, &battery_voltage,
           sizeof(battery_voltage));
    memcpy(ck_data->servo_voltage_page->lines, &servo_voltage,
           sizeof(servo_voltage));
    memcpy(ck_data->h_bridge_current_page->lines, &h_bridge_current,
           sizeof(h_bridge_current));
  }
}

void report(void *unused) {
  (void)unused;

  TimerHandle_t timer =
      xTimerCreate("report timer", pdMS_TO_TICKS(task_periods.report_period_ms),
                   pdFALSE,  // Don't auto reload timer
                   NULL,     // Timer ID, unused
                   report_timer);

  for (;;) {
    xTimerChangePeriod(timer, task_periods.report_period_ms, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation
    send_docs();
  }
}

void process_letter(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  CAN_RxHeaderTypeDef header;
  uint8_t data[CK_CAN_MAX_DLC];
  ck_letter_t letter;

  for (;;) {
    if (HAL_CAN_ActivateNotification(&peripherals->common_peripherals->hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      print("Error activating interrupt.\r\n");
      error();
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Process all messages
    while (HAL_CAN_GetRxMessage(&peripherals->common_peripherals->hcan,
                                CAN_RX_FIFO0, &header, data) == HAL_OK) {
      if (ck_correct_letter_received() != CK_OK) {
        print("CAN Kingdom error in ck_correct_letter_received().\r\n");
      }
      letter = frame_to_letter(&header, data);
      dispatch_letter(&letter);
    }
  }
}

void measure_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(measure_task);
}

void report_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(report_task);
}

void send_docs(void) {
  ck_data_t *ck_data = get_ck_data();

  if (ck_send_document(ck_data->sensor_power_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->servo_current_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->battery_voltage_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->servo_voltage_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->h_bridge_current_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
}

void dispatch_letter(ck_letter_t *letter) {
  ck_folder_t *folder = NULL;

  // Check for default letter
  if (ck_is_default_letter(letter) == CK_OK) {
    if (ck_default_letter_received() != CK_OK) {
      print("CAN Kingdom error in ck_default_letter_received().\r\n");
    }
  }
  // Check for king's letter
  else if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      print("failed to process king's letter.\r\n");
    }
  }
  // Check for any other letter
  else if (ck_get_envelopes_folder(&letter->envelope, &folder) == CK_OK) {
    if (handle_letter(folder, letter) != APP_OK) {
      print("failed to process page.\r\n");
    }
  }
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();

  if (folder->folder_no == ck_data->set_servo_voltage_folder->folder_no) {
    return process_set_servo_voltage_letter(letter);
  }
  if (folder->folder_no == ck_data->pwm_conf_folder->folder_no) {
    return process_pwm_conf_letter(letter);
  }
  if (folder->folder_no == ck_data->steering_folder->folder_no) {
    return process_steering_letter(letter);
  }
  if (folder->folder_no == ck_data->report_freq_folder->folder_no) {
    return process_report_freq_letter(letter);
  }
  return APP_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  // Only notify when the second ADC has finished
  if (hadc->Instance == ADC2) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(measure_task, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// Deactivate interrupt, then signal task. Let the task reactivate the
// interrupt.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(process_letter_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
