#include "freertos-tasks.h"

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "battery.h"
#include "ck-data.h"
#include "ck-rx-letters.h"
#include "led.h"
#include "peripherals.h"
#include "ports.h"
#include "potentiometer.h"

// STM32Common
#include "error.h"
#include "postmaster-hal.h"
#include "spi-flash.h"

// CK
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define BATTERY_MONITOR_DEFAULT_PERIOD_MS 20
#define BATTERY_REPORT_DEFAULT_PERIOD_MS 200
#define LOWEST_TASK_PRIORITY 24

static task_periods_t task_periods = {
    .battery_monitor_period_ms = BATTERY_MONITOR_DEFAULT_PERIOD_MS,
    .battery_report_period_ms = BATTERY_REPORT_DEFAULT_PERIOD_MS,
};

static TaskHandle_t battery_monitor_task;
static StaticTask_t battery_monitor_buf;
static StackType_t battery_monitor_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t battery_report_task;
static StaticTask_t battery_report_buf;
static StackType_t battery_report_stack[configMINIMAL_STACK_SIZE];

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
// Need at least one page of stack for interacting with the SPI flash.
static StackType_t
    process_letter_stack[SPI_FLASH_PAGE_SIZE + configMINIMAL_STACK_SIZE];

void battery_monitor(void *unused);
void battery_report(void *unused);
void process_letter(void *unused);
void battery_monitor_timer(TimerHandle_t timer);
void battery_report_timer(TimerHandle_t timer);
void update_pages(void);
void send_docs(void);
void dispatch_letter(ck_letter_t *letter);
int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

void task_init(void) {
  battery_monitor_task = xTaskCreateStatic(
      battery_monitor, "battery monitor", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, battery_monitor_stack, &battery_monitor_buf);

  battery_report_task = xTaskCreateStatic(
      battery_report, "battery report", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY, battery_report_stack, &battery_report_buf);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter",
      SPI_FLASH_PAGE_SIZE + configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 2, process_letter_stack, &process_letter_buf);
}

void set_task_periods(task_periods_t *periods) {
  if (periods->battery_monitor_period_ms != 0) {
    task_periods.battery_monitor_period_ms = periods->battery_monitor_period_ms;
  }
  if (periods->battery_report_period_ms != 0) {
    task_periods.battery_report_period_ms = periods->battery_report_period_ms;
  }
}

void battery_monitor(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  set_jumper_config(ALL_ON);
  configure_potentiometer(POTENTIOMETER_IVRA_DEFAULT);

  battery_state_init();
  set_fuse_config(FUSE_50_AMPERE);

  HAL_GPIO_WritePin(nREG_POWER_ON_GPIO_PORT, nREG_POWER_ON_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(nPOWER_OFF_GPIO_PORT, nPOWER_OFF_PIN, GPIO_PIN_SET);

  StaticTimer_t timer_buf;
  TimerHandle_t timer =
      xTimerCreateStatic("battery monitor timer",
                         pdMS_TO_TICKS(task_periods.battery_monitor_period_ms),
                         pdFALSE,  // Don't auto reload timer
                         NULL,     // Timer ID, unused
                         battery_monitor_timer, &timer_buf);

  adc_reading_t adc_reading;

  for (;;) {
    // This starts the timer with the set period
    xTimerChangePeriod(timer,
                       pdMS_TO_TICKS(task_periods.battery_monitor_period_ms),
                       portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adc_reading.adc1_buf,
                      sizeof(adc_reading.adc1_buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adc_reading.adc2_buf,
                      sizeof(adc_reading.adc2_buf) / sizeof(uint16_t));

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    update_battery_state(&adc_reading);

    update_pages();
  }
}

void battery_report(void *unused) {
  (void)unused;

  StaticTimer_t timer_buf;
  TimerHandle_t timer =
      xTimerCreateStatic("battery report timer",
                         pdMS_TO_TICKS(task_periods.battery_report_period_ms),
                         pdFALSE,  // Don't auto reload timer
                         NULL,     // Timer ID, unused
                         battery_report_timer, &timer_buf);

  for (;;) {
    // This starts the timer with the set period
    xTimerChangePeriod(timer,
                       pdMS_TO_TICKS(task_periods.battery_report_period_ms),
                       portMAX_DELAY);
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
      printf("Error activating interrupt.\r\n");
      error();
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Process all messages
    while (HAL_CAN_GetRxMessage(&peripherals->common_peripherals->hcan,
                                CAN_RX_FIFO0, &header, data) == HAL_OK) {
      if (ck_correct_letter_received() != CK_OK) {
        printf("CAN Kingdom error in ck_correct_letter_received().\r\n");
      }
      letter = frame_to_letter(&header, data);
      dispatch_letter(&letter);
    }
  }
}

void battery_monitor_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(battery_monitor_task);
}

void battery_report_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(battery_report_task);
}

void update_pages(void) {
  ck_data_t *ck_data = get_ck_data();
  battery_state_t *battery_state = get_battery_state();

  // Copy cells 0,1,2 to page 0 of cell doc, cells 3,4,5 to page 1.
  memcpy(&ck_data->cell_page0->lines[1], &battery_state->cells[0],
         ck_data->cell_folder->dlc);
  memcpy(&ck_data->cell_page1->lines[1], &battery_state->cells[3],
         ck_data->cell_folder->dlc);

  memcpy(ck_data->reg_out_page->lines, &battery_state->reg_out_voltage,
         sizeof(uint16_t));
  memcpy(&ck_data->reg_out_page->lines[2], &battery_state->reg_out_current,
         sizeof(uint16_t));

  memcpy(ck_data->vbat_out_current_page->lines,
         &battery_state->vbat_out_current,
         ck_data->vbat_out_current_folder->dlc);
}

void send_docs(void) {
  ck_data_t *ck_data = get_ck_data();

  if (ck_send_document(ck_data->cell_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->reg_out_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->vbat_out_current_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
}

void dispatch_letter(ck_letter_t *letter) {
  ck_folder_t *folder = NULL;

  // Check for default letter
  if (ck_is_default_letter(letter) == CK_OK) {
    if (ck_default_letter_received() != CK_OK) {
      printf("CAN Kingdom error in ck_default_letter_received().\r\n");
    }
  }
  // Check for king's letter
  else if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      printf("failed to process king's letter.\r\n");
    }
  }
  // Check for any other letter
  else if (ck_get_envelopes_folder(&letter->envelope, &folder) == CK_OK) {
    if (handle_letter(folder, letter) != APP_OK) {
      printf("failed to process page.\r\n");
    }
  }
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();

  if (folder->folder_no == ck_data->jumper_and_fuse_conf_folder->folder_no) {
    return process_jumper_and_fuse_conf_letter(letter);
  }
  if (folder->folder_no == ck_data->set_reg_out_voltage_folder->folder_no) {
    return process_set_reg_out_voltage_letter(letter);
  }
  if (folder->folder_no == ck_data->output_on_off_folder->folder_no) {
    return process_output_on_off_letter(letter);
  }
  if (folder->folder_no == ck_data->report_freq_folder->folder_no) {
    return process_report_freq_letter(letter);
  }
  if (folder->folder_no == ck_data->low_voltage_cutoff_folder->folder_no) {
    return process_low_voltage_cutoff_letter(letter);
  }
  return APP_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(battery_monitor_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

// Deactivate interrupt, then signal task. Let the task reactivate the
// interrupt.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(process_letter_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
