#include "freertos-tasks.h"

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "ck-data.h"
#include "ck-rx-letters.h"
#include "peripherals.h"
#include "servo.h"

// STM32Common
#include "error.h"
#include "letter-reader.h"
#include "lfs-wrapper.h"

// CK
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define REPORT_DEFAULT_PERIOD_MS 200

#define MEASURE_STACK_SIZE \
  (sizeof(adc_samples_t) + 2 * configMINIMAL_STACK_SIZE)

task_periods_t task_periods = {
    .report_period_ms = REPORT_DEFAULT_PERIOD_MS,
};

static TaskHandle_t measure_task;
static StaticTask_t measure_buf;
static StackType_t measure_stack[MEASURE_STACK_SIZE];

static TaskHandle_t report_task;
static StaticTask_t report_buf;
static StackType_t report_stack[configMINIMAL_STACK_SIZE];

void measure(void *unused);
void report(void *unused);
void report_timer(TimerHandle_t timer);
void sample_adc(volatile adc_samples_t *adc_samples);

void send_docs(void);
int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

void task_init(void) {
  uint8_t priority = LOWEST_TASK_PRIORITY;

  if (init_lfs_task(priority++) < 0) {
    error();
  }

  measure_task = xTaskCreateStatic(measure, "measure", MEASURE_STACK_SIZE, NULL,
                                   priority++, measure_stack, &measure_buf);

  report_task = xTaskCreateStatic(report, "report", configMINIMAL_STACK_SIZE,
                                  NULL, priority++, report_stack, &report_buf);

  letter_reader_cfg_t letter_reader_cfg = {
      .priority = priority++,
      .app_letter_handler_func = handle_letter,
  };

  if (init_letter_reader_task(letter_reader_cfg) != APP_OK) {
    error();
  }
}

void set_task_periods(task_periods_t *periods) {
  if (periods->report_period_ms != 0) {
    task_periods.report_period_ms = periods->report_period_ms;
  }
}

void measure(void *unused) {
  (void)unused;

  ck_data_t *ck_data = get_ck_data();

  volatile adc_samples_t adc_samples;
  adc_reading_t adc_average;

  uint16_t battery_voltage = 0;
  uint16_t h_bridge_current = 0;

  servo_state_t *servo = get_servo_state();

  for (;;) {
    sample_adc(&adc_samples);
    adc_average_samples(&adc_average, &adc_samples);
    battery_voltage = adc_to_battery_voltage(adc_average.adc1_buf[2]);
    h_bridge_current = adc_to_h_bridge_current(adc_average.adc2_buf[1]);

    memcpy(ck_data->servo_position_page->lines, &servo->position,
           sizeof(servo->position));
    memcpy(ck_data->servo_current_page->lines, &servo->current,
           sizeof(servo->current));
    memcpy(ck_data->battery_voltage_page->lines, &battery_voltage,
           sizeof(battery_voltage));
    memcpy(ck_data->servo_voltage_page->lines, &servo->voltage,
           sizeof(servo->voltage));
    memcpy(ck_data->h_bridge_current_page->lines, &h_bridge_current,
           sizeof(h_bridge_current));

    if (ck_get_action_mode() != CK_ACTION_MODE_FREEZE) {
      update_servo_state(&adc_average);
    }
  }
}

void sample_adc(volatile adc_samples_t *adc_samples) {
  peripherals_t *peripherals = get_peripherals();

  HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adc_samples->adc1_buf,
                    ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS);
  HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adc_samples->adc2_buf,
                    ADC_NUM_SAMPLES * ADC2_NUM_CHANNELS);

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA
}

void report(void *unused) {
  (void)unused;

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic(
      "report timer", pdMS_TO_TICKS(task_periods.report_period_ms),
      pdFALSE,  // Don't auto reload timer
      NULL,     // Timer ID, unused
      report_timer, &timer_buf);

  for (;;) {
    xTimerChangePeriod(timer, task_periods.report_period_ms, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    if (ck_get_action_mode() != CK_ACTION_MODE_FREEZE) {
      send_docs();
    }
  }
}

void report_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(report_task);
}

void send_docs(void) {
  ck_data_t *ck_data = get_ck_data();

  if (ck_send_document(ck_data->servo_position_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->servo_current_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->battery_voltage_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->servo_voltage_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->h_bridge_current_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  if (ck_get_action_mode() == CK_ACTION_MODE_FREEZE) {
    return APP_OK;
  }

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
  if (folder->folder_no == ck_data->subtrim_folder->folder_no) {
    return process_subtrim_letter(letter);
  }
  if (folder->folder_no == ck_data->report_freq_folder->folder_no) {
    return process_report_freq_letter(letter);
  }
  if (folder->folder_no == ck_data->reverse_folder->folder_no) {
    return process_reverse_letter(letter);
  }
  if (folder->folder_no == ck_data->failsafe_folder->folder_no) {
    return process_failsafe_letter(letter);
  }
  return APP_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  // Avoid double notifications by notifying on the slower of the two ADCs
  if (hadc->Instance == ADC1) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(measure_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}
