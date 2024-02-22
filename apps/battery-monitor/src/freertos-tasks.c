#include "freertos-tasks.h"

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "battery.h"
#include "ck-data.h"
#include "ck-rx-letters.h"
#include "jumpers.h"
#include "peripherals.h"
#include "potentiometer.h"
#include "power.h"

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

#define BATTERY_REPORT_DEFAULT_PERIOD_MS 200

#define BATTERY_MONITOR_STACK_SIZE \
  (sizeof(adc_samples_t) + configMINIMAL_STACK_SIZE)

static task_periods_t task_periods = {
    .battery_report_period_ms = BATTERY_REPORT_DEFAULT_PERIOD_MS,
};

static TaskHandle_t battery_monitor_task;
static StaticTask_t battery_monitor_buf;
static StackType_t battery_monitor_stack[BATTERY_MONITOR_STACK_SIZE];

static TaskHandle_t battery_report_task;
static StaticTask_t battery_report_buf;
static StackType_t battery_report_stack[configMINIMAL_STACK_SIZE];

void battery_monitor(void *unused);
void battery_report(void *unused);
void battery_report_timer(TimerHandle_t timer);
void sample_adc(volatile adc_samples_t *adc_samples);
void update_pages(void);
void send_docs(void);
int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

void task_init(void) {
  uint8_t priority = LOWEST_TASK_PRIORITY;

  if (init_lfs_task(priority++) < 0) {
    error();
  }

  battery_report_task = xTaskCreateStatic(
      battery_report, "battery report", configMINIMAL_STACK_SIZE, NULL,
      priority++, battery_report_stack, &battery_report_buf);

  battery_monitor_task = xTaskCreateStatic(
      battery_monitor, "battery monitor", BATTERY_MONITOR_STACK_SIZE, NULL,
      priority++, battery_monitor_stack, &battery_monitor_buf);

  letter_reader_cfg_t letter_reader_cfg = {
      .priority = priority++,
      .app_letter_handler_func = handle_letter,
  };

  if (init_letter_reader_task(letter_reader_cfg) != APP_OK) {
    error();
  }
}

void set_task_periods(task_periods_t *periods) {
  if (periods->battery_report_period_ms != 0) {
    task_periods.battery_report_period_ms = periods->battery_report_period_ms;
  }
}

void battery_monitor(void *unused) {
  (void)unused;

  set_current_measure_jumper_config(X11_ON_X12_ON);
  if (configure_potentiometer(0) != APP_OK) {
    error();
  }

  battery_state_init();

  set_reg_vout_power_on();
  set_vbat_power_on();

  volatile adc_samples_t adc_samples;
  adc_reading_t adc_average;

  for (;;) {
    sample_adc(&adc_samples);
    adc_average_samples(&adc_average, &adc_samples);
    update_battery_state(&adc_average);
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
    update_pages();
    send_docs();
  }
}

void battery_report_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(battery_report_task);
}

void update_pages(void) {
  ck_data_t *ck_data = get_ck_data();
  battery_state_t *battery_state = get_battery_state();

  // Copy cells 0,1,2 to page 0 of cell doc, cells 3,4,5 to page 1.
  memcpy(&ck_data->cell_page0->lines[1], &battery_state->cell_voltage[0],
         ck_data->cell_folder->dlc);
  memcpy(&ck_data->cell_page1->lines[1], &battery_state->cell_voltage[3],
         ck_data->cell_folder->dlc);

  memcpy(ck_data->reg_out_page->lines, &battery_state->reg_out_voltage,
         sizeof(uint16_t));
  memcpy(&ck_data->reg_out_page->lines[2], &battery_state->reg_out_current,
         sizeof(uint16_t));

  memcpy(ck_data->vbat_out_page->lines, &battery_state->vbat_out_voltage,
         sizeof(uint16_t));
  memcpy(&ck_data->vbat_out_page->lines[2], &battery_state->vbat_out_current,
         sizeof(uint32_t));
}

void send_docs(void) {
  ck_data_t *ck_data = get_ck_data();

  if (ck_send_document(ck_data->cell_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->reg_out_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->vbat_out_folder->folder_no) != CK_OK) {
    printf("failed to send doc.\r\n");
  }
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();

  if (folder->folder_no == ck_data->jumper_config_folder->folder_no) {
    return process_jumper_config_letter(letter);
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
  // Avoid double notifications by notifying on the slower of the two ADCs
  if (hadc->Instance == ADC2) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(battery_monitor_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}
