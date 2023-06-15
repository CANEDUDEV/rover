#include <string.h>

#include "adc.h"
#include "battery.h"
#include "ck-data.h"
#include "led.h"
#include "peripherals.h"
#include "ports.h"
#include "potentiometer.h"

// STM32Common
#include "error.h"
#include "print.h"

// CK
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define BATTERY_MONITOR_TASK_PERIOD_MS 20
#define BATTERY_REPORT_TASK_PERIOD_MS 200
#define LOWEST_TASK_PRIORITY 24

static TaskHandle_t battery_monitor_task;
static StaticTask_t battery_monitor_buf;
static StackType_t battery_monitor_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t battery_report_task;
static StaticTask_t battery_report_buf;
static StackType_t battery_report_stack[configMINIMAL_STACK_SIZE];

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];

void battery_monitor(void *unused);
void battery_report(void *unused);
void process_letter(void *unused);
void battery_monitor_timer(TimerHandle_t timer);
void battery_report_timer(TimerHandle_t timer);
void update_pages(void);
void send_docs(void);
ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data);
void dispatch_letter(ck_letter_t *letter);

void task_init(void) {
  battery_monitor_task = xTaskCreateStatic(
      battery_monitor, "battery monitor", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, battery_monitor_stack, &battery_monitor_buf);

  battery_report_task = xTaskCreateStatic(
      battery_report, "battery report", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY, battery_report_stack, &battery_report_buf);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 2, process_letter_stack, &process_letter_buf);
}

void battery_monitor(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  set_jumper_config(ALL_ON);
  configure_potentiometer(&peripherals->hi2c1, POTENTIOMETER_IVRA_DEFAULT);
  HAL_GPIO_WritePin(REG_PWR_ON_GPIO_PORT, REG_PWR_ON_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT, POWER_OFF_PIN, GPIO_PIN_SET);

  battery_state_init();

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic(
      "battery monitor timer", pdMS_TO_TICKS(BATTERY_MONITOR_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      battery_monitor_timer, &timer_buf);

  xTimerStart(timer, portMAX_DELAY);

  battery_state_t *battery_state = get_battery_state();
  adc_reading_t adc_reading;

  // TODO: implement set_fuse_config() to configure HW fuses,
  // use it to set over_current_threshold.
  //
  // TODO: calculate the maximum measured reg_out_current based on the jumper
  // config, use it to set a better over_current_threshold that considers the
  // reg_out_current separately from the vbat_out_current.
  //
  // We might detect false positives if the jumper config is set to detect low
  // currents but the actual currents are much higher. Nevertheless, the largest
  // part of the total current is the vbat_out_current.
  const uint32_t over_current_threshold = 49500;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adc_reading.adc1_buf,
                      sizeof(adc_reading.adc1_buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adc_reading.adc2_buf,
                      sizeof(adc_reading.adc2_buf) / sizeof(uint16_t));

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    update_battery_state(&adc_reading);

    // Check if over current or low voltage protection has triggered.
    if (battery_state->charge == LOW_VOLTAGE_CUTOFF ||
        battery_state->over_current_fault ||
        battery_state->reg_out_current + battery_state->vbat_out_current >
            over_current_threshold) {
      // Turn off the power outputs to reduce the battery power drain.
      HAL_GPIO_WritePin(REG_PWR_ON_GPIO_PORT, REG_PWR_ON_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT, POWER_OFF_PIN, GPIO_PIN_RESET);
      // Blink LEDs red to show user something is wrong.
      blink_leds_red();
    }

    update_pages();
  }
}

void battery_report(void *unused) {
  (void)unused;

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic(
      "battery report timer", pdMS_TO_TICKS(BATTERY_REPORT_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      battery_report_timer, &timer_buf);

  xTimerStart(timer, portMAX_DELAY);

  for (;;) {
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

  memcpy(ck_data->reg_out_current_page->lines, &battery_state->reg_out_current,
         ck_data->reg_out_current_folder->dlc);

  memcpy(ck_data->vbat_out_current_page->lines,
         &battery_state->vbat_out_current,
         ck_data->vbat_out_current_folder->dlc);
}

void send_docs(void) {
  ck_data_t *ck_data = get_ck_data();

  if (ck_send_document(ck_data->cell_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->reg_out_current_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->vbat_out_current_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
}

// Convert HAL CAN frame to ck_letter_t.
ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data) {
  ck_letter_t letter;
  letter.envelope.is_remote = header->RTR;
  letter.envelope.has_extended_id = header->IDE;
  if (letter.envelope.has_extended_id) {
    letter.envelope.envelope_no = header->ExtId;
  } else {
    letter.envelope.envelope_no = header->StdId;
  }
  letter.envelope.is_compressed = false;
  letter.page.line_count = header->DLC;
  memcpy(letter.page.lines, data, header->DLC);
  return letter;
}

void dispatch_letter(ck_letter_t *letter) {
  peripherals_t *peripherals = get_peripherals();
  // Check for default letter
  if (ck_is_default_letter(letter) == CK_OK) {
    if (ck_default_letter_received() != CK_OK) {
      print("CAN Kingdom error in ck_default_letter_received().\r\n");
    }
  }
  // Check for king's letter
  if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      print("failed to process king's letter.\r\n");
    }
  }
  // TODO: implement other letters
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
