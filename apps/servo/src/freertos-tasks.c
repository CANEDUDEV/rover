#include "freertos-tasks.h"

#include <string.h>

#include "adc.h"
#include "app.h"
#include "ck-data.h"
#include "peripherals.h"

// STM32Common
#include "error.h"
#include "print.h"
#include "stm32f3xx_hal.h"

// CK
#include "mayor.h"
#include "types.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define PWM_TASK_PERIOD_MS 20
#define POWER_MEASURE_TASK_PERIOD_MS 100
#define TASK_PRIORITY 24

static TaskHandle_t pwm_task;
static StaticTask_t pwm_buffer;
static StackType_t pwm_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t power_measure_task;
static StaticTask_t power_measure_buffer;
static StackType_t power_measure_stack[configMINIMAL_STACK_SIZE];

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];

void process_letter(void *unused);
void pwm(void *unused);
void power_measure(void *unused);

void send_docs(void);
ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data);
void dispatch_letter(ck_letter_t *letter);

void task_init(void) {
  pwm_task = xTaskCreateStatic(pwm, "pwm", configMINIMAL_STACK_SIZE, NULL,
                               TASK_PRIORITY + 1, pwm_stack, &pwm_buffer);

  power_measure_task = xTaskCreateStatic(
      power_measure, "power measure", configMINIMAL_STACK_SIZE, NULL,
      TASK_PRIORITY, power_measure_stack, &power_measure_buffer);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      TASK_PRIORITY + 2, process_letter_stack, &process_letter_buf);
}

void pwm_timer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(pwm_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void power_measure_timer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(power_measure_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void power_measure(void *unused) {
  UNUSED(unused);

  peripherals_t *peripherals = get_peripherals();

  TimerHandle_t xTimer = xTimerCreate(
      "power measure timer", pdMS_TO_TICKS(POWER_MEASURE_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      power_measure_timer);

  xTimerStart(xTimer, portMAX_DELAY);

  uint16_t adc1Buf[3];
  uint16_t adc2Buf[2];
  ck_data_t *ck_data = get_ck_data();

  uint16_t sensor_power = 0;
  uint16_t servo_current = 0;
  uint16_t battery_voltage = 0;
  uint16_t servo_voltage = 0;
  uint16_t h_bridge_current = 0;

  for (;;) {
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

    send_docs();
  }
}

void pwm(void *unused) {
  UNUSED(unused);

  peripherals_t *peripherals = get_peripherals();

  TimerHandle_t xTimer =
      xTimerCreate("pwm timer", pdMS_TO_TICKS(PWM_TASK_PERIOD_MS),
                   pdTRUE,  // Auto reload timer
                   NULL,    // Timer ID, unused
                   pwm_timer);

  xTimerStart(xTimer, portMAX_DELAY);
  HAL_TIM_PWM_Start(&peripherals->htim1, TIM_CHANNEL_4);

  uint32_t pulse = PWM_MID_POS_PULSE;
  STEERING_DIRECTION direction = LEFT;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation
    UpdatePWMDutyCycle(&pulse, &direction);
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
  // ck_folder_t *folder = NULL;

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
  // TODO
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  // Only notify when the second ADC has finished
  if (hadc->Instance == ADC2) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(power_measure_task, &xHigherPriorityTaskWoken);
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
