#include "freertos-tasks.h"

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "ck-data.h"
#include "ck-rx-letters.h"
#include "peripherals.h"
#include "potentiometer.h"

// STM32Common
#include "error.h"
#include "postmaster-hal.h"
#include "rover.h"
#include "stm32f3xx_hal.h"

// CK
#include "king.h"
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

static TaskHandle_t king_task;
static StaticTask_t king_buf;
static StackType_t king_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t measure_task;
static StaticTask_t measure_buf;
static StackType_t measure_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t report_task;
static StaticTask_t report_buf;
static StackType_t report_stack[configMINIMAL_STACK_SIZE];

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];

void king(void *unused);
// King helpers
void king_timer_callback(TimerHandle_t timer);
void send_default_letter(void);
void send_base_number(void);
void assign_envelopes(void);
void start_communication(void);

void measure(void *unused);
void report(void *unused);
void process_letter(void *unused);
void measure_timer(TimerHandle_t timer);
void report_timer(TimerHandle_t timer);

void send_docs(void);
void dispatch_letter(ck_letter_t *letter);
int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

void task_init(void) {
  report_task =
      xTaskCreateStatic(report, "report", configMINIMAL_STACK_SIZE, NULL,
                        LOWEST_TASK_PRIORITY, report_stack, &report_buf);

  measure_task =
      xTaskCreateStatic(measure, "measure", configMINIMAL_STACK_SIZE, NULL,
                        LOWEST_TASK_PRIORITY + 1, measure_stack, &measure_buf);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 2, process_letter_stack, &process_letter_buf);

  king_task =
      xTaskCreateStatic(king, "king", configMINIMAL_STACK_SIZE, NULL,
                        LOWEST_TASK_PRIORITY + 3, king_stack, &king_buf);
}

void set_task_periods(task_periods_t *periods) {
  if (periods->measure_period_ms != 0) {
    task_periods.measure_period_ms = periods->measure_period_ms;
  }
  if (periods->report_period_ms != 0) {
    task_periods.report_period_ms = periods->report_period_ms;
  }
}

void king(void *unused) {
  (void)unused;

  // Only the steering servo should act as king
#ifdef MOTOR
  vTaskSuspend(king_task);
#endif

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic("king timer", pdMS_TO_TICKS(150),
                                           pdFALSE,  // Don't auto reload timer
                                           NULL,     // Timer ID, unused
                                           king_timer_callback, &timer_buf);
  xTimerStart(timer, portMAX_DELAY);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for timer

  // There is already traffic on the CAN bus, so don't act as king.
  // TODO: how to handle traffic that is not detected due to incorrect bitrate?
  if (ck_get_comm_mode() == CK_COMM_MODE_LISTEN_ONLY) {
    printf("Someone else is king. Suspending king task.\r\n");
    vTaskSuspend(king_task);
    // Task will never be resumed
  }

  // Spoof a default letter reception
  if (ck_default_letter_received() != CK_OK) {
    printf("Error receiving fake default letter.\r\n");
    error();
  }

  // Send default letter several times to make sure it is received.
  for (uint8_t i = 0; i < 5; i++) {  // NOLINT(*-magic-numbers)
    send_default_letter();
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  send_base_number();
  vTaskDelay(pdMS_TO_TICKS(50));  // Give cities time to respond

  assign_envelopes();

  // Set reverse direction
  // Since the steering servo acts as king, set it to reverse.
  // This makes the motor not reversed but the steering reversed.
  ck_letter_t letter = {.page.line_count = 0};
  process_reverse_letter(&letter);

  start_communication();

  // Start up our own communications
  if (ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) != CK_OK) {
    printf("Error starting king.\r\n");
    error();
  }

  vTaskSuspend(king_task);
}

void king_timer_callback(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(king_task);
}

void send_default_letter(void) {
  peripherals_t *peripherals = get_peripherals();
  ck_letter_t default_letter = ck_default_letter();
  CAN_TxHeaderTypeDef can_header = {
      .IDE = CAN_ID_STD,
      .StdId = default_letter.envelope.envelope_no,
      .DLC = default_letter.page.line_count,
  };
  uint32_t mailbox = 0;

  while (HAL_CAN_GetTxMailboxesFreeLevel(
             &peripherals->common_peripherals->hcan) < 1) {
    // Busy wait for CAN frame to be sent
  }
  HAL_CAN_AddTxMessage(&peripherals->common_peripherals->hcan, &can_header,
                       default_letter.page.lines, &mailbox);
}

void send_base_number(void) {
  peripherals_t *peripherals = get_peripherals();

  ck_page_t page;

  ck_kp1_args_t kp1_args = {
      .address = 0,
      .base_no = ROVER_BASE_NUMBER,
      .has_extended_id = false,
      .mayor_response_no = 0,
  };
  if (ck_create_kings_page_1(&kp1_args, &page) != CK_OK) {
    printf("Error creating king's page.\r\n");
    error();
  }

  CAN_TxHeaderTypeDef can_header = {
      .IDE = CAN_ID_STD,
      .StdId = 0,
      .DLC = page.line_count,
  };

  uint32_t mailbox = 0;

  while (HAL_CAN_GetTxMailboxesFreeLevel(
             &peripherals->common_peripherals->hcan) < 1) {
    // Busy wait for CAN frame to be sent
  }
  HAL_CAN_AddTxMessage(&peripherals->common_peripherals->hcan, &can_header,
                       page.lines, &mailbox);

  // Also "send" KP1 to ourselves
  ck_letter_t letter;
  letter.page = page;
  letter.envelope.envelope_no = ROVER_BASE_NUMBER;
  letter.envelope.has_extended_id = false;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("Error receiving our own king's page.\r\n");
    error();
  }
}

void assign_servo_envelopes(void) {
  ck_data_t *ck_data = get_ck_data();
  ck_data->steering_folder->envelope_count = 1;
  ck_data->steering_folder->envelopes[0].envelope_no = ROVER_STEERING_ENVELOPE;
  ck_data->steering_folder->envelopes[0].enable = true;

  ck_data->steering_trim_folder->envelope_count = 1;
  ck_data->steering_trim_folder->envelopes[0].envelope_no =
      ROVER_STEERING_TRIM_ENVELOPE;
  ck_data->steering_trim_folder->envelopes[0].enable = true;

  ck_data->servo_current_folder->envelope_count = 1;
  ck_data->servo_current_folder->envelopes[0].envelope_no =
      ROVER_SERVO_CURRENT_ENVELOPE;
  ck_data->servo_current_folder->envelopes[0].enable = true;

  ck_data->servo_voltage_folder->envelope_count = 1;
  ck_data->servo_voltage_folder->envelopes[0].envelope_no =
      ROVER_SERVO_VOLTAGE_ENVELOPE;
  ck_data->servo_voltage_folder->envelopes[0].enable = true;

  ck_data->set_servo_voltage_folder->envelope_count = 1;
  ck_data->set_servo_voltage_folder->envelopes[0].envelope_no =
      ROVER_SERVO_SET_VOLTAGE_ENVELOPE;
  ck_data->set_servo_voltage_folder->envelopes[0].enable = true;

  ck_data->pwm_conf_folder->envelope_count = 1;
  ck_data->pwm_conf_folder->envelopes[0].envelope_no =
      ROVER_SERVO_PWM_CONFIG_ENVELOPE;
  ck_data->pwm_conf_folder->envelopes[0].enable = true;

  ck_data->report_freq_folder->envelope_count = 1;
  ck_data->report_freq_folder->envelopes[0].envelope_no =
      ROVER_SERVO_REPORT_FREQUENCY_ENVELOPE;
  ck_data->report_freq_folder->envelopes[0].enable = true;

  ck_data->reverse_folder->envelope_count = 1;
  ck_data->reverse_folder->envelopes[0].envelope_no =
      ROVER_SERVO_REVERSE_ENVELOPE;
  ck_data->reverse_folder->envelopes[0].enable = true;

  ck_data->failsafe_folder->envelope_count = 1;
  ck_data->failsafe_folder->envelopes[0].envelope_no =
      ROVER_SERVO_FAILSAFE_ENVELOPE;
  ck_data->failsafe_folder->envelopes[0].enable = true;

  ck_data->servo_position_folder->envelope_count = 1;
  ck_data->servo_position_folder->envelopes[0].envelope_no =
      ROVER_SERVO_POSITION_ENVELOPE;
  ck_data->servo_position_folder->envelopes[0].enable = true;
}

void assign_envelopes(void) {
  peripherals_t *peripherals = get_peripherals();
  rover_kingdom_t *kingdom = get_rover_kingdom();
  uint32_t mailbox = 0;
  ck_page_t page;

  // Asssign envelopes for others
  for (uint8_t i = 0; i < kingdom->assignment_count; i++) {
    ck_kp2_args_t kp2_args = {
        .address = kingdom->assignments[i].city,
        .envelope.enable = true,
        .envelope.envelope_no = kingdom->assignments[i].envelope,
        .folder_no = kingdom->assignments[i].folder,
        .envelope_action = CK_ENVELOPE_ASSIGN,
    };

    if (ck_create_kings_page_2(&kp2_args, &page) != CK_OK) {
      printf("Error creating king's page.\r\n");
      error();
    }

    CAN_TxHeaderTypeDef can_header = {
        .IDE = CAN_ID_STD,
        .StdId = 0,
        .DLC = page.line_count,
    };

    while (HAL_CAN_GetTxMailboxesFreeLevel(
               &peripherals->common_peripherals->hcan) < 1) {
      // Busy wait for CAN frame to be sent
    }
    HAL_CAN_AddTxMessage(&peripherals->common_peripherals->hcan, &can_header,
                         page.lines, &mailbox);
  }

  // Finally, assign our own envelopes
  assign_servo_envelopes();
}

void start_communication(void) {
  peripherals_t *peripherals = get_peripherals();
  ck_page_t page;
  ck_kp0_args_t kp0_args = {
      .address = 0,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
      .action_mode = CK_ACTION_MODE_KEEP_CURRENT,
      .comm_mode = CK_COMM_MODE_COMMUNICATE,
  };

  if (ck_create_kings_page_0(&kp0_args, &page) != CK_OK) {
    printf("Error creating king's page.\r\n");
    error();
  }

  CAN_TxHeaderTypeDef can_header = {
      .IDE = CAN_ID_STD,
      .StdId = 0,
      .DLC = page.line_count,
  };

  uint32_t mailbox = 0;

  while (HAL_CAN_GetTxMailboxesFreeLevel(
             &peripherals->common_peripherals->hcan) < 1) {
    // Busy wait for CAN frame to be sent
  }
  HAL_CAN_AddTxMessage(&peripherals->common_peripherals->hcan, &can_header,
                       page.lines, &mailbox);
}

void measure(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic(
      "measure timer", pdMS_TO_TICKS(task_periods.measure_period_ms),
      pdFALSE,  // Don't auto reload timer
      NULL,     // Timer ID, unused
      measure_timer, &timer_buf);

  uint16_t adc1_buf[3];
  uint16_t adc2_buf[2];
  ck_data_t *ck_data = get_ck_data();

  int16_t servo_position = 0;
  uint16_t servo_current = 0;
  uint16_t battery_voltage = 0;
  uint16_t servo_voltage = 0;
  uint16_t h_bridge_current = 0;

  for (;;) {
    xTimerChangePeriod(timer, task_periods.measure_period_ms, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adc1_buf,
                      sizeof(adc1_buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adc2_buf,
                      sizeof(adc2_buf) / sizeof(uint16_t));

    // Wait for DMA
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    servo_position = adc_to_servo_position(adc1_buf[0]);
    servo_current = adc_to_servo_current(adc1_buf[1]);
    battery_voltage = adc_to_battery_voltage(adc1_buf[2]);
    servo_voltage = adc_to_servo_voltage(adc2_buf[0]);
    h_bridge_current = adc_to_h_bridge_current(adc2_buf[1]);
    memcpy(ck_data->servo_position_page->lines, &servo_position,
           sizeof(servo_position));
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

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic(
      "report timer", pdMS_TO_TICKS(task_periods.report_period_ms),
      pdFALSE,  // Don't auto reload timer
      NULL,     // Timer ID, unused
      report_timer, &timer_buf);

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

  if (folder->folder_no == ck_data->set_servo_voltage_folder->folder_no) {
    return process_set_servo_voltage_letter(letter);
  }
  if (folder->folder_no == ck_data->pwm_conf_folder->folder_no) {
    return process_pwm_conf_letter(letter);
  }
  if (folder->folder_no == ck_data->steering_folder->folder_no) {
    return process_steering_letter(letter);
  }
  if (folder->folder_no == ck_data->steering_trim_folder->folder_no) {
    return process_steering_trim_letter(letter);
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
  (void)hadc;
  // Only notify when the second ADC has finished
  if (hadc->Instance == ADC2) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(measure_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
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
