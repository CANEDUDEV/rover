#include <stdio.h>
#include <string.h>

#include "app.h"
#include "clock.h"
#include "error.h"
#include "flash.h"
#include "peripherals.h"
#include "print.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

// Hardware
static peripherals_t *peripherals;

// FreeRTOS
#define PWM_TASK_PERIOD_MS 20
#define POWER_MEASURE_TASK_PERIOD_MS 100
#define TASK_PRIORITY 24

#define CAN_MESSAGE_QUEUE_LENGTH 10

QueueHandle_t can_message_queue;

static TaskHandle_t pwm_task;
static StaticTask_t pwm_buffer;
static StackType_t pwm_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t can_tx_task;
static StaticTask_t can_tx_buffer;
static StackType_t can_tx_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t power_measure_task;
static StaticTask_t power_measure_buffer;
static StackType_t power_measure_stack[configMINIMAL_STACK_SIZE];

void task_init(void);
void pwm(void *argument);
void can_tx(void *argument);
void power_measure(void *argument);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  peripherals_init();
  peripherals = get_peripherals();

  InitPotentiometers();

  task_init();

  can_message_queue = xQueueCreate(CAN_MESSAGE_QUEUE_LENGTH, sizeof(CANFrame));

  print(&peripherals->common_peripherals->huart1,
        "Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void task_init(void) {
  pwm_task = xTaskCreateStatic(pwm, "pwm", configMINIMAL_STACK_SIZE, NULL,
                               TASK_PRIORITY, pwm_stack, &pwm_buffer);

  can_tx_task =
      xTaskCreateStatic(can_tx, "can tx", configMINIMAL_STACK_SIZE, NULL,
                        TASK_PRIORITY, can_tx_stack, &can_tx_buffer);

  power_measure_task = xTaskCreateStatic(
      power_measure, "power measure", configMINIMAL_STACK_SIZE, NULL,
      TASK_PRIORITY, power_measure_stack, &power_measure_buffer);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(power_measure_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

void mailbox_free_callback(CAN_HandleTypeDef *_hcan) {
  // Only notify when going from 0 free mailboxes to 1 free
  if (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) <= 1) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(can_tx_task, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailbox_free_callback(_hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailbox_free_callback(_hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailbox_free_callback(_hcan);
}

void can_tx(void *argument) {
  UNUSED(argument);

  HAL_CAN_ActivateNotification(&peripherals->common_peripherals->hcan,
                               CAN_IT_TX_MAILBOX_EMPTY);
  HAL_CAN_Start(&peripherals->common_peripherals->hcan);

  uint32_t mailbox = 0;

  for (;;) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(
            &peripherals->common_peripherals->hcan) == 0) {
      // Wait for mailbox to become available
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    CANFrame frame;
    // Wait for queue to receive message
    xQueueReceive(can_message_queue, &frame, portMAX_DELAY);

    CAN_TxHeaderTypeDef header = {
        .StdId = frame.id,
        .DLC = frame.dlc,
    };

    HAL_CAN_AddTxMessage(&peripherals->common_peripherals->hcan, &header,
                         frame.data, &mailbox);
  }
}

void power_measure(void *argument) {
  UNUSED(argument);

  TimerHandle_t xTimer = xTimerCreate(
      "measureTaskTimer", pdMS_TO_TICKS(POWER_MEASURE_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      power_measure_timer);

  xTimerStart(xTimer, portMAX_DELAY);

  uint16_t adc1Buf[4];
  uint16_t adc2Buf[4];

  CANFrame sensorPowerMessage;
  CANFrame servoCurrentMessage;
  CANFrame batteryVoltageMessage;
  CANFrame vccServoVoltageMessage;
  CANFrame hBridgeWindingCurrentMessage;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adc1Buf,
                      sizeof(adc1Buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adc2Buf,
                      sizeof(adc2Buf) / sizeof(uint16_t));

    // Wait for DMA
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ADCToSensorPowerMessage(adc1Buf[0], &sensorPowerMessage);
    ADCToServoCurrentMessage(adc1Buf[2], &servoCurrentMessage);
    ADCToBatteryVoltageMessage(adc1Buf[3], &batteryVoltageMessage);
    ADCToVCCServoVoltageMessage(adc2Buf[0], &vccServoVoltageMessage);
    ADCToHBridgeWindingCurrentMessage(adc2Buf[1],
                                      &hBridgeWindingCurrentMessage);

    xQueueSendToBack(can_message_queue, &sensorPowerMessage, 0);
    xQueueSendToBack(can_message_queue, &servoCurrentMessage, 0);
    xQueueSendToBack(can_message_queue, &batteryVoltageMessage, 0);
    xQueueSendToBack(can_message_queue, &vccServoVoltageMessage, 0);
    xQueueSendToBack(can_message_queue, &hBridgeWindingCurrentMessage, 0);
  }
}

void pwm(void *argument) {
  UNUSED(argument);

  TimerHandle_t xTimer =
      xTimerCreate("defaultTaskTimer", pdMS_TO_TICKS(PWM_TASK_PERIOD_MS),
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
