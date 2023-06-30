#include <string.h>

#include "app.h"
#include "clock.h"
#include "error.h"
#include "flash.h"
#include "peripherals.h"
#include "print.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

// Hardware
static peripherals_t *peripherals;

// FreeRTOS
#define PWM_TASK_PERIOD_MS 20
#define POWER_MEASURE_TASK_PERIOD_MS 100
#define TASK_PRIORITY 24

static TaskHandle_t pwm_task;
static StaticTask_t pwm_buffer;
static StackType_t pwm_stack[configMINIMAL_STACK_SIZE];

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

  print("Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void task_init(void) {
  pwm_task = xTaskCreateStatic(pwm, "pwm", configMINIMAL_STACK_SIZE, NULL,
                               TASK_PRIORITY, pwm_stack, &pwm_buffer);

  power_measure_task = xTaskCreateStatic(
      power_measure, "power measure", configMINIMAL_STACK_SIZE, NULL,
      TASK_PRIORITY - 1, power_measure_stack, &power_measure_buffer);
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

void power_measure(void *argument) {
  UNUSED(argument);

  TimerHandle_t xTimer = xTimerCreate(
      "measureTaskTimer", pdMS_TO_TICKS(POWER_MEASURE_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      power_measure_timer);

  xTimerStart(xTimer, portMAX_DELAY);

  uint16_t adc1Buf[3];
  uint16_t adc2Buf[2];

  uint32_t mailbox = 0;

  CAN_TxHeaderTypeDef can_headers[5];  // NOLINT

  for (uint8_t i = 0; i < 5; i++) {  // NOLINT
    can_headers[i].IDE = CAN_ID_STD;
    can_headers[i].RTR = CAN_RTR_DATA;
  }

  CANFrame sensorPowerMessage;
  CANFrame servoCurrentMessage;
  CANFrame batteryVoltageMessage;
  CANFrame vccServoVoltageMessage;
  CANFrame hBridgeWindingCurrentMessage;

  CAN_HandleTypeDef *hcan = &peripherals->common_peripherals->hcan;
  HAL_CAN_Start(hcan);

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
    ADCToServoCurrentMessage(adc1Buf[1], &servoCurrentMessage);
    ADCToBatteryVoltageMessage(adc1Buf[2], &batteryVoltageMessage);
    ADCToVCCServoVoltageMessage(adc2Buf[0], &vccServoVoltageMessage);
    ADCToHBridgeWindingCurrentMessage(adc2Buf[1],
                                      &hBridgeWindingCurrentMessage);

    can_headers[0].DLC = sensorPowerMessage.dlc;
    can_headers[0].StdId = sensorPowerMessage.id;

    can_headers[1].DLC = servoCurrentMessage.dlc;
    can_headers[1].StdId = servoCurrentMessage.id;

    can_headers[2].DLC = batteryVoltageMessage.dlc;
    can_headers[2].StdId = batteryVoltageMessage.id;

    can_headers[3].DLC = vccServoVoltageMessage.dlc;
    can_headers[3].StdId = vccServoVoltageMessage.id;

    can_headers[4].DLC = hBridgeWindingCurrentMessage.dlc;
    can_headers[4].StdId = hBridgeWindingCurrentMessage.id;

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 3) {
      // Busy loop, wait until ready to send 3 messages
    }
    HAL_CAN_AddTxMessage(hcan, &can_headers[0], sensorPowerMessage.data,
                         &mailbox);
    HAL_CAN_AddTxMessage(hcan, &can_headers[1], servoCurrentMessage.data,
                         &mailbox);
    HAL_CAN_AddTxMessage(hcan, &can_headers[2], batteryVoltageMessage.data,
                         &mailbox);

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 2) {
      // Busy loop, wait until ready to send another 2 messages
    }
    HAL_CAN_AddTxMessage(hcan, &can_headers[3], vccServoVoltageMessage.data,
                         &mailbox);
    HAL_CAN_AddTxMessage(hcan, &can_headers[4],
                         hBridgeWindingCurrentMessage.data, &mailbox);
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
