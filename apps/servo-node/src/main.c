#include <stdio.h>
#include <string.h>

#include "app.h"
#include "peripherals.h"
#include "utils.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

// Hardware
static peripherals_t *peripherals;

void system_clock_config(void);

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

  if (FlashRWInit() != APP_OK) {
    Error_Handler();
  }

  // Configure the system clock
  system_clock_config();

  // Initialize all configured peripherals
  peripherals = get_peripherals();
  gpio_init();
  dma_init();
  can_init();
  uart1_init();
  spi1_init();
  adc1_init();
  adc2_init();
  i2c1_init();
  i2c3_init();
  spi3_init();
  tim1_init();

  InitPotentiometers();

  task_init();

  can_message_queue = xQueueCreate(CAN_MESSAGE_QUEUE_LENGTH, sizeof(CANFrame));

  Print(&peripherals->huart1, "Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void system_clock_config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 |
                                       RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C3 |
                                       RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
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
  NotifyTask(power_measure_task);
}

void defaultTaskTimer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  NotifyTask(pwm_task);
}

void measureTaskTimer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  NotifyTask(power_measure_task);
}

void mailboxFreeCallback(CAN_HandleTypeDef *_hcan) {
  // Only notify when going from 0 free mailboxes to 1 free
  if (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) <= 1) {
    NotifyTask(can_tx_task);
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailboxFreeCallback(_hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailboxFreeCallback(_hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailboxFreeCallback(_hcan);
}

void can_tx(void *argument) {
  UNUSED(argument);

  HAL_CAN_ActivateNotification(&peripherals->hcan, CAN_IT_TX_MAILBOX_EMPTY);
  HAL_CAN_Start(&peripherals->hcan);

  uint32_t mailbox = 0;

  for (;;) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&peripherals->hcan) == 0) {
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

    HAL_CAN_AddTxMessage(&peripherals->hcan, &header, frame.data, &mailbox);
  }
}

void power_measure(void *argument) {
  UNUSED(argument);

  TimerHandle_t xTimer = xTimerCreate(
      "measureTaskTimer", pdMS_TO_TICKS(POWER_MEASURE_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      measureTaskTimer);

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
                   defaultTaskTimer);

  xTimerStart(xTimer, portMAX_DELAY);
  HAL_TIM_PWM_Start(&peripherals->htim1, TIM_CHANNEL_4);

  uint32_t pulse = PWM_MID_POS_PULSE;
  STEERING_DIRECTION direction = LEFT;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation
    UpdatePWMDutyCycle(&pulse, &direction);
  }
}
