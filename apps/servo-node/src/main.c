#include <stdio.h>
#include <string.h>

#include "app.h"
#include "peripherals.h"
#include "utils.h"

// FreeRTOS includes
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

// How often the periodic tasks should run (ms)
#define DEFAULT_TASK_PERIOD_MS 20
#define MEASURE_TASK_PERIOD_MS 100

#define CAN_MESSAGE_QUEUE_LENGTH 10

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t CANTxTaskHandle;
const osThreadAttr_t CANTxTask_attributes = {
    .name = "CANTxTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

osThreadId_t measureTaskHandle;
const osThreadAttr_t measureTask_attributes = {
    .name = "measureTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

QueueHandle_t CANMessageQueue;

static peripherals_t *peripherals;

void system_clock_config(void);

// Tasks
void StartDefaultTask(void *argument);
void StartCANTxTask(void *argument);
void StartMeasureTask(void *argument);

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

  // Init scheduler
  osKernelInitialize();

  CANMessageQueue = xQueueCreate(CAN_MESSAGE_QUEUE_LENGTH, sizeof(CANFrame));

  // Create tasks
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  CANTxTaskHandle = osThreadNew(StartCANTxTask, NULL, &CANTxTask_attributes);
  measureTaskHandle =
      osThreadNew(StartMeasureTask, NULL, &measureTask_attributes);

  Print(&peripherals->huart1, "Starting application...\r\n");
  // Start scheduler
  osKernelStart();

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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  NotifyTask(measureTaskHandle);
}

void defaultTaskTimer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  NotifyTask(defaultTaskHandle);
}

void measureTaskTimer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  NotifyTask(measureTaskHandle);
}

void mailboxFreeCallback(CAN_HandleTypeDef *_hcan) {
  // Only notify when going from 0 free mailboxes to 1 free
  if (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) <= 1) {
    NotifyTask(CANTxTaskHandle);
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

void StartCANTxTask(void *argument) {
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
    xQueueReceive(CANMessageQueue, &frame, portMAX_DELAY);

    CAN_TxHeaderTypeDef header = {
        .StdId = frame.id,
        .DLC = frame.dlc,
    };

    HAL_CAN_AddTxMessage(&peripherals->hcan, &header, frame.data, &mailbox);
  }
}

void StartMeasureTask(void *argument) {
  UNUSED(argument);

  TimerHandle_t xTimer =
      xTimerCreate("measureTaskTimer", pdMS_TO_TICKS(MEASURE_TASK_PERIOD_MS),
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

    xQueueSendToBack(CANMessageQueue, &sensorPowerMessage, 0);
    xQueueSendToBack(CANMessageQueue, &servoCurrentMessage, 0);
    xQueueSendToBack(CANMessageQueue, &batteryVoltageMessage, 0);
    xQueueSendToBack(CANMessageQueue, &vccServoVoltageMessage, 0);
    xQueueSendToBack(CANMessageQueue, &hBridgeWindingCurrentMessage, 0);
  }
}

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

void StartDefaultTask(void *argument) {
  UNUSED(argument);

  TimerHandle_t xTimer =
      xTimerCreate("defaultTaskTimer", pdMS_TO_TICKS(DEFAULT_TASK_PERIOD_MS),
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
