#include <stdint.h>
#include <stm32f3xx_hal.h>
#include <string.h>

#include "app.h"
#include "peripherals.h"
#include "utils.h"

// FreeRTOS includes
#include "cmsis_os.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

// This controls how often the default task should run.
// Default to once every 20ms.
#define DEFAULT_TASK_PERIOD_MS 20

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static peripherals_t *peripherals;

void system_clock_config(void);
void StartDefaultTask(void *argument);

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
  adc1_init();
  can_init();
  i2c1_init();
  uart1_init();
  spi1_init();
  spi3_init();

  // Init scheduler
  osKernelInitialize();

  // Create tasks
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  Print(&peripherals->huart1, "Starting application...\n");

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  NotifyTask(defaultTaskHandle);
}

void defaultTaskTimer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  NotifyTask(defaultTaskHandle);
}

void StartDefaultTask(void *argument) {
  UNUSED(argument);

  uint16_t adcBuf[4];
  uint8_t swData[4];

  TimerHandle_t xTimer =
      xTimerCreate("defaultTaskTimer", pdMS_TO_TICKS(DEFAULT_TASK_PERIOD_MS),
                   pdTRUE,  // Auto reload timer
                   NULL,    // Timer ID, unused
                   defaultTaskTimer);

  HAL_CAN_Start(&peripherals->hcan);

  xTimerStart(xTimer, portMAX_DELAY);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Get joystick readings
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adcBuf,
                      sizeof(adcBuf) / sizeof(uint16_t));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA
    SendAnalogPortMessage(adcBuf);

    // Get Switch readings
    ReadSwitches(swData);
    SendSwitchPortMessage(swData);
  }
}
