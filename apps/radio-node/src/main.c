#include <stm32f3xx_hal.h>
#include <string.h>

#include "clock.h"
#include "error.h"
#include "flash.h"
#include "peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// Hardware
static peripherals_t *peripherals;

void system_clock_config(void);

// Application
#define CAN_BASE_ID 100U
#define CAN_MAX_DLC 8

#define SBUS_PACKET_LENGTH 25
#define SBUS_HEADER 0x0F

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLC];
} CANFrame;

// FreeRTOS
#define CAN_MESSAGE_QUEUE_LENGTH 10
#define IO_READ_TASK_PERIOD_MS 20
#define TASK_PRIORITY 24

QueueHandle_t can_message_queue;

static TaskHandle_t sbus_read_task;
static StaticTask_t sbus_read_buffer;
static StackType_t sbus_read_stack[configMINIMAL_STACK_SIZE];

static TaskHandle_t can_tx_task;
static StaticTask_t can_tx_buffer;
static StackType_t can_tx_stack[configMINIMAL_STACK_SIZE];

void task_init(void);
void sbus_read(void *argument);
void can_tx(void *argument);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  if (flash_init() != APP_OK) {
    error();
  }

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  peripherals = get_peripherals();
  gpio_init();
  can_init();
  uart1_init();
  spi1_init();

  task_init();

  can_message_queue = xQueueCreate(CAN_MESSAGE_QUEUE_LENGTH, sizeof(CANFrame));

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void task_init(void) {
  sbus_read_task =
      xTaskCreateStatic(sbus_read, "sbus read", configMINIMAL_STACK_SIZE, NULL,
                        TASK_PRIORITY, sbus_read_stack, &sbus_read_buffer);

  can_tx_task =
      xTaskCreateStatic(can_tx, "can tx", configMINIMAL_STACK_SIZE, NULL,
                        TASK_PRIORITY, can_tx_stack, &can_tx_buffer);
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
    error();
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
    error();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    error();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  UNUSED(huart);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(sbus_read_task, &xHigherPriorityTaskWoken);
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

  peripherals_t *peripherals = get_peripherals();
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

void sbus_read(void *argument) {
  UNUSED(argument);

  uint8_t sbusHeader = 0;
  uint8_t sbusData[SBUS_PACKET_LENGTH];

  peripherals_t *peripherals = get_peripherals();
  // Wait until reception of one complete message, in case we power up in the
  // middle of a transmission
  while (sbusHeader != SBUS_HEADER) {
    HAL_UART_Receive(&peripherals->huart1, &sbusHeader, sizeof(sbusHeader),
                     HAL_MAX_DELAY);
  }
  HAL_UART_Receive(&peripherals->huart1, sbusData, sizeof(sbusData) - 1,
                   HAL_MAX_DELAY);

  CANFrame frames[3];
  for (int i = 0; i < 3; i++) {
    frames[i].id = CAN_BASE_ID + i;
    frames[i].dlc = CAN_MAX_DLC;
  }

  HAL_CAN_Start(&peripherals->hcan);
  for (;;) {
    memset(sbusData, 0, sizeof(sbusData));
    HAL_UART_Receive_IT(&peripherals->huart1, sbusData, sizeof(sbusData));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (int i = 0; i < 3; i++) {
      memcpy(frames[i].data, &sbusData[i * CAN_MAX_DLC], CAN_MAX_DLC);
      xQueueSendToBack(can_message_queue, &frames[i], 0);
    }
  }
}
