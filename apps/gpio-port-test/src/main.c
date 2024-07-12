#include <stdio.h>

// STM32Common
#include "clock.h"
#include "common-peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "stm32f3xx_hal_gpio.h"
#include "task.h"

static StaticTask_t task_buf;
static StackType_t task_stack[configMINIMAL_STACK_SIZE];

void task(void* unused) {
  (void)unused;

  // For GPIO pin
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  // For power pin
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  while (1) {
    // Toggle GPIO pin on/off

    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    // vTaskDelay(2000);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    // vTaskDelay(1000);

    // Toggle power pin on/off

    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    // vTaskDelay(1000);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    // vTaskDelay(1000);

    // Toggle AN0 on / off

    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    // vTaskDelay(1000);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    // vTaskDelay(1000);

    // Read from AN0

    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    printf("state 1: %d\r\n", state);
    vTaskDelay(1000);
    state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    printf("state 2: %d\r\n", state);
    vTaskDelay(1000);

    printf("Hello\r\n");
  }
}

int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  common_peripherals_init();

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // VDD_IO_LEVEL
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  GPIO_InitTypeDef gpio_init = {
      .Pin = GPIO_PIN_2,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(GPIOB, &gpio_init);

  // GPIO0-3
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio_init);

  // GPIO_PWRON_1-2
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  gpio_init.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio_init);

  // GPIO_PWRON_3-4
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  // AN0 as output
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  gpio_init.Pin = GPIO_PIN_3;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // AN0 as input
  gpio_init.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  xTaskCreateStatic(task, "main", configMINIMAL_STACK_SIZE, NULL, 8, task_stack,
                    &task_buf);

  vTaskStartScheduler();

  while (1) {
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  (void)huart;
  uart1_msp_init();
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
  (void)huart;
  uart1_msp_deinit();
}
