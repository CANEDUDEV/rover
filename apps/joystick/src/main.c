#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "clock.h"
#include "error.h"
#include "flash.h"
#include "peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

// Hardware
static peripherals_t *peripherals;

// FreeRTOS
#define IO_READ_TASK_PERIOD_MS 20
#define TASK_PRIORITY 24

static TaskHandle_t io_read_task;
static StaticTask_t io_read_buffer;
static StackType_t io_read_stack[configMINIMAL_STACK_SIZE];

void task_init(void);
void io_read(void *argument);

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

  task_init();

  printf("Starting application...\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void task_init(void) {
  io_read_task =
      xTaskCreateStatic(io_read, "io read", configMINIMAL_STACK_SIZE, NULL,
                        TASK_PRIORITY, io_read_stack, &io_read_buffer);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(io_read_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void io_read_timer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(io_read_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void io_read(void *argument) {
  UNUSED(argument);

  uint16_t adcBuf[4];
  uint8_t swData[4];

  StaticTimer_t timer_buf;
  TimerHandle_t timer =
      xTimerCreateStatic("io read timer", pdMS_TO_TICKS(IO_READ_TASK_PERIOD_MS),
                         pdTRUE,  // Auto reload timer
                         NULL,    // Timer ID, unused
                         io_read_timer, &timer_buf);

  HAL_CAN_Start(&peripherals->common_peripherals->hcan);

  xTimerStart(timer, portMAX_DELAY);

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
