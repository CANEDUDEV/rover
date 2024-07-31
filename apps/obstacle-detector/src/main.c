#include <math.h>
#include <stdio.h>

#include "adc.h"
#include "peripherals.h"

// STM32Common
#include "clock.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "stm32f3xx_hal_adc.h"
#include "task.h"

#define TASK_STACK_SIZE (sizeof(adc_samples_t) + configMINIMAL_STACK_SIZE)

static TaskHandle_t main_task;
static StaticTask_t task_buf;
static StackType_t task_stack[TASK_STACK_SIZE];

void task(void* unused) {
  (void)unused;

  peripherals_t* peripherals = get_peripherals();

  adc_samples_t adc_samples;
  adc_reading_t adc_reading;

  uint16_t distance[ADC1_NUM_CHANNELS];

  while (1) {
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t*)adc_samples.adc1_buf,
                      ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    adc_average_samples(&adc_reading, &adc_samples);

    for (int i = 0; i < ADC1_NUM_CHANNELS; i++) {
      float measured_distance = adc_value_to_distance(adc_reading.adc1_buf[i]);
      distance[i] = (uint16_t)roundf(measured_distance);
      printf("distance %d: %umm\r\n", i, distance[i]);
    }

    printf("\r\n");

    vTaskDelay(100);
  }
}

int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  peripherals_init();

  main_task = xTaskCreateStatic(task, "main", TASK_STACK_SIZE, NULL, 8,
                                task_stack, &task_buf);

  vTaskStartScheduler();

  while (1) {
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(main_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}
