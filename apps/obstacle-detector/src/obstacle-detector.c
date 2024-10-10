#include "obstacle-detector.h"

#include <math.h>
#include <string.h>

#include "adc.h"
#include "ck-data.h"
#include "peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define OBSTACLE_DETECTOR_TASK_STACK_SIZE \
  (sizeof(adc_samples_t) + configMINIMAL_STACK_SIZE)

static TaskHandle_t obstacle_detector_task;
static StaticTask_t obstacle_detector_task_buf;
static StackType_t
    obstacle_detector_task_stack[OBSTACLE_DETECTOR_TASK_STACK_SIZE];

static void obstacle_detector(void* unused);

void init_obstacle_detector_task(uint8_t priority) {
  obstacle_detector_task = xTaskCreateStatic(
      obstacle_detector, "obstacle detector", OBSTACLE_DETECTOR_TASK_STACK_SIZE,
      NULL, priority, obstacle_detector_task_stack,
      &obstacle_detector_task_buf);
}

static void obstacle_detector(void* unused) {
  (void)unused;

  peripherals_t* peripherals = get_peripherals();
  ck_data_t* ck_data = get_ck_data();

  while (1) {
    adc_samples_t adc_samples = {0};
    adc_reading_t adc_reading = {0};
    uint16_t distance[ADC1_NUM_CHANNELS] = {0};

    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t*)adc_samples.adc1_buf,
                      ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    adc_average_samples(&adc_reading, &adc_samples);

    for (int i = 0; i < ADC1_NUM_CHANNELS; i++) {
      float measured_distance = adc_value_to_distance(adc_reading.adc1_buf[i]);
      distance[i] = (uint16_t)roundf(measured_distance);
    }

    memcpy(ck_data->object_distance_page->lines, distance,
           ck_data->object_distance_page->line_count);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(obstacle_detector_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}
