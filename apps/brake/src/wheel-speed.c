#include <stdio.h>
#include <string.h>

#include "ck-data.h"
#include "error.h"
#include "peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define WHEEL_SPEED_TASK_STACK_SIZE (2 * configMINIMAL_STACK_SIZE)
static TaskHandle_t wheel_speed_task;
static StaticTask_t wheel_speed_task_buf;
static StackType_t wheel_speed_task_stack[WHEEL_SPEED_TASK_STACK_SIZE];

void measure_wheel_speed(void* unused);
void measure_task_timer(TimerHandle_t timer);
uint32_t calculate_measure_delay(uint32_t rps);

#define TIM2_FREQ (72 * 1000 * 1000)
#define DEFAULT_WHEEL_DIAMETER_M 0.16F
#define DEFAULT_COG_COUNT 45

struct pwm_state {
  bool pulse_detected;
  uint32_t ic_value;
  uint32_t freq;
};

typedef struct {
  uint32_t cog_count;
  float wheel_diameter_m;
} wheel_speed_t;

static struct pwm_state pwm;

static wheel_speed_t ws_params = {
    .wheel_diameter_m = DEFAULT_WHEEL_DIAMETER_M,
    .cog_count = DEFAULT_COG_COUNT,
};

void init_wheel_speed_task(uint8_t priority) {
  wheel_speed_task = xTaskCreateStatic(
      measure_wheel_speed, "wheel speed", WHEEL_SPEED_TASK_STACK_SIZE, NULL,
      priority, wheel_speed_task_stack, &wheel_speed_task_buf);
}

// 8 bytes in page
// bytes 0-3: number of cogs in wheel (uint32_t)
// bytes 4-7: wheel diameter in meters (float)
int process_set_wheel_parameters_letter(const ck_letter_t* letter) {
  ck_data_t* ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->set_wheel_parameters_folder->dlc) {
    return APP_NOT_OK;
  }

  uint32_t cog_count = 0;
  memcpy(&cog_count, letter->page.lines, sizeof(cog_count));
  if (cog_count != 0) {
    ws_params.cog_count = cog_count;
  }

  float wheel_diameter_m = 0;
  memcpy(&wheel_diameter_m, &letter->page.lines[4], sizeof(wheel_diameter_m));

  if (wheel_diameter_m <= 0) {
    return APP_NOT_OK;
  }

  if (wheel_diameter_m > 0) {
    ws_params.wheel_diameter_m = wheel_diameter_m;
  }

  return APP_OK;
}

void measure_wheel_speed(void* unused) {
  (void)unused;

  uint32_t measure_delay_ms = 500;  // NOLINT

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic("measure timer", pdMS_TO_TICKS(500),
                                           pdFALSE,  // Don't auto reload timer
                                           NULL,     // Timer ID, unused
                                           measure_task_timer, &timer_buf);

  peripherals_t* peripherals = get_peripherals();
  HAL_TIM_IC_Start_IT(&peripherals->htim2, TIM_CHANNEL_1);

  const float pi = 3.14159F;
  const float mps_to_kph = 3.6F;

  ck_data_t* ck_data = get_ck_data();

  while (1) {
    pwm.pulse_detected = false;
    xTimerChangePeriod(timer, measure_delay_ms, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t freq = 0;

    if (pwm.pulse_detected) {
      freq = pwm.freq;
    }

    float rps = (float)freq / (float)ws_params.cog_count;

    measure_delay_ms = calculate_measure_delay((uint32_t)rps);

    const float rpm = rps * 60;
    float speed = pi * ws_params.wheel_diameter_m * rps * mps_to_kph;

    memcpy(ck_data->wheel_speed_page->lines, &rpm, sizeof(float));
    memcpy(&ck_data->wheel_speed_page->lines[4], &speed, sizeof(float));
  }
}

void measure_task_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(wheel_speed_task);
}

uint32_t calculate_measure_delay(uint32_t rps) {
  // Measure more often when moving faster.
  const int32_t min_measure_delay_ms = 10;
  const int32_t max_measure_delay_ms = 200;

  if (rps == 0) {
    return max_measure_delay_ms;
  }

  uint32_t measure_delay_ms = 1000 / rps;  // NOLINT

  if (measure_delay_ms < min_measure_delay_ms) {
    measure_delay_ms = min_measure_delay_ms;
  }

  if (measure_delay_ms > max_measure_delay_ms) {
    measure_delay_ms = max_measure_delay_ms;
  }

  return measure_delay_ms;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uint32_t ic_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (ic_value != 0) {
      pwm.freq = TIM2_FREQ / ic_value;
    }

    pwm.pulse_detected = true;
  }
}
