#include "rover.h"

#include <stdio.h>

#include "rover-helpers.h"

// STM32Common
#include "device-id.h"
#include "error.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

static TaskHandle_t king_task;
static StaticTask_t king_buf;
static StackType_t king_stack[configMINIMAL_STACK_SIZE];

// King task helpers
static void king(void *unused);
static void king_timer_callback(TimerHandle_t timer);

void init_king_task(uint32_t priority) {
  king_task = xTaskCreateStatic(king, "king", configMINIMAL_STACK_SIZE, NULL,
                                priority, king_stack, &king_buf);
}

static void king(void *unused) {
  (void)unused;

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic("king timer", pdMS_TO_TICKS(150),
                                           pdFALSE,  // Don't auto reload timer
                                           NULL,     // Timer ID, unused
                                           king_timer_callback, &timer_buf);
  xTimerStart(timer, portMAX_DELAY);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for timer

  if (someone_else_is_king()) {
    printf("Suspending king task.\r\n");
    vTaskSuspend(king_task);
    // Task will never be resumed
  }

  ck_err_t err = CK_OK;

  // Send default letter several times to make sure everyone receives it.
  for (uint8_t i = 0; i < 5; i++) {  // NOLINT(*-magic-numbers)
    err = send_default_letter();
    if (err != CK_OK) {
      error();
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  err = set_rover_base_number();
  if (err != CK_OK) {
    error();
  }

  vTaskDelay(pdMS_TO_TICKS(50));  // Give cities time to respond

  err = assign_rover_envelopes(get_cached_ck_id());
  if (err != CK_OK) {
    error();
  }

  err = configure_rover_settings();
  if (err != CK_OK) {
    error();
  }

  err = start_communication();
  if (err != CK_OK) {
    error();
  }

  vTaskSuspend(king_task);
}

static void king_timer_callback(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(king_task);
}
