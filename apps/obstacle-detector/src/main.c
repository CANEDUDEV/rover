#include <stdio.h>

#include "ck.h"
#include "obstacle-detector.h"
#include "peripherals.h"
#include "report.h"

// STM32Common
#include "clock.h"
#include "error.h"
#include "jsondb.h"
#include "lfs-wrapper.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  peripherals_init();

  uint8_t priority = LOWEST_TASK_PRIORITY;

  if (init_lfs_task(priority++) < 0) {
    error();
  }

  jsondb_init();

  init_obstacle_detector_task(priority++);
  init_report_task(priority++);
  init_ck_task(priority++);

  printf("Starting application...\r\n");

  vTaskStartScheduler();

  while (1) {
  }
}
