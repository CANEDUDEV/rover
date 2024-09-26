#include <stdio.h>

#include "ck.h"
#include "lfs-wrapper.h"
#include "peripherals.h"
#include "potentiometer.h"
#include "report.h"
#include "wheel-speed.h"

// STM32Common
#include "clock.h"
#include "error.h"
#include "jsondb.h"

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

  init_potentiometers();
  write_sensor_potentiometer(POTENTIOMETER_SENSOR_DEFAULT);
  write_servo_potentiometer(POTENTIOMETER_SERVO_DEFAULT);

  uint8_t priority = LOWEST_TASK_PRIORITY;

  if (init_lfs_task(priority++) < 0) {
    error();
  }

  jsondb_init();

  init_wheel_speed_task(priority++);
  init_report_task(priority++);
  init_ck_task(priority++);

  printf("Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}
