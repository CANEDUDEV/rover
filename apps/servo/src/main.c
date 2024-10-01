#include <stdio.h>

#include "ck-data.h"
#include "freertos-tasks.h"
#include "peripherals.h"
#include "pwm.h"
#include "rover.h"

// CK
#include "mayor.h"

// STM32Common
#include "clock.h"
#include "device-id.h"
#include "error.h"
#include "jsondb.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

void mayor_init(void);
ck_err_t set_action_mode(ck_action_mode_t mode);
ck_err_t set_city_mode(ck_city_mode_t mode);

int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  peripherals_init();

  task_init();
  jsondb_init();
  mayor_init();

  printf("Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void mayor_init(void) {
  ck_data_init();
  ck_data_t *ck_data = get_ck_data();

  uint32_t city_address = ROVER_SERVO_ID;

#ifdef MOTOR
  city_address = ROVER_MOTOR_ID;
#endif

  ck_id_t ck_id;

  // Hard coded ID for each rover city.
  // Written to flash here to be shared with the bootloader.
  // Can be overridden, in that case the original ID can be restored by
  // formatting the FS.
  int err = read_ck_id(&ck_id);
  if (err != APP_OK) {
    printf("error: couldn't read ck_id. Writing default ck_id to FS.\r\n");
    ck_id = get_default_ck_id(city_address);
    err = write_ck_id(&ck_id);
    if (err != APP_OK) {
      printf("Error writing ck_id to FS: 0x%x\r\n", err);
    }
  }

  ck_mayor_t mayor = {
      .ean_no = 100 + city_address,  // NOLINT(*-magic-numbers)
      .serial_no = get_device_serial(),
      .ck_id = ck_id,
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .folder_count = CK_DATA_FOLDER_COUNT,
      .folders = ck_data->folders,
      .list_count = CK_DATA_LIST_COUNT,
      .lists = ck_data->lists,
      .skip_startup = true,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    printf("Error setting up mayor.\r\n");
    error();
  }
}

ck_err_t set_action_mode(ck_action_mode_t mode) {
  if (mode == CK_ACTION_MODE_RESET) {
    // This delay is there because otherwise error frames will be generated on
    // the CAN bus. The root cause is still unknown.
    vTaskDelay(pdMS_TO_TICKS(10));
    pwm_set_pulse(PWM_NEUTRAL_PULSE_MUS);
    HAL_NVIC_SystemReset();
  }
  if (mode == CK_ACTION_MODE_FREEZE) {
    pwm_set_pulse(PWM_NEUTRAL_PULSE_MUS);
  }
  return CK_OK;
}

ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}
