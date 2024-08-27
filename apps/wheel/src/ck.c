#include <stdio.h>

#include "ck-data.h"
#include "report.h"
#include "wheel-speed.h"

// Libs
#include "rover.h"

// STM32Common
#include "device-id.h"
#include "error.h"
#include "letter-reader.h"
#include "stm32f3xx_hal_cortex.h"

// CK
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

// For the CK startup sequence timer
StaticTimer_t default_letter_timer_buf;
TimerHandle_t default_letter_timer;
void default_letter_timer_callback(TimerHandle_t timer);
void start_default_letter_timer(void);

void mayor_init(void);
ck_err_t set_action_mode(ck_action_mode_t mode);
ck_err_t set_city_mode(ck_city_mode_t mode);

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

void init_ck_task(uint8_t priority) {
  letter_reader_cfg_t letter_reader_cfg = {
      .priority = priority++,
      .app_letter_handler_func = handle_letter,
  };

  if (init_letter_reader_task(letter_reader_cfg) != APP_OK) {
    error();
  }

  mayor_init();
}

void mayor_init(void) {
  default_letter_timer = xTimerCreateStatic(
      "default letter timer", pdMS_TO_TICKS(200),
      pdFALSE,  // Don't auto reload timer
      NULL,     // Timer ID, unused
      default_letter_timer_callback, &default_letter_timer_buf);

  ck_data_init();
  ck_data_t *ck_data = get_ck_data();

  uint32_t city_address = ROVER_WHEEL_FRONT_LEFT_ID;

  ck_id_t ck_id;

  // Hard coded ID for each rover city.
  // Written to flash here to be shared with the bootloader.
  // We also verify that hard coded city address is correct.
  int err = read_ck_id(&ck_id);
  if (err != APP_OK || ck_id.city_address != city_address) {
    printf("Rewriting ck_id to SPI flash.\r\n");
    ck_id = get_default_ck_id(city_address);
    err = write_ck_id(&ck_id);
    if (err != APP_OK) {
      printf("Error writing ck_id to SPI flash: 0x%x\r\n", err);
    }
  }

  ck_mayor_t mayor = {
      .ean_no = 100 + city_address,  // NOLINT(*-magic-numbers)
      .serial_no = get_device_serial(),
      .ck_id = ck_id,
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .start_200ms_timer = start_default_letter_timer,
      .folder_count = CK_DATA_FOLDER_COUNT,
      .folders = ck_data->folders,
      .list_count = CK_DATA_LIST_COUNT,
      .lists = ck_data->lists,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    printf("Error setting up mayor.\r\n");
    error();
  }
}

void default_letter_timer_callback(TimerHandle_t timer) {
  (void)timer;

  if (ck_default_letter_timeout() != CK_OK) {
    printf("CAN Kingdom error in ck_default_letter_timeout().\r\n");
  }
}

void start_default_letter_timer(void) {
  xTimerStart(default_letter_timer, portMAX_DELAY);
}

ck_err_t set_action_mode(ck_action_mode_t mode) {
  if (mode == CK_ACTION_MODE_RESET) {
    // This delay is there because otherwise error frames will be generated on
    // the CAN bus. The root cause is still unknown.
    vTaskDelay(pdMS_TO_TICKS(10));
    HAL_NVIC_SystemReset();
  }
  return CK_OK;
}

ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  if (ck_get_action_mode() == CK_ACTION_MODE_FREEZE) {
    return APP_OK;
  }

  ck_data_t *ck_data = get_ck_data();

  if (folder->folder_no == ck_data->set_wheel_parameters_folder->folder_no) {
    return process_set_wheel_parameters_letter(letter);
  }
  if (folder->folder_no == ck_data->set_report_freq_folder->folder_no) {
    return process_report_freq_letter(letter);
  }

  return APP_OK;
}