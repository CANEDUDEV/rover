#include "ck-data.h"
#include "freertos-tasks.h"
#include "peripherals.h"
#include "potentiometer.h"
#include "rover.h"

// CK
#include "mayor.h"
#include "postmaster-hal.h"

// STM32Common
#include "clock.h"
#include "error.h"
#include "print.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

// For the CK startup sequence timer
StaticTimer_t default_letter_timer_buf;
TimerHandle_t default_letter_timer;
void default_letter_timer_callback(TimerHandle_t timer);
void start_default_letter_timer(void);

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

  // Configure potentiometers
  configure_servo_potentiometer(POT_SERVO_DEFAULT);
  configure_sensor_potentiometer(POT_SENSOR_DEFAULT);

  peripherals_t *peripherals = get_peripherals();
  HAL_TIM_PWM_Start(&peripherals->htim1, TIM_CHANNEL_4);

  task_init();
  mayor_init();

  print("Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void mayor_init(void) {
  peripherals_t *peripherals = get_peripherals();
  postmaster_init(
      &peripherals->common_peripherals->hcan);  // Set up the postmaster

  default_letter_timer = xTimerCreateStatic(
      "default letter timer", pdMS_TO_TICKS(200),
      pdFALSE,  // Auto reload timer
      NULL,     // Timer ID, unused
      default_letter_timer_callback, &default_letter_timer_buf);

  ck_data_init();
  ck_data_t *ck_data = get_ck_data();

  ck_mayor_t mayor = {
      .ean_no = 123,     // NOLINT(*-magic-numbers)
      .serial_no = 456,  // NOLINT(*-magic-numbers)
#ifdef MOTOR
      .city_address = ROVER_MOTOR_ID,
#else
      .city_address = ROVER_SERVO_ID,
#endif
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .start_200ms_timer = start_default_letter_timer,
      .folder_count = CK_DATA_FOLDER_COUNT,
      .folders = ck_data->folders,
      .list_count = CK_DATA_LIST_COUNT,
      .lists = ck_data->lists,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    print("Error setting up mayor.\r\n");
    error();
  }
}

void default_letter_timer_callback(TimerHandle_t timer) {
  (void)timer;

  if (ck_default_letter_timeout() != CK_OK) {
    print("CAN Kingdom error in ck_default_letter_timeout().\r\n");
  }
}

void start_default_letter_timer(void) {
  xTimerStart(default_letter_timer, portMAX_DELAY);
}

ck_err_t set_action_mode(ck_action_mode_t mode) {
  (void)mode;
  return CK_OK;
}

ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}
