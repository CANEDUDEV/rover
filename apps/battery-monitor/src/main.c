#include <stdio.h>
#include <string.h>

#include "app.h"
#include "clock.h"
#include "error.h"
#include "flash.h"
#include "mayor.h"
#include "peripherals.h"
#include "ports.h"
#include "postmaster-hal.h"
#include "postmaster.h"
#include "print.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

// Hardware
static peripherals_t *peripherals;

// FreeRTOS
#define BATTERY_MONITOR_TASK_PERIOD_MS 20
#define BATTERY_REPORT_TASK_PERIOD_MS 200
#define LOWEST_TASK_PRIORITY 24

static TaskHandle_t battery_monitor_task;
static StaticTask_t battery_monitor_buf;
static StackType_t battery_monitor_stack[configMINIMAL_STACK_SIZE];
void battery_monitor(void *unused);

static TaskHandle_t battery_report_task;
static StaticTask_t battery_report_buf;
static StackType_t battery_report_stack[configMINIMAL_STACK_SIZE];
void battery_report(void *unused);

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t proc_letter_buf;
static StackType_t proc_letter_stack[configMINIMAL_STACK_SIZE];
void proc_letter(void *unused);

void task_init(void);

// CAN Kingdom data
#define PAGE_COUNT 4
#define DOC_COUNT 3
#define LIST_COUNT 2
#define FOLDER_COUNT 5
static ck_page_t pages[PAGE_COUNT];
static ck_document_t docs[DOC_COUNT];
static ck_list_t lists[LIST_COUNT];
static ck_folder_t folders[FOLDER_COUNT];

// Convenience pointers
static ck_list_t *tx_list = &lists[0];
static ck_list_t *rx_list = &lists[1];

static ck_page_t *cell_page0 = &pages[0];
static ck_page_t *cell_page1 = &pages[1];
static ck_page_t *reg_out_current_page = &pages[2];
static ck_page_t *vbat_out_current_page = &pages[3];

static ck_document_t *cell_doc = &docs[0];
static ck_document_t *reg_out_current_doc = &docs[1];
static ck_document_t *vbat_out_current_doc = &docs[2];

static ck_folder_t *cell_folder = &folders[2];
static ck_folder_t *reg_out_current_folder = &folders[3];
static ck_folder_t *vbat_out_current_folder = &folders[4];

// For the CK startup sequence timer
StaticTimer_t default_letter_timer_buf;
TimerHandle_t default_letter_timer;
void default_letter_timer_callback(TimerHandle_t timer);
void start_default_letter_timer(void);

void ck_init(void);
void update_pages(battery_state_t *battery_state);
void send_docs(void);

// Application
#define CAN_BASE_ID 0x100
#define LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD 10

// Set to 1 by GPIO external interrupt on the OVER_CURRENT pin.
static uint8_t over_current_fault = 0;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  system_clock_init();

  // Initialize all configured peripherals
  peripherals_init();
  peripherals = get_peripherals();

  task_init();
  ck_init();

  print(&peripherals->common_peripherals->huart1,
        "Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler.
  while (1) {
  }
}

ck_err_t set_action_mode(ck_action_mode_t mode) {
  (void)mode;
  return CK_OK;
}

ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

void ck_page_init(void) {
  // Set up the pages
  // NOLINTBEGIN(*-magic-numbers)
  cell_page0->line_count = 7;
  cell_page0->lines[0] = 0;
  // Pagination
  cell_page1->line_count = 7;
  cell_page1->lines[0] = 1;
  reg_out_current_page->line_count = 2;
  vbat_out_current_page->line_count = 4;
  // NOLINTEND(*-magic-numbers)
}

void ck_doc_init(void) {
  // Set up the documents
  cell_doc->direction = CK_DIRECTION_TRANSMIT;
  cell_doc->page_count = 2;
  cell_doc->pages[0] = cell_page0;
  cell_doc->pages[1] = cell_page1;

  reg_out_current_doc->direction = CK_DIRECTION_TRANSMIT;
  reg_out_current_doc->page_count = 1;
  reg_out_current_doc->pages[0] = reg_out_current_page;

  vbat_out_current_doc->direction = CK_DIRECTION_TRANSMIT;
  vbat_out_current_doc->page_count = 1;
  vbat_out_current_doc->pages[0] = vbat_out_current_page;
}

void ck_list_init(void) {
  // Set up the doc lists
  rx_list->type = CK_LIST_DOCUMENT;
  rx_list->direction = CK_DIRECTION_RECEIVE;
  rx_list->list_no = 0;
  rx_list->record_count = 1;  // Only 1 slot, for the king's doc.

  tx_list->type = CK_LIST_DOCUMENT;
  tx_list->direction = CK_DIRECTION_TRANSMIT;
  tx_list->list_no = 0;
  // We have 3 documents, and CK needs 1 slot for the mayor's doc.
  tx_list->record_count = 4;
  tx_list->records[1] = cell_doc;
  tx_list->records[2] = reg_out_current_doc;
  tx_list->records[3] = vbat_out_current_doc;
}

void ck_folder_init(void) {
  // Set up the folders
  cell_folder->direction = CK_DIRECTION_TRANSMIT;
  cell_folder->doc_list_no = 0;
  cell_folder->doc_no = 1;  // 0 reserved by mayor's doc
  cell_folder->enable = true;
  cell_folder->folder_no = 2;
  // 3 cells per page + 1 byte for pagination.
  cell_folder->dlc = 3 * sizeof(uint16_t) + 1;

  reg_out_current_folder->direction = CK_DIRECTION_TRANSMIT;
  reg_out_current_folder->doc_list_no = 0;
  reg_out_current_folder->doc_no = 2;
  reg_out_current_folder->enable = true;
  reg_out_current_folder->folder_no = 3;
  reg_out_current_folder->dlc = sizeof(uint16_t);

  vbat_out_current_folder->direction = CK_DIRECTION_TRANSMIT;
  vbat_out_current_folder->doc_list_no = 0;
  vbat_out_current_folder->doc_no = 3;
  vbat_out_current_folder->enable = true;
  vbat_out_current_folder->folder_no = 4;
  vbat_out_current_folder->dlc = sizeof(uint32_t);
}

void ck_init(void) {
  ck_page_init();
  ck_doc_init();
  ck_list_init();
  ck_folder_init();

  postmaster_init(
      &peripherals->common_peripherals->hcan);  // Set up the postmaster

  default_letter_timer = xTimerCreateStatic(
      "default letter timer", pdMS_TO_TICKS(200),
      pdFALSE,  // Auto reload timer
      NULL,     // Timer ID, unused
      default_letter_timer_callback, &default_letter_timer_buf);

  ck_mayor_t mayor = {
      .ean_no = 123,       // NOLINT(*-magic-numbers)
      .serial_no = 456,    // NOLINT(*-magic-numbers)
      .city_address = 13,  // NOLINT(*-magic-numbers)
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .start_200ms_timer = start_default_letter_timer,
      .folder_count = FOLDER_COUNT,
      .folders = folders,
      .list_count = LIST_COUNT,
      .lists = lists,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    print(&peripherals->common_peripherals->huart1,
          "Error setting up mayor.\r\n");
    error();
  }
}

void task_init(void) {
  battery_monitor_task = xTaskCreateStatic(
      battery_monitor, "battery monitor", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, battery_monitor_stack, &battery_monitor_buf);

  battery_report_task = xTaskCreateStatic(
      battery_report, "battery report", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY, battery_report_stack, &battery_report_buf);

  process_letter_task = xTaskCreateStatic(
      proc_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 2, proc_letter_stack, &proc_letter_buf);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(battery_monitor_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
  if (GPIO_PIN != OVER_CURRENT_PIN) {
    return;
  }
  over_current_fault = 1;
}

void battery_monitor_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(battery_monitor_task);
}

void battery_monitor(void *unused) {
  (void)unused;

  set_jumper_config(ALL_ON);
  config_voltage_regulator(&peripherals->hi2c1, POT_IVRA_DEFAULT);
  HAL_GPIO_WritePin(REG_PWR_ON_GPIO_PORT, REG_PWR_ON_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT, POWER_OFF_PIN, GPIO_PIN_SET);

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic(
      "battery monitor timer", pdMS_TO_TICKS(BATTERY_MONITOR_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      battery_monitor_timer, &timer_buf);

  xTimerStart(timer, portMAX_DELAY);

  battery_state_t battery_state;
  adc_reading_t adc_reading;
  battery_charge_t battery_charge = CHARGE_100_PERCENT;
  uint8_t low_power_report_count = 0;

  // TODO: implement set_fuse_config() to configure HW fuses,
  // use it to set over_current_threshold.
  //
  // TODO: calculate the maximum measured reg_out_current based on the jumper
  // config, use it to set a better over_current_threshold that considers the
  // reg_out_current separately from the vbat_out_current.
  //
  // We might detect false positives if the jumper config is set to detect low
  // currents but the actual currents are much higher. Nevertheless, the largest
  // part of the total current is the vbat_out_current.
  const uint32_t over_current_threshold = 49500;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adc_reading.adc1_buf,
                      sizeof(adc_reading.adc1_buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adc_reading.adc2_buf,
                      sizeof(adc_reading.adc2_buf) / sizeof(uint16_t));

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    parse_adc_values(&adc_reading, &battery_state);

    // Check if over current or low voltage protection has triggered.
    if (low_power_report_count > LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD ||
        over_current_fault != 0 ||
        battery_state.reg_out_current + battery_state.vbat_out_current >
            over_current_threshold) {
      // Turn off the power outputs to reduce the battery power drain.
      HAL_GPIO_WritePin(REG_PWR_ON_GPIO_PORT, REG_PWR_ON_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT, POWER_OFF_PIN, GPIO_PIN_RESET);
      // Blink LEDs red to show user something is wrong.
      blink_leds_red();
    } else {  // Only update the charge state if no fault has occured.
      battery_charge = read_battery_charge(&battery_state);
      low_power_report_count += set_charge_state_led(&battery_charge);
    }

    update_pages(&battery_state);
  }
}

void update_pages(battery_state_t *battery_state) {
  // Copy cells 0,1,2 to page 0 of cell doc, cells 3,4,5 to page 1.
  memcpy(&cell_page0->lines[1], &battery_state->cells[0], cell_folder->dlc);
  memcpy(&cell_page1->lines[1], &battery_state->cells[3], cell_folder->dlc);

  memcpy(reg_out_current_page->lines, &battery_state->reg_out_current,
         reg_out_current_folder->dlc);

  memcpy(vbat_out_current_page->lines, &battery_state->vbat_out_current,
         vbat_out_current_folder->dlc);
}

void send_docs(void) {
  if (ck_send_document(cell_folder->folder_no) != CK_OK) {
    print(&peripherals->common_peripherals->huart1, "failed to send doc.\r\n");
  }
  if (ck_send_document(reg_out_current_folder->folder_no) != CK_OK) {
    print(&peripherals->common_peripherals->huart1, "failed to send doc.\r\n");
  }
  if (ck_send_document(vbat_out_current_folder->folder_no) != CK_OK) {
    print(&peripherals->common_peripherals->huart1, "failed to send doc.\r\n");
  }
}

// Convert HAL CAN frame to ck_letter_t.
ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data) {
  ck_letter_t letter;
  letter.envelope.is_remote = header->RTR;
  letter.envelope.has_extended_id = header->IDE;
  if (letter.envelope.has_extended_id) {
    letter.envelope.envelope_no = header->ExtId;
  } else {
    letter.envelope.envelope_no = header->StdId;
  }
  letter.envelope.is_compressed = false;
  letter.page.line_count = header->DLC;
  memcpy(letter.page.lines, data, header->DLC);
  return letter;
}

// Deactivate interrupt, then signal task. Let the task reactivate the
// interrupt.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(process_letter_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void dispatch_letter(ck_letter_t *letter) {
  // Check for default letter
  if (ck_is_default_letter(letter) == CK_OK) {
    if (ck_default_letter_received() != CK_OK) {
      print(&peripherals->common_peripherals->huart1,
            "CAN Kingdom error in ck_default_letter_received().\r\n");
    }
  }
  // Check for king's letter
  if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      print(&peripherals->common_peripherals->huart1,
            "failed to process king's letter.\r\n");
    }
  }
  // TODO: implement other letters
}

void default_letter_timer_callback(TimerHandle_t timer) {
  (void)timer;

  if (ck_default_letter_timeout() != CK_OK) {
    print(&peripherals->common_peripherals->huart1,
          "CAN Kingdom error in ck_default_letter_timeout().\r\n");
  }
}

void start_default_letter_timer(void) {
  xTimerStart(default_letter_timer, portMAX_DELAY);
}

void proc_letter(void *unused) {
  (void)unused;

  CAN_RxHeaderTypeDef header;
  uint8_t data[CK_CAN_MAX_DLC];
  ck_letter_t letter;

  for (;;) {
    if (HAL_CAN_ActivateNotification(&peripherals->common_peripherals->hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      print(&peripherals->common_peripherals->huart1,
            "Error activating interrupt.\r\n");
      error();
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Process all messages
    while (HAL_CAN_GetRxMessage(&peripherals->common_peripherals->hcan,
                                CAN_RX_FIFO0, &header, data) == HAL_OK) {
      if (ck_correct_letter_received() != CK_OK) {
        print(&peripherals->common_peripherals->huart1,
              "CAN Kingdom error in ck_correct_letter_received().\r\n");
      }
      letter = frame_to_letter(&header, data);
      dispatch_letter(&letter);
    }
  }
}

void battery_report_timer(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(battery_report_task);
}

void battery_report(void *unused) {
  (void)unused;

  StaticTimer_t timer_buf;
  TimerHandle_t timer = xTimerCreateStatic(
      "battery report timer", pdMS_TO_TICKS(BATTERY_REPORT_TASK_PERIOD_MS),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      battery_report_timer, &timer_buf);

  xTimerStart(timer, portMAX_DELAY);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation
    send_docs();
  }
}
