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
#define BATTERY_MONITOR_TASK_PERIOD 100
#define LOWEST_TASK_PRIORITY 24

static TaskHandle_t battery_monitor_task;
static StaticTask_t battery_monitor_buf;
static StackType_t battery_monitor_stack[configMINIMAL_STACK_SIZE];
void battery_monitor(void *unused);

// CAN Kingdom process received letters task
static TaskHandle_t proc_letter_task;
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

// Set to true when 200ms has passed without receiving a default letter.
static bool default_letter_timeout = false;
static bool default_letter_received = false;
static bool base_number_known = false;

void ck_data_init(void);
void mayor_init(void);
void update_pages(BatteryNodeState *batteryNodeState);
void send_docs(void);

// Application
#define CAN_BASE_ID 0x100
#define LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD 10

// Set to 1 by GPIO external interrupt on the OVER_CURRENT pin.
static uint8_t OverCurrentFault = 0;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  if (flash_init() != APP_OK) {
    error();
  }

  system_clock_init();

  // Initialize all configured peripherals
  peripherals = get_peripherals();
  gpio_init();
  dma_init();
  can_init();
  uart1_init();
  spi1_init();
  adc1_init();
  adc2_init();
  i2c1_init();

  mayor_init();
  task_init();

  print(&peripherals->huart1, "Starting application...\r\n");

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

void ck_data_init(void) {
  ck_page_init();
  ck_doc_init();
  ck_list_init();
  ck_folder_init();

  // TODO: Set envelopes using king's pages.
  cell_folder->envelope_count = 1;
  cell_folder->envelopes[0].envelope_no = CAN_BASE_ID;
  cell_folder->envelopes[0].enable = true;
  reg_out_current_folder->envelope_count = 1;
  reg_out_current_folder->envelopes[0].envelope_no = CAN_BASE_ID + 1;
  reg_out_current_folder->envelopes[0].enable = true;
  vbat_out_current_folder->envelope_count = 1;
  vbat_out_current_folder->envelopes[0].envelope_no = CAN_BASE_ID + 2;
  vbat_out_current_folder->envelopes[0].enable = true;
}

void mayor_init(void) {
  ck_data_init();
  peripherals_t *peripherals = get_peripherals();
  postmaster_init(&(peripherals->hcan));  // Set up the postmaster

  ck_mayor_t mayor = {
      .ean_no = 123,       // NOLINT(*-magic-numbers)
      .serial_no = 456,    // NOLINT(*-magic-numbers)
      .city_address = 13,  // NOLINT(*-magic-numbers)
      .base_no = 25,       // NOLINT(*-magic-numbers)
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .folder_count = 5,  // NOLINT(*-magic-numbers)
      .folders = folders,
      .list_count = 2,
      .lists = lists,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    print(&peripherals->huart1, "Error setting up mayor.\r\n");
    error();
  }

  // Go on bus in silent mode
  if (ck_set_comm_mode(CK_COMM_MODE_SILENT) != CK_OK) {
    print(&peripherals->huart1, "Error setting comm mode.\r\n");
    error();
  }
}

void task_init(void) {
  battery_monitor_task = xTaskCreateStatic(
      battery_monitor, "battery monitor", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY, battery_monitor_stack, &battery_monitor_buf);

  proc_letter_task = xTaskCreateStatic(
      proc_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, proc_letter_stack, &proc_letter_buf);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(battery_monitor_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin != OVER_CURRENT_Pin) {
    return;
  }
  OverCurrentFault = 1;
}

void battery_monitor_timer(TimerHandle_t xTimer) {
  (void)xTimer;
  xTaskNotifyGive(battery_monitor_task);
}

void battery_monitor(void *unused) {
  (void)unused;

  SetJumperConfig(ALL_ON);
  ConfigureVoltageRegulator(&peripherals->hi2c1, POT_IVRA_DEFAULT);
  HAL_GPIO_WritePin(REG_PWR_ON_GPIO_Port, REG_PWR_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(POWER_OFF_GPIO_Port, POWER_OFF_Pin, GPIO_PIN_SET);

  StaticTimer_t timer_buf;
  TimerHandle_t xTimer = xTimerCreateStatic(
      "battery monitor timer", pdMS_TO_TICKS(BATTERY_MONITOR_TASK_PERIOD),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      battery_monitor_timer, &timer_buf);

  xTimerStart(xTimer, portMAX_DELAY);

  BatteryNodeState batteryNodeState;
  ADCReading adcBuf;
  BatteryCharge charge = CHARGE_100_PERCENT;
  uint8_t lowPowerReportCount = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adcBuf.adc1Buf,
                      sizeof(adcBuf.adc1Buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adcBuf.adc2Buf,
                      sizeof(adcBuf.adc2Buf) / sizeof(uint16_t));

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    ParseADCValues(&adcBuf, &batteryNodeState);

    // Check if over current or low voltage protection has triggered.
    if (lowPowerReportCount > LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD ||
        OverCurrentFault != 0) {
      // Turn off the power outputs to reduce the battery power drain.
      HAL_GPIO_WritePin(REG_PWR_ON_GPIO_Port, REG_PWR_ON_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(POWER_OFF_GPIO_Port, POWER_OFF_Pin, GPIO_PIN_RESET);
      // Blink LEDs red to show user something is wrong.
      BlinkLEDsRed();
    } else {  // Only update the charge state if no fault has occured.
      charge = ReadBatteryCharge(&batteryNodeState);
      lowPowerReportCount += SetChargeStateLED(&charge);
    }

    update_pages(&batteryNodeState);
    send_docs();
  }
}

void update_pages(BatteryNodeState *batteryNodeState) {
  // Copy cells 0,1,2 to page 0 of cell doc, cells 3,4,5 to page 1.
  memcpy(&cell_page0->lines[1], &batteryNodeState->cells[0], cell_folder->dlc);
  memcpy(&cell_page1->lines[1], &batteryNodeState->cells[3], cell_folder->dlc);

  memcpy(reg_out_current_page->lines, &batteryNodeState->regOutCurrent,
         reg_out_current_folder->dlc);

  memcpy(vbat_out_current_page->lines, &batteryNodeState->vbatOutCurrent,
         vbat_out_current_folder->dlc);
}

void send_docs(void) {
  if (ck_send_document(cell_folder->folder_no) != CK_OK) {
    print(&peripherals->huart1, "failed to send doc.\r\n");
  }
  if (ck_send_document(reg_out_current_folder->folder_no) != CK_OK) {
    print(&peripherals->huart1, "failed to send doc.\r\n");
  }
  if (ck_send_document(vbat_out_current_folder->folder_no) != CK_OK) {
    print(&peripherals->huart1, "failed to send doc.\r\n");
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
  letter.page.line_count = header->DLC;
  memcpy(letter.page.lines, data, header->DLC);
  return letter;
}

// Deactivate interrupt, then signal task. Let the task reactivate the
// interrupt.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(proc_letter_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool is_default_letter(ck_letter_t *letter) {
  ck_letter_t dletter = default_letter();
  if (letter->envelope.envelope_no == dletter.envelope.envelope_no &&
      letter->page.line_count == dletter.page.line_count &&
      memcmp(letter->page.lines, dletter.page.lines, dletter.page.line_count) ==
          0) {
    return true;
  }
  return false;
}

void dispatch_letter(ck_letter_t *letter) {
  // Check for king's letter
  if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      print(&peripherals->huart1, "failed to process king's letter.\r\n");
    }
    return;
  }
  // TODO: implement other letters
}

void default_letter_timer(TimerHandle_t timer) {
  (void)timer;
  default_letter_timeout = true;

  // 2b
  if (!default_letter_received) {
    // TODO: Set bit timing registers from flash
    if (ck_set_comm_mode(CK_COMM_MODE_SILENT) != CK_OK) {
      print(&peripherals->huart1, "Error setting comm mode.\r\n");
      error();
    }
    print(&peripherals->huart1, "2b.\r\n");
  }
}

bool is_startup_finished(bool startup_finished, bool letter_received) {
  // Check for correctly received letter
  if (startup_finished) {
    return true;
  }
  if (letter_received) {
    // 3a
    if (base_number_known) {
      if (ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) != CK_OK) {
        print(&peripherals->huart1, "Error setting comm mode.\r\n");
        error();
      }
      if (ck_send_mayors_page(0) != CK_OK) {
        print(&peripherals->huart1, "Error sending mayor's page.\r\n");
        return false;
      }
      print(&peripherals->huart1, "3a.\r\n");
    }
    // 3b
    else {
      if (ck_set_comm_mode(CK_COMM_MODE_LISTEN_ONLY) != CK_OK) {
        print(&peripherals->huart1, "Error setting comm mode.\r\n");
        error();
      }
      print(&peripherals->huart1, "3b.\r\n");
    }
    return true;
  }
  return false;
}

void proc_letter(void *unused) {
  (void)unused;

  // Timer to check for default letter. 200ms according to CAN Kingdom spec.
  StaticTimer_t timer_buf;
  TimerHandle_t timer =
      xTimerCreateStatic("default letter timer", pdMS_TO_TICKS(200),
                         pdFALSE,  // Auto reload timer
                         NULL,     // Timer ID, unused
                         default_letter_timer, &timer_buf);

  xTimerStart(timer, portMAX_DELAY);

  CAN_RxHeaderTypeDef header;
  uint8_t data[CK_CAN_MAX_DLC];
  ck_letter_t letter;

  // Set to true when a CAN frame has been received correctly,
  // indicating compatible bus parameters.
  bool letter_received = false;

  bool startup_finished = false;

  for (;;) {
    if (HAL_CAN_ActivateNotification(&peripherals->hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      print(&peripherals->huart1, "Error activating interrupt.\r\n");
      error();
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Process all messages
    while (HAL_CAN_GetRxMessage(&peripherals->hcan, CAN_RX_FIFO0, &header,
                                data) == HAL_OK) {
      letter_received = true;
      letter = frame_to_letter(&header, data);

      // Check for default letter
      // 2a
      if (!default_letter_received && !default_letter_timeout &&
          is_default_letter(&letter)) {
        default_letter_received = true;
        if (ck_set_comm_mode(CK_COMM_MODE_LISTEN_ONLY) != CK_OK) {
          print(&peripherals->huart1, "Error setting comm mode.\r\n");
          error();
        }
        startup_finished = true;
        print(&peripherals->huart1, "default letter received.\r\n");
        break;
      }

      dispatch_letter(&letter);
    }

    startup_finished = is_startup_finished(startup_finished, letter_received);
  }
}
