#include <stdio.h>
#include <string.h>

// FreeRTOS includes
#include "cmsis_os.h"
#include "timers.h"

// Our own includes
#include "app.h"
#include "mayor.h"
#include "peripherals.h"
#include "ports.h"
#include "postmaster-hal.h"
#include "utils.h"

#define MEASURE_TASK_PERIOD_MS 100
#define DEFAULT_TASK_PERIOD_MS 200

#define CAN_BASE_ID 0x100

#define LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD 10

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static osThreadId_t measureTaskHandle;
static const osThreadAttr_t measureTask_attributes = {
    .name = "measureTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static BatteryNodeState batteryNodeState;

static peripherals_t *peripherals;
// Set to 1 by GPIO external interrupt on the OVER_CURRENT pin.
static uint8_t OverCurrentFault = 0;

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

// Function prototypes
void system_clock_init(void);
void StartDefaultTask(void *argument);

// Init CAN kingdom stack
void mayor_init(void);

void StartMeasureTask(void *argument);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  if (FlashRWInit() != APP_OK) {
    Error_Handler();
  }

  /* Configure the system clock */
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

  /* Init scheduler */
  osKernelInitialize();

  /* Create the thread(s) */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  measureTaskHandle =
      osThreadNew(StartMeasureTask, NULL, &measureTask_attributes);

  Print(&peripherals->huart1, "Starting application...\r\n");

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */

  while (1) {
  }
}

void system_clock_init(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
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

void mayor_init(void) {
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

  // Set up the lists
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

  // Set up the folders
  cell_folder->direction = CK_DIRECTION_TRANSMIT;
  cell_folder->doc_list_no = 0;
  cell_folder->doc_no = 1;  // 0 reserved by mayor's doc
  cell_folder->enable = true;
  cell_folder->folder_no = 2;
  // 3 cells per page + 1 byte for pagination.
  cell_folder->dlc = 3 * sizeof(batteryNodeState.cells[0]) + 1;

  reg_out_current_folder->direction = CK_DIRECTION_TRANSMIT;
  reg_out_current_folder->doc_list_no = 0;
  reg_out_current_folder->doc_no = 2;
  reg_out_current_folder->enable = true;
  reg_out_current_folder->folder_no = 3;
  reg_out_current_folder->dlc = sizeof(batteryNodeState.regOutCurrent);

  vbat_out_current_folder->direction = CK_DIRECTION_TRANSMIT;
  vbat_out_current_folder->doc_list_no = 0;
  vbat_out_current_folder->doc_no = 3;
  vbat_out_current_folder->enable = true;
  vbat_out_current_folder->folder_no = 4;
  vbat_out_current_folder->dlc = sizeof(batteryNodeState.vbatOutCurrent);

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

  peripherals_t *peripherals = get_peripherals();
  postmaster_init(&peripherals->hcan);  // Set up the postmaster

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
    Error_Handler();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  NotifyTask(measureTaskHandle);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin != OVER_CURRENT_Pin) {
    return;
  }
  OverCurrentFault = 1;
}

void defaultTaskTimer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  NotifyTask(defaultTaskHandle);
}

void measureTaskTimer(TimerHandle_t xTimer) {
  UNUSED(xTimer);
  NotifyTask(measureTaskHandle);
}

void StartMeasureTask(void *argument) {
  UNUSED(argument);

  TimerHandle_t xTimer =
      xTimerCreate("measureTaskTimer", pdMS_TO_TICKS(MEASURE_TASK_PERIOD_MS),
                   pdTRUE,  // Auto reload timer
                   NULL,    // Timer ID, unused
                   measureTaskTimer);

  xTimerStart(xTimer, portMAX_DELAY);

  ADCReading adcBuf;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adcBuf.adc1Buf,
                      sizeof(adcBuf.adc1Buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adcBuf.adc2Buf,
                      sizeof(adcBuf.adc2Buf) / sizeof(uint16_t));

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    ParseADCValues(&adcBuf, &batteryNodeState);

    // Copy cells 0,1,2 to page 0 of cell doc, cells 3,4,5 to page 1.
    memcpy(&cell_page0->lines[1], &batteryNodeState.cells[0], cell_folder->dlc);
    memcpy(&cell_page1->lines[1], &batteryNodeState.cells[3], cell_folder->dlc);

    memcpy(reg_out_current_page->lines, &batteryNodeState.regOutCurrent,
           reg_out_current_folder->dlc);

    memcpy(vbat_out_current_page->lines, &batteryNodeState.vbatOutCurrent,
           vbat_out_current_folder->dlc);

    // Send the documents
    if (ck_send_document(cell_folder->folder_no) != CK_OK) {
      Error_Handler();
    }
    if (ck_send_document(reg_out_current_folder->folder_no) != CK_OK) {
      Error_Handler();
    }
    if (ck_send_document(vbat_out_current_folder->folder_no) != CK_OK) {
      Error_Handler();
    }
  }
}

void StartDefaultTask(void *argument) {
  UNUSED(argument);

  SetJumperConfig(ALL_ON);

  ConfigureVoltageRegulator(&peripherals->hi2c1, POT_IVRA_DEFAULT);
  HAL_GPIO_WritePin(REG_PWR_ON_GPIO_Port, REG_PWR_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(POWER_OFF_GPIO_Port, POWER_OFF_Pin, GPIO_PIN_SET);

  TimerHandle_t xTimer =
      xTimerCreate("defaultTaskTimer", pdMS_TO_TICKS(DEFAULT_TASK_PERIOD_MS),
                   pdTRUE,  // Auto reload timer
                   NULL,    // Timer ID, unused
                   defaultTaskTimer);

  xTimerStart(xTimer, portMAX_DELAY);

  BatteryCharge charge = CHARGE_100_PERCENT;
  uint8_t lowPowerReportCount = 0;

  HAL_CAN_Start(&peripherals->hcan);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Don't try to update charge state after low voltage cutoff
    // or over current fault.
    if (lowPowerReportCount > LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD ||
        OverCurrentFault != 0) {
      // Turn off the power outputs to reduce the battery power drain.
      HAL_GPIO_WritePin(REG_PWR_ON_GPIO_Port, REG_PWR_ON_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(POWER_OFF_GPIO_Port, POWER_OFF_Pin, GPIO_PIN_RESET);
      // Blink LEDs red to show user something is wrong.
      BlinkLEDsRed();
      continue;
    }

    charge = ReadBatteryCharge(&batteryNodeState);
    lowPowerReportCount += SetChargeStateLED(&charge);
  }
}
