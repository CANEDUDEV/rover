#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ck-data.h"
#include "clock.h"
#include "common-peripherals.h"
#include "device-id.h"
#include "error.h"
#include "lfs-config.h"

// CK
#include "mayor.h"
#include "postmaster-hal.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "timers.h"

#define LOWEST_TASK_PRIORITY 24

// CK
#define CK_FOLDER_COUNT 2
#define CK_LIST_COUNT 2

// Symbols defined in bootloader linker script
extern const uint32_t APPROM_START;
extern const uint32_t APPROM_SIZE;
// Used to store symbol values
static uint32_t approm_start;
static uint32_t approm_size;

// CK
static void mayor_init(void);
static ck_err_t set_action_mode(ck_action_mode_t mode);
static ck_err_t set_city_mode(ck_city_mode_t mode);

static StaticTimer_t default_letter_timer_buf;
static TimerHandle_t default_letter_timer;
static void default_letter_timer_callback(TimerHandle_t timer);
static void start_default_letter_timer(void);

static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];
static void process_letter(void *unused);
static void dispatch_letter(ck_letter_t *letter);
static int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

// Bootloader
static bool bootloader_entered = false;
static void enter_bootloader(const ck_letter_t *letter);
static void exit_bootloader(const ck_letter_t *letter);
static int process_flash_erase_letter(const ck_letter_t *letter);
static int process_flash_program_letter(const ck_letter_t *letter);

// Flag indicating HW failure in flash.
static bool flash_failed = false;

static TaskHandle_t flash_program_task;
static StaticTask_t flash_program_buf;
static StackType_t flash_program_stack[configMINIMAL_STACK_SIZE];
static StreamBufferHandle_t program_stream;
static StaticStreamBuffer_t program_stream_buf;
static uint8_t program_stream_storage[4096];  // NOLINT(*-magic-numbers)
static void flash_program(void *unused);
static void send_abort_page(void);

static void send_ack(const ck_letter_t *letter);
static void send_nack(const ck_letter_t *letter);

// NOLINTNEXTLINE(readability*,bugprone*)
static void start_app(uint32_t pc, uint32_t sp);

int main(void) {
  HAL_Init();
  system_clock_init();
  common_peripherals_init();

  flash_program_task = xTaskCreateStatic(
      flash_program, "flash program", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY, flash_program_stack, &flash_program_buf);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, process_letter_stack, &process_letter_buf);

  if (lfs_init() < 0) {
    printf("Error initializing littlefs.\r\n");
    error();
  }

  mayor_init();

  // Get values from symbol table
  approm_start = (uint32_t)&APPROM_START;
  approm_size = (uint32_t)&APPROM_SIZE;

  vTaskStartScheduler();

  while (1) {
    // Never reached
  }
}

static void mayor_init(void) {
  default_letter_timer = xTimerCreateStatic(
      "default letter timer", pdMS_TO_TICKS(200),
      pdFALSE,  // Don't auto reload timer
      NULL,     // Timer ID, unused
      default_letter_timer_callback, &default_letter_timer_buf);

  ck_data_init();
  ck_data_t *ck_data = get_ck_data();
  ck_id_t ck_id;

  if (read_ck_id(&ck_id) != APP_OK) {
    printf(
        "Failed to read ck_id, happens on first boot or corruption. "
        "Using default ck_id for this boot.\r\n");
    ck_id = get_default_ck_id(1);  // city_address == 1
  }

  ck_mayor_t mayor = {
      .ean_no = 100 + ck_id.city_address,  // NOLINT(*-magic-numbers)
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

  if (ck_add_mayors_page(ck_data->bootloader_page) != CK_OK) {
    printf("Error adding bootloader page to mayor.\r\n");
    error();
  }
}

// Exit bootloader if no attempted communication with bootloader.
static void default_letter_timer_callback(TimerHandle_t timer) {
  (void)timer;

  if (ck_default_letter_timeout() != CK_OK) {
    printf("CAN Kingdom error in ck_default_letter_timeout().\r\n");
  }

  if (!bootloader_entered) {
    exit_bootloader(NULL);
  }
}

static void start_default_letter_timer(void) {
  xTimerStart(default_letter_timer, portMAX_DELAY);
}

static ck_err_t set_action_mode(ck_action_mode_t mode) {
  (void)mode;
  return CK_OK;
}

static ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

static void process_letter(void *unused) {
  (void)unused;

  common_peripherals_t *peripherals = get_common_peripherals();

  CAN_RxHeaderTypeDef header;
  uint8_t data[CK_CAN_MAX_DLC];
  ck_letter_t letter;

  for (;;) {
    if (HAL_CAN_ActivateNotification(&peripherals->hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      printf("Error activating interrupt.\r\n");
      error();
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Process all messages
    while (HAL_CAN_GetRxMessage(&peripherals->hcan, CAN_RX_FIFO0, &header,
                                data) == HAL_OK) {
      if (ck_correct_letter_received() != CK_OK) {
        printf("CAN Kingdom error in ck_correct_letter_received().\r\n");
      }
      letter = frame_to_letter(&header, data);
      dispatch_letter(&letter);
    }
  }
}

static void dispatch_letter(ck_letter_t *letter) {
  ck_folder_t *folder = NULL;

  // Check for default letter
  if (ck_is_default_letter(letter) == CK_OK) {
    if (ck_default_letter_received() != CK_OK) {
      printf("CAN Kingdom error in ck_default_letter_received().\r\n");
    }
  }

  // Check for king's letter
  else if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      printf("failed to process king's letter.\r\n");
    }
  }

  // Check for any other letter
  else if (ck_get_envelopes_folder(&letter->envelope, &folder) == CK_OK) {
    if (handle_letter(folder, letter) != APP_OK) {
      printf("failed to process page.\r\n");
    }
  }
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();

  if (folder->folder_no == ck_data->enter_bootloader_folder->folder_no) {
    enter_bootloader(letter);
  }
  if (folder->folder_no == ck_data->exit_bootloader_folder->folder_no) {
    exit_bootloader(letter);
  }
  if (folder->folder_no == ck_data->flash_erase_folder->folder_no) {
    return process_flash_erase_letter(letter);
  }
  // The folder_no given could refer to either of the two folders, since they
  // share the same envelope.
  if (folder->folder_no == ck_data->flash_program_receive_folder->folder_no ||
      folder->folder_no == ck_data->flash_program_transmit_folder->folder_no) {
    return process_flash_program_letter(letter);
  }
  return APP_OK;
}

static void enter_bootloader(const ck_letter_t *letter) {
  printf("Entered bootloader.\r\n");
  bootloader_entered = true;
  send_ack(letter);
}

static void exit_bootloader(const ck_letter_t *letter) {
  // Send ACK with received CAN ID if triggered by flasher.
  if (letter != NULL) {
    send_ack(letter);
  }

  printf("Exiting bootloader...\r\n");

  // Release any resources used by littlefs
  int err = lfs_deinit();
  if (err < 0) {
    printf("lfs_deinit error: %d\r\n", err);
  }

  // Resets the peripheral buses, so no need for additional deinit.
  HAL_DeInit();

  // Disable interrupts
  __disable_irq();

  // Relocate vector table
  SCB->VTOR = approm_start;

  // Jump to app
  uint32_t *app_code = (uint32_t *)approm_start;
  uint32_t app_sp = app_code[0];
  uint32_t app_pc = app_code[1];

  start_app(app_pc, app_sp);
}

static int process_flash_erase_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();

  if (letter->page.line_count != ck_data->flash_erase_folder->dlc) {
    printf("Incorrect flash erase page length.\r\n");
    send_nack(letter);
    return APP_NOT_OK;
  }

  printf("Erasing flash...\r\n");

  // Get start and end addresses of erase operation
  uint32_t bytes_to_erase = 0;
  memcpy(&bytes_to_erase, letter->page.lines, sizeof(bytes_to_erase));

  // Align bytes to page size
  const uint32_t page_size = 2 * 1024;  // 1 page = 2KB
  if (bytes_to_erase % page_size != 0) {
    bytes_to_erase =
        (bytes_to_erase + page_size) - (bytes_to_erase % page_size);
  }

  // Check if erase within flash boundary
  if (bytes_to_erase > approm_size) {
    bytes_to_erase = approm_size;
  }

  // Erase flash
  const uint32_t flash_erase_ok = 0xFFFFFFFF;  // Defined by HAL

  FLASH_EraseInitTypeDef flash_erase_conf = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = approm_start,
      .NbPages = bytes_to_erase / page_size,
  };

  uint32_t err = 0;
  HAL_StatusTypeDef status = HAL_OK;

  taskENTER_CRITICAL();
  HAL_FLASH_Unlock();
  status = HAL_FLASHEx_Erase(&flash_erase_conf, &err);
  HAL_FLASH_Lock();
  taskEXIT_CRITICAL();

  if (err != flash_erase_ok || status != HAL_OK) {
    flash_failed = true;
    printf("Failed to erase flash.\r\n");
    send_nack(letter);
    return APP_NOT_OK;
  }

  send_ack(letter);

  printf("Finished erasing flash.\r\n");
  return APP_OK;
}

static int process_flash_program_letter(const ck_letter_t *letter) {
  static bool transfer_started = false;

  ck_data_t *ck_data = get_ck_data();

  // Fatal error, something is wrong with the flash.
  if (flash_failed) {
    printf("Fatal flash error. Aborting.\r\n");
    send_abort_page();
    transfer_started = false;  // Reset transfer state
    return APP_NOT_OK;
  }

  // Letter validation
  if (letter->page.line_count != ck_data->flash_program_receive_folder->dlc ||
      (letter->page.lines[0] != 1 && letter->page.lines[0] != 3 &&
       letter->page.lines[0] != 4)) {
    printf("Incorrect flash program letter. Aborting.\r\n");
    send_abort_page();
    transfer_started = false;  // Reset transfer state
    return APP_NOT_OK;
  }

  static uint32_t bytes_to_receive = 0;
  static uint32_t received_bytes = 0;
  static uint8_t expected_data_page = 0;

  // These symbols are defined by CK as part of the block transfer page.
  const uint16_t send_all = 0xFFFF;          // Request whole bundle
  const uint16_t receive_complete = 0x0000;  // Received bundle uccessfully.

  // Setup page
  // If the transfer has already been set up, we need to reset the state.
  if (letter->page.lines[0] == 1) {
    memcpy(&bytes_to_receive, &letter->page.lines[1], sizeof(bytes_to_receive));
    // Check if we can receive that many bytes
    if (bytes_to_receive > approm_size) {
      printf("Bytes to receive larger than approm size. Aborting.\r\n");
      send_abort_page();
      return APP_NOT_OK;
    }

    // Respond that we can receive the whole bundle at once.
    memcpy(&ck_data->flash_program_bundle_request_page->lines[1], &send_all,
           sizeof(send_all));
    if (ck_send_page(ck_data->flash_program_transmit_folder->folder_no,
                     ck_data->flash_program_bundle_request_page->lines[0]) !=
        CK_OK) {
      printf("Error sending block transfer bundle request page.\r\n");
      return APP_NOT_OK;
    }

    printf("Flashing app...\r\n");

    // Initial state for flashing
    transfer_started = true;
    received_bytes = 0;
    expected_data_page = 3;
    xTaskNotifyGive(flash_program_task);  // Activate flasher task
    return APP_OK;
  }

  if (!transfer_started) {
    printf("Block transfer not initialized, Aborting.\r\n");
    send_abort_page();
    return APP_NOT_OK;
  }

  // Handle information transport pages

  // Duplicate transmission, discard message
  if (letter->page.lines[0] != expected_data_page) {
    printf("Duplicate received, skipping.\r\n");
    return APP_OK;
  }

  // Alternate page number for information transport
  if (expected_data_page == 3) {
    expected_data_page = 4;
  } else {
    expected_data_page = 3;
  }

  // Send received bytes to stream. Each page always contains 7 bytes.
  const size_t bytes_per_page = 7;
  size_t bytes_to_write = bytes_per_page;
  if (received_bytes + bytes_to_write > bytes_to_receive) {
    bytes_to_write = bytes_to_receive - received_bytes;
  }

  received_bytes += xStreamBufferSend(program_stream, &letter->page.lines[1],
                                      bytes_to_write, portMAX_DELAY);

  // According to the CK specification, if the sender is transmitting less than
  // 7 bytes they should set the unused bytes in a page to 0. Thus, it's
  // possible to receive more bytes than wanted without considering it an
  // error.
  if (received_bytes >= bytes_to_receive) {
    // Wait for stream buffer to empty
    while (xStreamBufferIsEmpty(program_stream) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Send "reception complete" message
    memcpy(&ck_data->flash_program_bundle_request_page->lines[1],
           &receive_complete, sizeof(receive_complete));
    if (ck_send_page(ck_data->flash_program_transmit_folder->folder_no,
                     ck_data->flash_program_bundle_request_page->lines[0]) !=
        CK_OK) {
      printf("Error sending block transfer bundle request page.\r\n");
      return APP_NOT_OK;
    }

    printf("Finished flashing app.\r\n");
  }

  return APP_OK;
}

static void flash_program(void *unused) {
  (void)unused;

  // Set up the stream to which bytes will be buffered. Configure for reading
  // one word (4 bytes) at a time, since we will program the flash one word at a
  // time.

  uint8_t word[4];
  program_stream = xStreamBufferCreateStatic(
      sizeof(program_stream_storage) - 1, sizeof(word), program_stream_storage,
      &program_stream_buf);

  for (;;) {
    // Clear stream buffer
    while (xStreamBufferReset(program_stream) != pdPASS) {
      // Wait for successful clear of stream buffer.
    }

    // Reset parameters
    size_t read_bytes = 0;
    uint32_t current_address = approm_start;

    // Wait for task activation
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (;;) {
      memset(word, 0, sizeof(word));
      read_bytes = xStreamBufferReceive(program_stream, &word, sizeof(word),
                                        pdMS_TO_TICKS(100));

      if (read_bytes == 0) {
        // No more bytes to read, finish.
        break;
      }

      // Partial read
      if (read_bytes < sizeof(word)) {
        // Try to read the rest of the word
        xStreamBufferReceive(program_stream, &word[read_bytes],
                             sizeof(word) - read_bytes, pdMS_TO_TICKS(100));
      }

      // Program flash with word
      taskENTER_CRITICAL();
      HAL_FLASH_Unlock();
      HAL_StatusTypeDef status = HAL_OK;
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_address,
                                 *(uint64_t *)word);
      HAL_FLASH_Lock();
      taskEXIT_CRITICAL();

      if (status != HAL_OK) {
        ck_data_t *ck_data = get_ck_data();
        printf("Fatal error, flashing failed.\r\n");
        if (ck_send_page(ck_data->flash_program_transmit_folder->folder_no,
                         ck_data->flash_program_abort_page->lines[0]) !=
            CK_OK) {
          printf("Error sending block transfer abort page.\r\n");
        }
        flash_failed = true;
        break;
      }

      // Increment address
      current_address += sizeof(word);
    }
  }
}

static void send_abort_page(void) {
  ck_data_t *ck_data = get_ck_data();
  if (ck_send_page(ck_data->flash_program_transmit_folder->folder_no,
                   ck_data->flash_program_abort_page->lines[0]) != CK_OK) {
    printf("Error sending block transfer abort page.\r\n");
  }
}

static void send_ack(const ck_letter_t *letter) {
  // Send ACK with received CAN ID
  ck_data_t *ck_data = get_ck_data();
  ck_data->command_ack_page->lines[0] = 0;  // ACK
  memcpy(&ck_data->command_ack_page->lines[1], &letter->envelope.envelope_no,
         sizeof(letter->envelope.envelope_no));

  // Don't raise error here since this is an error on the flasher's part.
  if (ck_send_document(ck_data->command_ack_folder->folder_no) != CK_OK) {
    printf("Error sending ACK with ID 0x%x.\r\n", letter->envelope.envelope_no);
  }
}

static void send_nack(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  ck_data->command_ack_page->lines[0] = 1;  // NACK
  memcpy(&ck_data->command_ack_page->lines[1], &letter->envelope.envelope_no,
         sizeof(letter->envelope.envelope_no));

  // Don't raise error here since this is an error on the flasher's part.
  if (ck_send_document(ck_data->command_ack_folder->folder_no) != CK_OK) {
    printf("Error sending NACK with ID 0x%x.\r\n",
           letter->envelope.envelope_no);
  }
}

// Loads sp into Main Stack Pointer (MSP) register and pc into PC register,
// causing the application to jump to PC.
//
// NOLINTNEXTLINE(readability*,bugprone*)
static void start_app(uint32_t pc, uint32_t sp) {
  __asm(
      "msr msp, %[sp] /* load sp into MSP register */\n"
      "mov pc, %[pc]  /* load pc into PC register */\n"
      :
      : [sp] "r"(sp), [pc] "r"(pc));  // Replace [sp] by sp, and [pc] by pc.
}

// Deactivate interrupt, then signal task. Let the task reactivate the
// interrupt.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(process_letter_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN) {
    can_msp_init();
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN) {
    can_msp_deinit();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_init();
  }
  if (hspi->Instance == SPI2) {
    spi2_msp_init();
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_deinit();
  }
  if (hspi->Instance == SPI2) {
    spi2_msp_deinit();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uart1_msp_init();
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uart1_msp_deinit();
  }
}
