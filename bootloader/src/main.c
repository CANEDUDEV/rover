#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ck-data.h"

// STM32Common
#include "clock.h"
#include "common-peripherals.h"
#include "device-id.h"
#include "error.h"
#include "letter-reader.h"
#include "lfs-wrapper.h"

// JSON
#include "json.h"
#include "jsondb.h"

// CK
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
#include "timers.h"

// Symbols defined in bootloader linker script
extern const uint32_t APPROM_START;
extern const uint32_t APPROM_SIZE;
// Used to store symbol values
static uint32_t approm_start;
static uint32_t approm_size;

// These symbols are defined by CK as part of the block transfer page.
enum bundle_request_command {
  BUNDLE_RECEIVE_COMPLETE = 0,
  BUNDLE_REQUEST_WHOLE = 0xFFFF,
};

enum block_type {
  BLOCK_PROGRAM_DATA,
  BLOCK_CONFIG_DATA,
};

// CK
static void mayor_init(void);
static ck_err_t set_action_mode(ck_action_mode_t mode);
static ck_err_t set_city_mode(ck_city_mode_t mode);

static StaticTimer_t default_letter_timer_buf;
static TimerHandle_t default_letter_timer;
static void default_letter_timer_callback(TimerHandle_t timer);
static void start_default_letter_timer(void);

static int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

// Bootloader
static bool bootloader_entered = false;
static void enter_bootloader(const ck_letter_t *letter);
static void exit_bootloader(const ck_letter_t *letter);
static int process_flash_erase_letter(const ck_letter_t *letter);
static int process_fs_format_letter(const ck_letter_t *letter);
static int process_block_transfer(const ck_letter_t *letter,
                                  enum block_type block_type);

const uint8_t receive_timeout_ms = 100;
static bool transfer_in_progress = false;
static bool transfer_failed = false;

#define WRITE_CONFIG_STACK_SIZE (4 * configMINIMAL_STACK_SIZE)

char *flash_program_task_name = "flash program";
static TaskHandle_t flash_program_task;
static StaticTask_t flash_program_buf;
static StackType_t flash_program_stack[configMINIMAL_STACK_SIZE];

char *write_config_task_name = "write config";
static TaskHandle_t write_config_task;
static StaticTask_t write_config_buf;
static StackType_t write_config_stack[WRITE_CONFIG_STACK_SIZE];

static StreamBufferHandle_t byte_stream;
static StaticStreamBuffer_t byte_stream_buf;
static uint8_t byte_stream_storage[32 * 1024];  // NOLINT(*-magic-numbers)
static void flash_program(void *unused);
static void write_config(void *unused);

static void send_abort_page(uint8_t folder_no);
static int send_bundle_request_page(uint8_t folder_no,
                                    enum bundle_request_command command);
static void send_ack(const ck_letter_t *letter);
static void send_nack(const ck_letter_t *letter);

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static void start_app(uint32_t pc, uint32_t sp);

int main(void) {
  HAL_Init();
  system_clock_init();
  common_peripherals_init();

  uint8_t priority = LOWEST_TASK_PRIORITY;

  if (init_lfs_task(priority++) < 0) {
    error();
  }

  flash_program_task = xTaskCreateStatic(
      flash_program, flash_program_task_name, configMINIMAL_STACK_SIZE, NULL,
      priority++, flash_program_stack, &flash_program_buf);

  write_config_task = xTaskCreateStatic(
      write_config, write_config_task_name, WRITE_CONFIG_STACK_SIZE, NULL,
      priority++, write_config_stack, &write_config_buf);

  letter_reader_cfg_t letter_reader_cfg = {
      .priority = priority++,
      .app_letter_handler_func = handle_letter,
  };

  if (init_letter_reader_task(letter_reader_cfg) != APP_OK) {
    error();
  }

  jsondb_init();

  mayor_init();

  byte_stream = xStreamBufferCreateStatic(
      sizeof(byte_stream_storage),
      sizeof(uint32_t),  // Set trigger level to word size (4 bytes),
                         // since flash is programmed one word at a time.
      byte_stream_storage, &byte_stream_buf);

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

  if (!bootloader_entered) {
    exit_bootloader(NULL);
  }
}

static void start_default_letter_timer(void) {
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

static ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();

  if (folder->folder_no == ck_data->enter_bootloader_folder->folder_no) {
    enter_bootloader(letter);
  }
  if (folder->folder_no == ck_data->exit_bootloader_folder->folder_no) {
    exit_bootloader(letter);
  }

  // If in silent mode, we should not act on the flashing letters. This makes it
  // possible for a flasher app to selectively flash nodes on a bus.
  if (ck_get_comm_mode() == CK_COMM_MODE_SILENT) {
    return APP_OK;
  }

  if (folder->folder_no == ck_data->flash_erase_folder->folder_no) {
    return process_flash_erase_letter(letter);
  }

  if (folder->folder_no == ck_data->fs_format_folder->folder_no) {
    return process_fs_format_letter(letter);
  }

  // The folder_no given could refer to either of the two folders that are part
  // of the same block transfer document, since they share the same envelope.
  if (folder->folder_no == ck_data->program_receive_folder->folder_no ||
      folder->folder_no == ck_data->program_transmit_folder->folder_no) {
    return process_block_transfer(letter, BLOCK_PROGRAM_DATA);
  }

  if (folder->folder_no == ck_data->config_receive_folder->folder_no ||
      folder->folder_no == ck_data->config_transmit_folder->folder_no) {
    return process_block_transfer(letter, BLOCK_CONFIG_DATA);
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
  lfs_deinit();

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

  printf("flash erase: starting...\r\n");

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

  HAL_FLASH_Unlock();
  status = HAL_FLASHEx_Erase(&flash_erase_conf, &err);
  HAL_FLASH_Lock();

  if (err != flash_erase_ok || status != HAL_OK) {
    printf("flash erase: fatal flash error. Aborting.\r\n");
    send_nack(letter);
    return APP_NOT_OK;
  }

  send_ack(letter);

  printf("flash erase: finished.\r\n");
  return APP_OK;
}

static int process_fs_format_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();

  if (letter->page.line_count != ck_data->fs_format_folder->dlc) {
    printf("fs format: incorrect page length.\r\n");
    send_nack(letter);
    return APP_NOT_OK;
  }

  printf("fs format: starting...\r\n");

  if (format_and_mount() != APP_OK) {
    printf("fs format: fatal fs error. Aborting.\r\n");
    send_nack(letter);
    return APP_NOT_OK;
  }

  send_ack(letter);

  printf("fs format: finished.\r\n");

  return APP_OK;
}

static int process_block_transfer(const ck_letter_t *letter,
                                  enum block_type block_type) {
  ck_data_t *ck_data = get_ck_data();

  uint32_t folder_no = 0;
  uint32_t dlc = 0;
  size_t max_data_size = 0;
  void *task = NULL;
  char *task_name = NULL;

  switch (block_type) {
    case BLOCK_PROGRAM_DATA:
      folder_no = ck_data->program_transmit_folder->folder_no;
      dlc = ck_data->program_transmit_folder->dlc;
      task = flash_program_task;
      task_name = flash_program_task_name;
      max_data_size = approm_size;
      break;
    case BLOCK_CONFIG_DATA:
      folder_no = ck_data->config_transmit_folder->folder_no;
      dlc = ck_data->config_transmit_folder->dlc;
      task = write_config_task;
      task_name = write_config_task_name;
      max_data_size = get_jsondb_max_size();
      break;
  }

  // Letter validation
  if (letter->page.line_count != dlc ||
      (letter->page.lines[0] != 1 && letter->page.lines[0] != 3 &&
       letter->page.lines[0] != 4)) {
    printf("%s: incorrect letter. Aborting.\r\n", task_name);
    send_abort_page(folder_no);
    transfer_in_progress = false;  // Reset transfer state
    return APP_NOT_OK;
  }

  static uint32_t bytes_to_receive = 0;
  static uint32_t received_bytes = 0;
  static uint8_t expected_data_page = 0;

  // Setup page
  // If the transfer has already been set up, we need to reset the state.
  if (letter->page.lines[0] == 1) {
    memcpy(&bytes_to_receive, &letter->page.lines[1], sizeof(bytes_to_receive));
    // Check if we can receive that many bytes
    if (bytes_to_receive > approm_size) {
      printf("%s: bytes to receive larger than max size: %d. Aborting.\r\n",
             task_name, max_data_size);
      send_abort_page(folder_no);
      return APP_NOT_OK;
    }

    // Respond that we can receive the whole bundle at once.
    if (send_bundle_request_page(folder_no, BUNDLE_REQUEST_WHOLE) != APP_OK) {
      return APP_NOT_OK;
    }

    printf("%s: starting...\r\n", task_name);

    // Initial state for flashing
    transfer_in_progress = true;
    transfer_failed = false;
    received_bytes = 0;
    expected_data_page = 3;
    xTaskNotifyGive(task);  // Activate flasher task
    return APP_OK;
  }

  if (!transfer_in_progress) {
    printf("%s: block transfer not initialized, Aborting.\r\n", task_name);
    send_abort_page(folder_no);
    return APP_NOT_OK;
  }

  // Handle information transport pages

  // Duplicate transmission, discard message
  if (letter->page.lines[0] != expected_data_page) {
    printf("%s: duplicate received, skipping.\r\n", task_name);
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

  received_bytes += xStreamBufferSend(byte_stream, &letter->page.lines[1],
                                      bytes_to_write, portMAX_DELAY);

  // According to the CK specification, if the sender is transmitting less than
  // 7 bytes they should set the unused bytes in a page to 0. Thus, it's
  // possible to receive more bytes than wanted without considering it an
  // error.
  if (received_bytes >= bytes_to_receive) {
    // Wait for flasher/writer task to signal that it's finished
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) != pdPASS) {
      // The timeout is long, if task is not finished by now something fishy is
      // going on.
      transfer_failed = true;
    }

    if (!transfer_failed) {
      if (send_bundle_request_page(folder_no, BUNDLE_RECEIVE_COMPLETE) !=
          APP_OK) {
        return APP_NOT_OK;
      }
      printf("%s: finished.\r\n", task_name);
    } else {
      printf("%s: failed.\r\n", task_name);
    }
  }

  return APP_OK;
}

static void flash_program(void *unused) {
  (void)unused;

  ck_data_t *ck_data = get_ck_data();

  uint8_t word[4];

  for (;;) {
    xStreamBufferReset(byte_stream);

    // Reset parameters
    size_t read_bytes = 0;
    uint32_t current_address = approm_start;

    // Wait for task activation
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    HAL_FLASH_Unlock();

    for (;;) {
      memset(word, 0, sizeof(word));
      read_bytes = xStreamBufferReceive(byte_stream, &word, sizeof(word),
                                        pdMS_TO_TICKS(receive_timeout_ms));

      if (read_bytes == 0) {
        // No more bytes to read, finish.
        break;
      }

      // Partial read
      if (read_bytes < sizeof(word)) {
        // Try to read the rest of the word
        xStreamBufferReceive(byte_stream, &word[read_bytes],
                             sizeof(word) - read_bytes,
                             pdMS_TO_TICKS(receive_timeout_ms));
      }

      // Program flash with word
      HAL_StatusTypeDef status = HAL_OK;
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_address,
                                 *(uint32_t *)word);

      if (status != HAL_OK) {
        printf("%s: fatal error: flashing failed.\r\n",
               flash_program_task_name);
        send_abort_page(ck_data->program_transmit_folder->folder_no);
        transfer_failed = true;
        break;
      }

      // Increment address
      current_address += sizeof(word);
    }

    HAL_FLASH_Lock();
    transfer_in_progress = false;
    xTaskNotifyGive(get_letter_reader_task_handle());
  }
}

static void write_config(void *unused) {
  (void)unused;

  ck_data_t *ck_data = get_ck_data();

  char *config_storage = get_jsondb_raw();

  for (;;) {
    memset(config_storage, 0, get_jsondb_max_size());
    xStreamBufferReset(byte_stream);

    size_t read_bytes = 0;
    size_t written_bytes = 0;
    const size_t chunk_size = 7;  // Payload bytes per CAN frame.
    uint8_t buf[chunk_size];

    // Wait for task activation
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (;;) {
      // Don't read into config_storage directly, to avoid out of bounds access.
      read_bytes = xStreamBufferReceive(byte_stream, buf, chunk_size,
                                        pdMS_TO_TICKS(receive_timeout_ms));

      if (read_bytes == 0) {
        // No more bytes to read, finish.
        break;
      }

      if (written_bytes + read_bytes >= get_jsondb_max_size()) {
        printf("%s: config too large. Aborting.\r\n", write_config_task_name);
        send_abort_page(ck_data->config_transmit_folder->folder_no);
        transfer_failed = true;
        break;
      }

      memcpy(config_storage + written_bytes, buf, read_bytes);
      written_bytes += read_bytes;
    }

    json_object_t *json = NULL;
    if (!transfer_failed) {
      json = json_parse(config_storage);
      if (!json) {
        printf("%s: invalid JSON received: \r\n%s.\r\n", write_config_task_name,
               config_storage);
        send_abort_page(ck_data->config_transmit_folder->folder_no);
        transfer_failed = true;
      }
    }

    if (json) {
      if (jsondb_update(json) != APP_OK) {
        printf("%s: fatal error: couldn't write config to FS. Aborting.\r\n",
               write_config_task_name);
        send_abort_page(ck_data->config_transmit_folder->folder_no);
        transfer_failed = true;
      }
    }

    transfer_in_progress = false;
    xTaskNotifyGive(get_letter_reader_task_handle());
  }
}

static void send_abort_page(uint8_t folder_no) {
  ck_data_t *ck_data = get_ck_data();
  if (ck_send_page(folder_no, ck_data->abort_page->lines[0]) != CK_OK) {
    printf("Error sending block transfer abort page.\r\n");
  }
}

// NOLINTNEXTLINE(readability*,bugprone*)
static int send_bundle_request_page(uint8_t folder_no,
                                    enum bundle_request_command command) {
  ck_data_t *ck_data = get_ck_data();
  uint16_t page_data = (uint16_t)command;
  memcpy(&ck_data->bundle_request_page->lines[1], &page_data,
         sizeof(page_data));
  if (ck_send_page(folder_no, ck_data->bundle_request_page->lines[0]) !=
      CK_OK) {
    printf("Error sending block transfer bundle request page.\r\n");
    return APP_NOT_OK;
  }
  return APP_OK;
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
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static void start_app(uint32_t pc, uint32_t sp) {
  __asm(
      "msr msp, %[sp] /* load sp into MSP register */\n"
      "mov pc, %[pc]  /* load pc into PC register */\n"
      :
      : [sp] "r"(sp), [pc] "r"(pc));  // Replace [sp] by sp, and [pc] by pc.
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
