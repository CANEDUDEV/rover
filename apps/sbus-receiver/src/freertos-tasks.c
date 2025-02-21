#include "freertos-tasks.h"

#include <stdio.h>
#include <string.h>

#include "ck-data.h"
#include "float.h"
#include "peripherals.h"
#include "ports.h"
#include "sbus.h"
#include "steering.h"

// CK
#include "mayor.h"

// STM32Common
#include "error.h"
#include "letter-reader.h"
#include "lfs-wrapper.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define STEERING_TASK_STACK_SIZE (2 * configMINIMAL_STACK_SIZE)
static TaskHandle_t steering_task;
static StaticTask_t steering_task_buf;
static StackType_t steering_task_stack[STEERING_TASK_STACK_SIZE];

// DSHOT
#define DSHOT_FRAME_SIZE_BITS 16
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_CRC_MASK 0xF
#define DSHOT_BIT_0_PULSE 45
#define DSHOT_BIT_1_PULSE (2 * DSHOT_BIT_0_PULSE)

typedef enum {
  DSHOT_150,
  DSHOT_300,
  DSHOT_600,
} dshot_protocol_t;

typedef struct {
  uint16_t data;
} dshot_frame_t;

typedef struct {
  // Use double buffering
  uint32_t data[2][DSHOT_FRAME_SIZE_BITS];
  uint8_t active_buf;
  dshot_frame_t next_frame;
} dshot_dma_buf_t;

static dshot_dma_buf_t dshot_dma_buffers[4];

void dshot_init_bufs(void);
void dshot_set_protocol(dshot_protocol_t proto);
void dshot_start(void);
void dshot_update_buf(size_t bufno);

dshot_frame_t create_dshot_frame(uint16_t throttle);

// SBUS
void sbus_read_and_steer(void *unused);
void send_steering_command(steering_command_t *command);

// CK
int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter);

#define IMU_TASK_STACK_SIZE (2 * configMINIMAL_STACK_SIZE)
static TaskHandle_t imu_task;
static StaticTask_t imu_task_buf;
static StackType_t imu_task_stack[IMU_TASK_STACK_SIZE];

#define IMU_REG_GYRO_BASE 0x43
#define IMU_REG_ACCEL_BASE 0x3B
#define IMU_REG_WHOAMI 0x75

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} sensor_reading_t;

struct float_reading {
  float x;
  float y;
  float z;
};

typedef struct float_reading gyro_t;
typedef struct float_reading accel_t;

uint8_t imu_read_reg(uint8_t reg) {
  SPI_HandleTypeDef *hspi = &get_peripherals()->hspi3;
  const uint8_t read_reg = 1 << 7;
  uint8_t tx = read_reg | reg;
  uint8_t rx = 0;
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, &tx, sizeof(tx), portMAX_DELAY);
  HAL_SPI_Receive(hspi, &rx, sizeof(rx), portMAX_DELAY);
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_SET);
  return rx;
}

uint16_t imu_read_double_reg(uint8_t high_reg) {
  const uint8_t byte_shift = 8;
  uint16_t msb = imu_read_reg(high_reg);
  uint16_t lsb = imu_read_reg(high_reg + 1);
  return (msb << byte_shift | lsb);
}

sensor_reading_t imu_read_sensor_reg(uint8_t base_reg) {
  sensor_reading_t reading = {
      .x = (int16_t)imu_read_double_reg(base_reg),
      .y = (int16_t)imu_read_double_reg(base_reg + 2),
      .z = (int16_t)imu_read_double_reg(base_reg + 4),
  };
  return reading;
}

void imu_update_gyro(gyro_t *gyro) {
  sensor_reading_t reading = imu_read_sensor_reg(IMU_REG_GYRO_BASE);
  const float gyro_max = 250;  // Configurable, use default for now
  gyro->x = gyro_max * ((float)reading.x / INT16_MAX);
  gyro->y = gyro_max * ((float)reading.y / INT16_MAX);
  gyro->z = gyro_max * ((float)reading.z / INT16_MAX);
}

void imu_update_accel(accel_t *accel) {
  sensor_reading_t reading = imu_read_sensor_reg(IMU_REG_ACCEL_BASE);
  const float g = 9.80665F;
  const float accel_max = 2 * g;  // Configurable, use default for now
  accel->x = accel_max * ((float)reading.x / INT16_MAX);
  accel->y = accel_max * ((float)reading.y / INT16_MAX);
  accel->z = accel_max * ((float)reading.z / INT16_MAX);
}

void imu_task_fn(void *unused) {
  (void)unused;

  const size_t float_str_size = 32;
  char x_str[float_str_size];
  char y_str[float_str_size];
  char z_str[float_str_size];

  gyro_t gyro;
  accel_t accel;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("whoami: 0x%02x\r\n", imu_read_reg(IMU_REG_WHOAMI));

    imu_update_gyro(&gyro);

    float_sprint(x_str, gyro.x);
    float_sprint(y_str, gyro.y);
    float_sprint(z_str, gyro.z);
    printf("gyro: {x: %s, y: %s, z: %s (deg/s)}\r\n", x_str, y_str, z_str);

    imu_update_accel(&accel);

    float_sprint(x_str, accel.x);
    float_sprint(y_str, accel.y);
    float_sprint(z_str, accel.z);
    printf("accel: {x: %s, y: %s, z: %s (m/s2)}\r\n", x_str, y_str, z_str);
  }
}

void task_init(void) {
  uint8_t priority = LOWEST_TASK_PRIORITY;

  if (init_lfs_task(priority++) < 0) {
    error();
  }

  imu_task = xTaskCreateStatic(imu_task_fn, "IMU task", IMU_TASK_STACK_SIZE,
                               NULL, priority++, imu_task_stack, &imu_task_buf);

  steering_task = xTaskCreateStatic(sbus_read_and_steer, "steering task",
                                    STEERING_TASK_STACK_SIZE, NULL, priority++,
                                    steering_task_stack, &steering_task_buf);

  letter_reader_cfg_t letter_reader_cfg = {
      .priority = priority++,
      .app_letter_handler_func = handle_letter,
  };

  if (init_letter_reader_task(letter_reader_cfg) != APP_OK) {
    error();
  }
}

void sbus_read_and_steer(void *unused) {
  (void)unused;

  init_steering();

  dshot_set_protocol(DSHOT_600);
  dshot_init_bufs();
  dshot_start();

  for (;;) {
    sbus_message_t message;
    steering_command_t steering_command = neutral_steering_command();

    if (sbus_read_message(&message) != APP_OK) {
      send_steering_command(&steering_command);
      continue;
    }

    // Failsafe usually triggers if many frames are lost in a row.
    // Indicates heavy connection loss. Frame lost indicateds slight
    // connection loss or issue with frame.
    bool read_failed = message.failsafe_activated || message.frame_lost;

    if (read_failed) {
      printf("Frame lost, sending neutral command.\r\n");
    } else {
      steering_command = sbus_message_to_steering_command(&message);
    }

    send_steering_command(&steering_command);
  }
}

void send_steering_command(steering_command_t *command) {
  // DSHOT
  const uint16_t dshot_throttle = (2 * (command->throttle - 1000)) + 48;
  dshot_frame_t dshot_frame = create_dshot_frame(dshot_throttle);
  for (int i = 0; i < 4; i++) {
    dshot_dma_buffers[i].next_frame = dshot_frame;
  }

  // CAN
  if (!command->steering_is_on ||
      ck_get_action_mode() == CK_ACTION_MODE_FREEZE) {
    return;
  }

  ck_data_t *ck_data = get_ck_data();

  memcpy(&ck_data->steering_page->lines[1], &command->steering_angle,
         sizeof(command->steering_angle));
  memcpy(&ck_data->throttle_page->lines[1], &command->throttle,
         sizeof(command->throttle));

  ck_err_t ret = ck_send_document(ck_data->steering_folder->folder_no);
  if (ret != CK_OK && ret != CK_ERR_TIMEOUT) {
    printf("error: failed to send steering doc\r\n");
  }
  ret = ck_send_document(ck_data->throttle_folder->folder_no);
  if (ret != CK_OK && ret != CK_ERR_TIMEOUT) {
    printf("error: failed to send throttle doc\r\n");
  }
}

// Assumes timer clock is 72 MHz
void dshot_set_protocol(dshot_protocol_t proto) {
  TIM_HandleTypeDef *htim = &get_peripherals()->htim1;
  switch (proto) {
    case DSHOT_150:
      __HAL_TIM_SET_PRESCALER(htim, 4 - 1);
      break;

    case DSHOT_300:
      __HAL_TIM_SET_PRESCALER(htim, 2 - 1);
      break;

    case DSHOT_600:
      __HAL_TIM_SET_PRESCALER(htim, 0);
      break;
  }
}

void dshot_init_bufs(void) {
  for (int i = 0; i < 4; i++) {
    dshot_dma_buffers[i].next_frame = create_dshot_frame(0);

    dshot_dma_buffers[i].active_buf = 0;
    dshot_update_buf(i);
    dshot_dma_buffers[i].active_buf = 1;
  }
}

void dshot_update_buf(size_t bufno) {
  dshot_dma_buf_t *current = &dshot_dma_buffers[bufno];

  for (size_t i = 0; i < DSHOT_FRAME_SIZE_BITS; i++) {
    bool bit_high = (current->next_frame.data >> i) & 1;
    if (bit_high) {
      current->data[current->active_buf][DSHOT_FRAME_SIZE_BITS - 1 - i] =
          DSHOT_BIT_1_PULSE;
    } else {
      current->data[current->active_buf][DSHOT_FRAME_SIZE_BITS - 1 - i] =
          DSHOT_BIT_0_PULSE;
    }
  }
}

void dshot_start(void) {
  peripherals_t *peripherals = get_peripherals();

  HAL_TIM_PWM_Start_DMA(&peripherals->htim1, TIM_CHANNEL_1,
                        (uint32_t *)dshot_dma_buffers[0].data,
                        DSHOT_FRAME_SIZE_BITS * 2);
  HAL_TIM_PWM_Start_DMA(&peripherals->htim1, TIM_CHANNEL_2,
                        (uint32_t *)dshot_dma_buffers[1].data,
                        DSHOT_FRAME_SIZE_BITS * 2);
  HAL_TIM_PWM_Start_DMA(&peripherals->htim1, TIM_CHANNEL_3,
                        (uint32_t *)dshot_dma_buffers[2].data,
                        DSHOT_FRAME_SIZE_BITS * 2);
  HAL_TIM_PWM_Start_DMA(&peripherals->htim1, TIM_CHANNEL_4,
                        (uint32_t *)dshot_dma_buffers[3].data,
                        DSHOT_FRAME_SIZE_BITS * 2);
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
  uint8_t channel = 0;
  switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      channel = 0;
      break;

    case HAL_TIM_ACTIVE_CHANNEL_2:
      channel = 1;
      break;

    case HAL_TIM_ACTIVE_CHANNEL_3:
      channel = 2;
      break;

    case HAL_TIM_ACTIVE_CHANNEL_4:
      channel = 3;
      break;

    default:
      break;
  }

  dshot_dma_buf_t *dshot_buf = &dshot_dma_buffers[channel];
  dshot_buf->active_buf = (dshot_buf->active_buf + 1) % 2;
  dshot_update_buf(channel);
}

dshot_frame_t create_dshot_frame(uint16_t throttle) {
  uint16_t data = throttle;
  if (data > DSHOT_THROTTLE_MAX) {
    data = DSHOT_THROTTLE_MAX;
  }

  data <<= 1;  // 11 bit throttle + telemetry bit
  uint16_t crc_data = data;
  uint16_t crc = 0;
  for (int i = 0; i < 3; i++) {
    crc ^= crc_data;  // XOR data by nibbles
    crc_data >>= 4;
  }
  crc &= DSHOT_CRC_MASK;

  dshot_frame_t frame = {.data = (data << 4) | crc};

  return frame;
}

int handle_letter(const ck_folder_t *folder, const ck_letter_t *letter) {
  (void)folder;
  (void)letter;
  return APP_OK;
}
