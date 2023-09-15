#include "clock.h"
#include "common-peripherals.h"
#include "print.h"

// Bootloader is 64KB.
#define APPROM_START (FLASH_BASE + 64 * 1024)

// program counter (pc) is in r0, stack pointer (sp) in r1. Loads sp into Main
// Stack Pointer (MSP) register, and then branches to address given by pc.
// NOLINTNEXTLINE(readability*,bugprone*)
__attribute__((naked)) static void start_app(uint32_t pc, uint32_t sp) {
  __asm(
      "           \n\
          msr msp, r1 /* load r1 into MSP */\n\
          bx r0       /* branch to the address at r0 */\n\
    ");
}

int main(void) {
  HAL_Init();
  system_clock_init();
  common_peripherals_init();

  print("Entering bootloader...\r\n");

  uint32_t* app_code = (uint32_t*)APPROM_START;
  uint32_t app_sp = app_code[0];
  uint32_t app_start = app_code[1];

  print("Exiting bootloader...\r\n");

  // Resets the peripheral buses, so no need for additional deinit.
  HAL_DeInit();

  // Relocate vector table
  SCB->VTOR = APPROM_START;
  start_app(app_start, app_sp);

  while (1) {
    // Never reached
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN) {
    can_msp_init();
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN) {
    can_msp_deinit();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_init();
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_deinit();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    uart1_msp_init();
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    uart1_msp_deinit();
  }
}
