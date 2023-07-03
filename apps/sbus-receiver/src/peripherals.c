#include "peripherals.h"

#include "error.h"
#include "ports.h"
#include "print.h"
#include "stm32f3xx_hal.h"

#define USART2_IRQ_PRIORITY 5

static peripherals_t peripherals;

void gpio_init(void);
void uart2_init(void);

peripherals_t* get_peripherals(void) { return &peripherals; }

void peripherals_init(void) {
  common_peripherals_init();
  peripherals.common_peripherals = get_common_peripherals();

  gpio_init();
  uart2_init();
}

void gpio_init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void uart2_init(void) {
  UART_HandleTypeDef* huart2 = &peripherals.huart2;
  huart2->Instance = USART2;
  huart2->Init.BaudRate = 100000;  // NOLINT
  huart2->Init.WordLength = UART_WORDLENGTH_8B;
  huart2->Init.StopBits = UART_STOPBITS_2;
  huart2->Init.Parity = UART_PARITY_EVEN;
  huart2->Init.Mode = UART_MODE_RX;
  huart2->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2->Init.OverSampling = UART_OVERSAMPLING_16;
  huart2->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart2->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(huart2) != HAL_OK) {
    error();
  }
}

/**
 * @brief CAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN) {
    can_msp_init();
  }
}

/**
 * @brief CAN MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN) {
    can_msp_deinit();
  }
}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_init();
  }
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_deinit();
  }
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    uart1_msp_init();

  } else if (huart->Instance == USART2) {
    GPIO_InitTypeDef gpio_init_struct;
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    gpio_init_struct.Pin = UART2_RX_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(UART2_GPIO_PORT, &gpio_init_struct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, USART2_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    uart1_msp_deinit();
  } else if (huart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(UART2_GPIO_PORT, UART2_RX_PIN);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}
