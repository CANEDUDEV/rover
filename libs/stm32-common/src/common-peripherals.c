#include "common-peripherals.h"

#include "common-ports.h"
#include "error.h"

#define USART1_IRQ_PRIORITY 5
#define USB_LP_CAN_RX0_IRQ_PRIORITY 5
#define PENDSV_IRQ_PRIORITY 15

static common_peripherals_t common_peripherals;

void can_init(void);
void canfd_init(void);
void spi_flash_init(void);
void uart1_init(void);

common_peripherals_t* get_common_peripherals(void) {
  return &common_peripherals;
}

void common_peripherals_init(void) {
  can_init();
  canfd_init();
  spi_flash_init();
  uart1_init();
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
void can_init(void) {
  CAN_HandleTypeDef* hcan = &common_peripherals.hcan;
  hcan->Instance = CAN;
  hcan->Init.Mode = CAN_MODE_NORMAL;
  hcan->Init.Prescaler = 18;  // NOLINT
  hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = ENABLE;
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = ENABLE;
  hcan->Init.ReceiveFifoLocked = DISABLE;
  hcan->Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(hcan) != HAL_OK) {
    error();
  }

  CAN_FilterTypeDef allow_all = {
      .FilterIdHigh = 0,
      .FilterIdLow = 0,
      .FilterMaskIdHigh = 0,
      .FilterMaskIdLow = 0,
      .FilterFIFOAssignment = CAN_RX_FIFO0,
      .FilterBank = 0,
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterScale = CAN_FILTERSCALE_32BIT,
      .FilterActivation = CAN_FILTER_ENABLE,
  };

  if (HAL_CAN_ConfigFilter(hcan, &allow_all) != HAL_OK) {
    error();
  }
}

void canfd_init(void) {
  SPI_HandleTypeDef* hcanfd = &common_peripherals.hcanfd;
  /* SPI1 parameter configuration*/
  hcanfd->Instance = SPI1;
  hcanfd->Init.Mode = SPI_MODE_MASTER;
  hcanfd->Init.Direction = SPI_DIRECTION_2LINES;
  hcanfd->Init.DataSize = SPI_DATASIZE_4BIT;
  hcanfd->Init.CLKPolarity = SPI_POLARITY_LOW;
  hcanfd->Init.CLKPhase = SPI_PHASE_1EDGE;
  hcanfd->Init.NSS = SPI_NSS_HARD_OUTPUT;
  hcanfd->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hcanfd->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hcanfd->Init.TIMode = SPI_TIMODE_DISABLE;
  hcanfd->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hcanfd->Init.CRCPolynomial = 7;  // NOLINT
  hcanfd->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hcanfd->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(hcanfd) != HAL_OK) {
    error();
  }
}

void spi_flash_init(void) {
  SPI_HandleTypeDef* hspi_flash = &common_peripherals.hspi_flash;
  /* SPI2 parameter configuration*/
  hspi_flash->Instance = SPI2;
  hspi_flash->Init.Mode = SPI_MODE_MASTER;
  hspi_flash->Init.Direction = SPI_DIRECTION_2LINES;
  hspi_flash->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_flash->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_flash->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi_flash->Init.NSS = SPI_NSS_SOFT;
  hspi_flash->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi_flash->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_flash->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_flash->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(hspi_flash) != HAL_OK) {
    error();
  }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
void uart1_init(void) {
  UART_HandleTypeDef* huart1 = &common_peripherals.huart1;
  huart1->Instance = USART1;
  huart1->Init.BaudRate = 115200;  // NOLINT
  huart1->Init.WordLength = UART_WORDLENGTH_8B;
  huart1->Init.StopBits = UART_STOPBITS_1;
  huart1->Init.Parity = UART_PARITY_NONE;
  huart1->Init.Mode = UART_MODE_TX_RX;
  huart1->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1->Init.OverSampling = UART_OVERSAMPLING_16;
  huart1->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart1) != HAL_OK) {
    error();
  }
}

void can_msp_init(void) {
  GPIO_InitTypeDef gpio_init;
  /* Peripheral clock enable */
  __HAL_RCC_CAN1_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**CAN GPIO Configuration
  PB8     ------> CAN_RX
  PB9     ------> CAN_TX
  */
  gpio_init.Pin = CAN_RX_PIN | CAN_TX_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF9_CAN;
  HAL_GPIO_Init(CAN_GPIO_PORT, &gpio_init);

  /* CAN interrupt Init */
  HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, USB_LP_CAN_RX0_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
}

void can_msp_deinit(void) {
  /* Peripheral clock disable */
  __HAL_RCC_CAN1_CLK_DISABLE();

  /**CAN GPIO Configuration
  PB8     ------> CAN_RX
  PB9     ------> CAN_TX
  */
  HAL_GPIO_DeInit(CAN_GPIO_PORT, CAN_RX_PIN | CAN_TX_PIN);

  /* CAN interrupt DeInit */
  HAL_NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn);
}

void spi1_msp_init(void) {
  GPIO_InitTypeDef gpio_init;
  /* Peripheral clock enable */
  __HAL_RCC_SPI1_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**SPI1 GPIO Configuration
  PA15     ------> SPI1_NSS
  PB3     ------> SPI1_SCK
  PB4     ------> SPI1_MISO
  PB5     ------> SPI1_MOSI
  */
  gpio_init.Pin = CAN_FD_SPI_CS_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(CAN_FD_SPI_CS_GPIO_PORT, &gpio_init);

  gpio_init.Pin =
      CAN_FD_SPI_SCK_PIN | CAN_FD_SPI_MISO_PIN | CAN_FD_SPI_MOSI_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(CAN_FD_SPI_MISO_GPIO_PORT, &gpio_init);
}

void spi1_msp_deinit(void) {
  /* Peripheral clock disable */
  __HAL_RCC_SPI1_CLK_DISABLE();

  /**SPI1 GPIO Configuration
  PA15     ------> SPI1_NSS
  PB3     ------> SPI1_SCK
  PB4     ------> SPI1_MISO
  PB5     ------> SPI1_MOSI
  */
  HAL_GPIO_DeInit(CAN_FD_SPI_CS_GPIO_PORT, CAN_FD_SPI_CS_PIN);

  HAL_GPIO_DeInit(
      GPIOB, CAN_FD_SPI_SCK_PIN | CAN_FD_SPI_MISO_PIN | CAN_FD_SPI_MOSI_PIN);
}

void spi2_msp_init(void) {
  GPIO_InitTypeDef gpio_init;
  /* Peripheral clock enable */
  __HAL_RCC_SPI2_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**SPI2 GPIO Configuration
  PB12     ------> SPI2_NSS
  PB13     ------> SPI2_SCK
  PB14     ------> SPI2_MISO
  PB15     ------> SPI2_MOSI
  */
  gpio_init.Pin =
      SPI_FLASH_SCK_PIN | SPI_FLASH_SO_IO1_PIN | SPI_FLASH_SI_IO0_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI_FLASH_GPIO_PORT, &gpio_init);

  // Init NSS
  HAL_GPIO_WritePin(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN, GPIO_PIN_SET);

  gpio_init.Pin = SPI_FLASH_NSS_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI_FLASH_GPIO_PORT, &gpio_init);
}

void spi2_msp_deinit(void) {
  /* Peripheral clock disable */
  __HAL_RCC_SPI2_CLK_DISABLE();

  /**SPI2 GPIO Configuration
  PB12     ------> SPI2_NSS
  PB13     ------> SPI2_SCK
  PB14     ------> SPI2_MISO
  PB15     ------> SPI2_MOSI
  */
  HAL_GPIO_DeInit(SPI_FLASH_GPIO_PORT, SPI_FLASH_NSS_PIN | SPI_FLASH_SCK_PIN |
                                           SPI_FLASH_SO_IO1_PIN |
                                           SPI_FLASH_SI_IO0_PIN);
}

void uart1_msp_init(void) {
  GPIO_InitTypeDef gpio_init;
  /* Peripheral clock enable */
  __HAL_RCC_USART1_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART1 GPIO Configuration
  PA9     ------> USART1_TX
  PA10     ------> USART1_RX
  */
  gpio_init.Pin = UART1_RX_PIN | UART1_TX_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(UART1_GPIO_PORT, &gpio_init);

  /* USART1 interrupt Init */
  HAL_NVIC_SetPriority(USART1_IRQn, USART1_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void uart1_msp_deinit(void) {
  /* Peripheral clock disable */
  __HAL_RCC_USART1_CLK_DISABLE();

  /**USART1 GPIO Configuration
  PA9     ------> USART1_TX
  PA10     ------> USART1_RX
  */
  HAL_GPIO_DeInit(UART1_GPIO_PORT, UART1_RX_PIN | UART1_TX_PIN);

  /* USART1 interrupt DeInit */
  HAL_NVIC_DisableIRQ(USART1_IRQn);
}

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, PENDSV_IRQ_PRIORITY, 0);
}
