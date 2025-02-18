#include "peripherals.h"

#include "error.h"
#include "ports.h"
#include "stm32f3xx_hal_gpio.h"

#define TIM1_DMA_IRQ_PRIORITY 0
#define USART2_IRQ_PRIORITY 5

static peripherals_t peripherals;

void gpio_init(void);
void dma_init(void);
void tim1_init(void);
void imu_init(void);
void uart2_init(void);

peripherals_t* get_peripherals(void) {
  return &peripherals;
}

void peripherals_init(void) {
  common_peripherals_init();
  peripherals.common_peripherals = get_common_peripherals();

  gpio_init();
  dma_init();
  tim1_init();
  imu_init();
  uart2_init();
}

void gpio_init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 3.3V output
  HAL_GPIO_WritePin(VDD_IO_LEVEL_GPIO_PORT, VDD_IO_LEVEL_PIN, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpio_init;
  gpio_init.Pin = VDD_IO_LEVEL_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VDD_IO_LEVEL_GPIO_PORT, &gpio_init);

  // Enable all GPIO outputs
  HAL_GPIO_WritePin(GPIO_PWRON_1_2_GPIO_PORT, GPIO_PWRON_1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO_PWRON_1_2_GPIO_PORT, GPIO_PWRON_2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO_PWRON_3_4_GPIO_PORT, GPIO_PWRON_3_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO_PWRON_3_4_GPIO_PORT, GPIO_PWRON_4_PIN, GPIO_PIN_SET);

  gpio_init.Pin = GPIO_PWRON_1_PIN | GPIO_PWRON_2_PIN;
  HAL_GPIO_Init(GPIO_PWRON_1_2_GPIO_PORT, &gpio_init);

  gpio_init.Pin = GPIO_PWRON_3_PIN | GPIO_PWRON_4_PIN;
  HAL_GPIO_Init(GPIO_PWRON_3_4_GPIO_PORT, &gpio_init);
}

void tim1_init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  TIM_HandleTypeDef* htim1 = &peripherals.htim1;

  htim1->Instance = TIM1;
  htim1->Init.Prescaler = 0;
  htim1->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1->Init.Period = 120;  // NOLINT(*-magic-numbers)
  htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim1) != HAL_OK) {
    error();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim1, &sClockSourceConfig) != HAL_OK) {
    error();
  }
  if (HAL_TIM_PWM_Init(htim1) != HAL_OK) {
    error();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim1, &sMasterConfig) != HAL_OK) {
    error();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;  // NOLINT(*-magic-numbers)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    error();
  }
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    error();
  }
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    error();
  }
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    error();
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init;

  gpio_init.Pin = TIM1_CH1_PIN | TIM1_CH2_PIN | TIM1_CH3_PIN | TIM1_CH4_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF2_TIM1;

  HAL_GPIO_Init(TIM1_GPIO_PORT, &gpio_init);
}

void dma_init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();

  // TIM1_CH1
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, TIM1_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  // TIM1_CH2
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, TIM1_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  // TIM1_CH3
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, TIM1_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

  // TIM1_CH4
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, TIM1_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

// IMU over SPI3
void imu_init(void) {
  SPI_HandleTypeDef* hspi3 = &peripherals.hspi3;
  hspi3->Instance = SPI3;
  hspi3->Init.Mode = SPI_MODE_MASTER;
  hspi3->Init.Direction = SPI_DIRECTION_2LINES;
  hspi3->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3->Init.NSS = SPI_NSS_SOFT;
  hspi3->Init.BaudRatePrescaler =
      SPI_BAUDRATEPRESCALER_64;  // IMU max freq is 1MHz, this gives < 1 MHZ
  hspi3->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3->Init.CRCPolynomial = 7;  // NOLINT
  hspi3->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(hspi3) != HAL_OK) {
    error();
  }
}

void uart2_init(void) {
  UART_HandleTypeDef* huart2 = &peripherals.huart2;
  huart2->Instance = USART2;
  huart2->Init.BaudRate = 100000;                // NOLINT
  huart2->Init.WordLength = UART_WORDLENGTH_9B;  // 8 data bits, 1 parity bit
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

  } else if (hspi->Instance == SPI2) {
    spi2_msp_init();
  } else if (hspi->Instance == SPI3) {
    GPIO_InitTypeDef gpio_init;
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    PD2     ------> SPI3_NSS
    */
    gpio_init.Pin = SPI3_SCK_PIN | SPI3_MISO_PIN | SPI3_MOSI_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(SPI3_SCK_GPIO_PORT, &gpio_init);

    // Init NSS
    HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_SET);

    gpio_init.Pin = SPI3_NSS_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI3_NSS_GPIO_PORT, &gpio_init);
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

  } else if (hspi->Instance == SPI2) {
    spi2_msp_deinit();
  } else if (hspi->Instance == SPI3) {
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    PD2     ------> SPI3_NSS
    */

    HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_DeInit(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN);
    HAL_GPIO_DeInit(SPI3_SCK_GPIO_PORT,
                    SPI3_SCK_PIN | SPI3_MISO_PIN | SPI3_MOSI_PIN);
  }
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM1) {
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    DMA_HandleTypeDef* hdma_tim1_ch1 = &peripherals.hdma_tim1_ch1;

    hdma_tim1_ch1->Instance = DMA1_Channel2;
    hdma_tim1_ch1->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch1->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch1->Init.Mode = DMA_CIRCULAR;
    hdma_tim1_ch1->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(hdma_tim1_ch1) != HAL_OK) {
      error();
    }

    __HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC1], *hdma_tim1_ch1);

    DMA_HandleTypeDef* hdma_tim1_ch2 = &peripherals.hdma_tim1_ch2;

    hdma_tim1_ch2->Instance = DMA1_Channel3;
    hdma_tim1_ch2->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch2->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch2->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch2->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch2->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch2->Init.Mode = DMA_CIRCULAR;

    hdma_tim1_ch2->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(hdma_tim1_ch2) != HAL_OK) {
      error();
    }

    __HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC2], *hdma_tim1_ch2);

    DMA_HandleTypeDef* hdma_tim1_ch3 = &peripherals.hdma_tim1_ch3;

    hdma_tim1_ch3->Instance = DMA1_Channel6;
    hdma_tim1_ch3->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch3->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch3->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch3->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch3->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch3->Init.Mode = DMA_CIRCULAR;
    hdma_tim1_ch3->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(hdma_tim1_ch3) != HAL_OK) {
      error();
    }

    __HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC3], *hdma_tim1_ch3);

    DMA_HandleTypeDef* hdma_tim1_ch4 = &peripherals.hdma_tim1_ch4;

    hdma_tim1_ch4->Instance = DMA1_Channel4;
    hdma_tim1_ch4->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch4->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch4->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch4->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch4->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch4->Init.Mode = DMA_CIRCULAR;
    hdma_tim1_ch4->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(hdma_tim1_ch4) != HAL_OK) {
      error();
    }

    __HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC4], *hdma_tim1_ch4);
  }
}

/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM1) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* tim1 DMA DeInit */
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC3]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC4]);

    HAL_GPIO_DeInit(TIM1_GPIO_PORT,
                    TIM1_CH1_PIN | TIM1_CH2_PIN | TIM1_CH3_PIN | TIM1_CH4_PIN);
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
    GPIO_InitTypeDef gpio_init;
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    gpio_init.Pin = UART2_RX_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(UART2_GPIO_PORT, &gpio_init);

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
