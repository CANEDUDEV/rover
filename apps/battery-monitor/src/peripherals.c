#include "peripherals.h"

#include "error.h"
#include "ports.h"
#include "stm32f3xx_hal.h"

#define DMA1_Channel1_IRQ_PRIORITY 5
#define DMA2_Channel1_IRQ_PRIORITY 5
#define EXTI15_10_IRQ_PRIORITY 5

static peripherals_t peripherals;

static uint32_t hal_rcc_adc12_clk_enabled = 0;

void adc1_init(void);
void adc2_init(void);
void i2c1_init(void);
void dma_init(void);
void gpio_init(void);

peripherals_t* get_peripherals(void) {
  return &peripherals;
}

void peripherals_init(void) {
  common_peripherals_init();
  peripherals.common_peripherals = get_common_peripherals();

  gpio_init();
  dma_init();  // Must be called before ADC init functions.
  adc1_init();
  adc2_init();
  i2c1_init();
}

void adc1_init(void) {
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef config;
  ADC_HandleTypeDef* hadc1 = &peripherals.hadc1;

  /** Common config
   */
  hadc1->Instance = ADC1;
  hadc1->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1->Init.Resolution = ADC_RESOLUTION_12B;
  hadc1->Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1->Init.ContinuousConvMode = DISABLE;
  hadc1->Init.DiscontinuousConvMode = DISABLE;
  hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1->Init.NbrOfConversion = 4;
  hadc1->Init.DMAContinuousRequests = DISABLE;
  hadc1->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1->Init.LowPowerAutoWait = DISABLE;
  hadc1->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(hadc1) != HAL_OK) {
    error();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_1;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc1, &config) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_2;
  config.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(hadc1, &config) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_3;
  config.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(hadc1, &config) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_4;
  config.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(hadc1, &config) != HAL_OK) {
    error();
  }

  if (HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    error();
  }
}

void adc2_init(void) {
  ADC_ChannelConfTypeDef config;
  ADC_HandleTypeDef* hadc2 = &peripherals.hadc2;

  // Common config for all channels
  hadc2->Instance = ADC2;
  hadc2->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2->Init.Resolution = ADC_RESOLUTION_12B;
  hadc2->Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2->Init.ContinuousConvMode = DISABLE;
  hadc2->Init.DiscontinuousConvMode = DISABLE;
  hadc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2->Init.NbrOfConversion = 5;  // NOLINT
  hadc2->Init.DMAContinuousRequests = DISABLE;
  hadc2->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2->Init.LowPowerAutoWait = DISABLE;
  hadc2->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(hadc2) != HAL_OK) {
    error();
  }

  // Configure the ADC channels
  config.Channel = ADC_CHANNEL_1;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc2, &config) != HAL_OK) {
    error();
  }

  config.Channel = ADC_CHANNEL_2;
  config.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(hadc2, &config) != HAL_OK) {
    error();
  }

  config.Channel = ADC_CHANNEL_3;
  config.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(hadc2, &config) != HAL_OK) {
    error();
  }

  config.Channel = ADC_CHANNEL_4;
  config.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(hadc2, &config) != HAL_OK) {
    error();
  }

  config.Channel = ADC_CHANNEL_11;
  config.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(hadc2, &config) != HAL_OK) {
    error();
  }

  if (HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    error();
  }
}

void i2c1_init(void) {
  I2C_HandleTypeDef* hi2c1 = &peripherals.hi2c1;
  hi2c1->Instance = I2C1;
  hi2c1->Init.Timing = 0x10808DD3;  // NOLINT
  hi2c1->Init.OwnAddress1 = 0;
  hi2c1->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1->Init.OwnAddress2 = 0;
  hi2c1->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c1) != HAL_OK) {
    error();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    error();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(hi2c1, 0) != HAL_OK) {
    error();
  }
}

/**
 * Enable DMA controller clock
 */
void dma_init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, DMA1_Channel1_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, DMA2_Channel1_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}

void gpio_init(void) {
  GPIO_InitTypeDef gpio_init;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_PORT,
                    LED4_PIN | LED3_PIN | LED2_PIN | LED1_PIN | nPOWER_OFF_PIN,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(REG_POWER_ON_GPIO_PORT, REG_POWER_ON_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : OVER_CURRENT_PIN */
  gpio_init.Pin = OVER_CURRENT_PIN;
  gpio_init.Mode = GPIO_MODE_IT_FALLING;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OVER_CURRENT_GPIO_PORT, &gpio_init);

  /*Configure GPIO pins : LED4_PIN LED3_PIN LED2_PIN LED1_PIN
                           POWER_OFF_PIN */
  gpio_init.Pin = LED4_PIN | LED3_PIN | LED2_PIN | LED1_PIN | nPOWER_OFF_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_PORT, &gpio_init);

  /*Configure GPIO pin : REG_POWER_ON_PIN */
  gpio_init.Pin = REG_POWER_ON_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REG_POWER_ON_GPIO_PORT, &gpio_init);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, EXTI15_10_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ADC1 DMA Init */
void adc1_dma_init(ADC_HandleTypeDef* hadc) {
  DMA_HandleTypeDef* hdma_adc1 = &peripherals.hdma_adc1;
  hdma_adc1->Instance = DMA1_Channel1;
  hdma_adc1->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1->Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1->Init.Mode = DMA_NORMAL;
  hdma_adc1->Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(hdma_adc1) != HAL_OK) {
    error();
  }
  __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc1);
}

/* ADC2 DMA Init */
void adc2_dma_init(ADC_HandleTypeDef* hadc) {
  DMA_HandleTypeDef* hdma_adc2 = &peripherals.hdma_adc2;
  hdma_adc2->Instance = DMA2_Channel1;
  hdma_adc2->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc2->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc2->Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc2->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc2->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc2->Init.Mode = DMA_NORMAL;
  hdma_adc2->Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(hdma_adc2) != HAL_OK) {
    error();
  }

  __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc2);
}

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef gpio_init;
  if (hadc->Instance == ADC1) {
    /* Peripheral clock enable */
    hal_rcc_adc12_clk_enabled++;
    if (hal_rcc_adc12_clk_enabled) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    gpio_init.Pin = CELL1_MEASURE_PIN | CELL2_MEASURE_PIN | CELL3_MEASURE_PIN |
                    CELL4_MEASURE_PIN;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CELL1_MEASURE_GPIO_PORT, &gpio_init);
    adc1_dma_init(hadc);

  } else if (hadc->Instance == ADC2) {
    /* Peripheral clock enable */
    hal_rcc_adc12_clk_enabled++;
    if (hal_rcc_adc12_clk_enabled == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    PA6     ------> ADC2_IN3
    PA7     ------> ADC2_IN4
    PC5     ------> ADC2_IN11
    */
    gpio_init.Pin = CELL5_MEASURE_PIN | CELL6_MEASURE_PIN |
                    I_PWR_A_MEASURE_PIN | REG_VOUT_MEASURE_PIN;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CELL5_MEASURE_GPIO_PORT, &gpio_init);

    gpio_init.Pin = VBAT_I_MEASURE_PIN;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBAT_I_MEASURE_GPIO_PORT, &gpio_init);
    adc2_dma_init(hadc);
  }
}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    /* Peripheral clock disable */
    hal_rcc_adc12_clk_enabled--;
    if (hal_rcc_adc12_clk_enabled == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(CELL1_MEASURE_GPIO_PORT,
                    CELL1_MEASURE_PIN | CELL2_MEASURE_PIN | CELL3_MEASURE_PIN |
                        CELL4_MEASURE_PIN);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

  } else if (hadc->Instance == ADC2) {
    /* Peripheral clock disable */
    hal_rcc_adc12_clk_enabled--;
    if (hal_rcc_adc12_clk_enabled == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    PA6     ------> ADC2_IN3
    PC5     ------> ADC2_IN11
    */
    HAL_GPIO_DeInit(CELL5_MEASURE_GPIO_PORT,
                    CELL5_MEASURE_PIN | CELL6_MEASURE_PIN |
                        I_PWR_A_MEASURE_PIN | REG_VOUT_MEASURE_PIN);

    HAL_GPIO_DeInit(VBAT_I_MEASURE_GPIO_PORT, VBAT_I_MEASURE_PIN);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
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
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
  GPIO_InitTypeDef gpio_init;
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    gpio_init.Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C1_SCL_GPIO_PORT, &gpio_init);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1) {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(I2C1_SCL_GPIO_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN);
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
  }
}
