#include "peripherals.h"

#include "error.h"
#include "ports.h"

#define DMA1_Channel1_IRQ_PRIORITY 5
#define DMA2_Channel1_IRQ_PRIORITY 5
#define USART1_IRQ_PRIORITY 5
#define USB_HP_CAN_TX_IRQ_PRIORITY 5
#define PENDSV_IRQ_PRIORITY 15

static peripherals_t peripherals;

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

peripherals_t* get_peripherals(void) { return &peripherals; }

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

void adc1_init(void) {
  ADC_HandleTypeDef* hadc1 = &peripherals.hadc1;
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  /** Common config
   */
  hadc1->Instance = ADC1;
  hadc1->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1->Init.Resolution = ADC_RESOLUTION_12B;
  hadc1->Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1->Init.ContinuousConvMode = ENABLE;
  hadc1->Init.DiscontinuousConvMode = DISABLE;
  hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1->Init.NbrOfConversion = 3;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK) {
    error();
  }

  if (HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    error();
  }
}

void adc2_init(void) {
  ADC_HandleTypeDef* hadc2 = &peripherals.hadc2;
  ADC_ChannelConfTypeDef sConfig;

  /** Common config
   */
  hadc2->Instance = ADC2;
  hadc2->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2->Init.Resolution = ADC_RESOLUTION_12B;
  hadc2->Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2->Init.ContinuousConvMode = ENABLE;
  hadc2->Init.DiscontinuousConvMode = DISABLE;
  hadc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2->Init.NbrOfConversion = 2;
  hadc2->Init.DMAContinuousRequests = DISABLE;
  hadc2->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2->Init.LowPowerAutoWait = DISABLE;
  hadc2->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(hadc2) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK) {
    error();
  }

  if (HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    error();
  }
}

void can_init(void) {
  CAN_HandleTypeDef* hcan = &peripherals.hcan;
  hcan->Instance = CAN;
  hcan->Init.Prescaler = 18;  // NOLINT
  hcan->Init.Mode = CAN_MODE_NORMAL;
  hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = DISABLE;
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = DISABLE;
  hcan->Init.ReceiveFifoLocked = DISABLE;
  hcan->Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(hcan) != HAL_OK) {
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

void i2c3_init(void) {
  I2C_HandleTypeDef* hi2c3 = &peripherals.hi2c3;
  hi2c3->Instance = I2C3;
  hi2c3->Init.Timing = 0x10808DD3;  // NOLINT
  hi2c3->Init.OwnAddress1 = 0;
  hi2c3->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3->Init.OwnAddress2 = 0;
  hi2c3->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c3) != HAL_OK) {
    error();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    error();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(hi2c3, 0) != HAL_OK) {
    error();
  }
}

void spi1_init(void) {
  SPI_HandleTypeDef* hspi1 = &peripherals.hspi1;
  hspi1->Instance = SPI1;
  hspi1->Init.Mode = SPI_MODE_MASTER;
  hspi1->Init.Direction = SPI_DIRECTION_2LINES;
  hspi1->Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1->Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1->Init.CRCPolynomial = 7;  // NOLINT
  hspi1->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(hspi1) != HAL_OK) {
    error();
  }
}

void spi3_init(void) {
  SPI_HandleTypeDef* hspi3 = &peripherals.hspi3;
  hspi3->Instance = SPI3;
  hspi3->Init.Mode = SPI_MODE_MASTER;
  hspi3->Init.Direction = SPI_DIRECTION_2LINES;
  hspi3->Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3->Init.NSS = SPI_NSS_SOFT;
  hspi3->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3->Init.CRCPolynomial = 7;  // NOLINT
  hspi3->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(hspi3) != HAL_OK) {
    error();
  }
}

void tim1_init(void) {
  TIM_HandleTypeDef* htim1 = &peripherals.htim1;
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1->Instance = TIM1;
  htim1->Init.Prescaler = PWM_PSC_1MHZ;
  htim1->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1->Init.Period = PWM_PERIOD_50HZ;
  htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1->Init.RepetitionCounter = 0;
  htim1->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim1, &sMasterConfig) != HAL_OK) {
    error();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_MID_POS_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    error();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    error();
  }

  HAL_TIM_MspPostInit(htim1);
}

void uart1_init(void) {
  UART_HandleTypeDef* huart1 = &peripherals.huart1;
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

void dma_init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, DMA1_Channel1_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, DMA2_Channel1_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}

void gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
                    DEBUG_LED_Pin | H_BRIDGE_ENABLE_Pin | H_BRIDGE_MODE1_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, H_BRIDDGE_MODE2_Pin | H_BRIDGE_PHASE_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(H_BRIDGE_nSLEEP_GPIO_Port, H_BRIDGE_nSLEEP_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_LED_Pin H_BRIDGE_ENABLE_Pin H_BRIDGE_MODE1_Pin
   */
  GPIO_InitStruct.Pin =
      DEBUG_LED_Pin | H_BRIDGE_ENABLE_Pin | H_BRIDGE_MODE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : H_BRIDDGE_MODE2_Pin H_BRIDGE_PHASE_Pin */
  GPIO_InitStruct.Pin = H_BRIDDGE_MODE2_Pin | H_BRIDGE_PHASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : H_BRIDGE_nSLEEP_Pin */
  GPIO_InitStruct.Pin = H_BRIDGE_nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(H_BRIDGE_nSLEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : H_BRIDGE_nFAULT_Pin */
  GPIO_InitStruct.Pin = H_BRIDGE_nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(H_BRIDGE_nFAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_FD_INT_Pin CAN_FD_SOF_Pin */
  GPIO_InitStruct.Pin = CAN_FD_INT_Pin | CAN_FD_SOF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_FD_INT1_Pin CAN_FD_INT0_Pin */
  GPIO_InitStruct.Pin = CAN_FD_INT1_Pin | CAN_FD_INT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_NSS_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_NSS_GPIO_Port, &GPIO_InitStruct);
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
  GPIO_InitTypeDef GPIO_InitStruct;
  if (hadc->Instance == ADC1) {
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin =
        SENSOR_POWER_Pin | SERVO_CURRENT_Pin | BAT_VOLTAGE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    adc1_dma_init(hadc);

  } else if (hadc->Instance == ADC2) {
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    */
    GPIO_InitStruct.Pin = VCC_SERVO_VOLTAGE_Pin | H_BRIDGE_VPROP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
    HAL_RCC_ADC12_CLK_ENABLED--;
    if (HAL_RCC_ADC12_CLK_ENABLED == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(GPIOA,
                    SENSOR_POWER_Pin | SERVO_CURRENT_Pin | BAT_VOLTAGE_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

  } else if (hadc->Instance == ADC2) {
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if (HAL_RCC_ADC12_CLK_ENABLED == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    */
    HAL_GPIO_DeInit(GPIOA, VCC_SERVO_VOLTAGE_Pin | H_BRIDGE_VPROP_Pin);

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
  GPIO_InitTypeDef GPIO_InitStruct;
  if (hcan->Instance == CAN) {
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN_TX_IRQn, USB_HP_CAN_TX_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);
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
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    /* CAN interrupt DeInit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN_TX_IRQn);
  }
}

/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

  } else if (hi2c->Instance == I2C3) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
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
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  } else if (hi2c->Instance == I2C3) {
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
  }
}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if (hspi->Instance == SPI1) {
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
    GPIO_InitStruct.Pin = CAN_FD_SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(CAN_FD_SPI_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        CAN_FD_SPI_SCK_Pin | CAN_FD_SPI_MISO_Pin | CAN_FD_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  } else if (hspi->Instance == SPI3) {
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA15     ------> SPI1_NSS
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(CAN_FD_SPI_CS_GPIO_Port, CAN_FD_SPI_CS_Pin);

    HAL_GPIO_DeInit(
        GPIOB, CAN_FD_SPI_SCK_Pin | CAN_FD_SPI_MISO_Pin | CAN_FD_SPI_MOSI_Pin);

  } else if (hspi->Instance == SPI3) {
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
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
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if (htim->Instance == TIM1) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PC3     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = SERVO_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(SERVO_PWM_GPIO_Port, &GPIO_InitStruct);
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
  }
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if (huart->Instance == USART1) {
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, USART1_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
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
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
}
