#include "error.h"
#include "stm32f3xx_hal.h"

void system_clock_init(void) {
  RCC_OscInitTypeDef rcc_osc_init;
  RCC_ClkInitTypeDef rcc_clk_init;
  RCC_PeriphCLKInitTypeDef rcc_periph_clk_init;

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rcc_osc_init.HSEState = RCC_HSE_ON;
  rcc_osc_init.HSIState = RCC_HSI_ON;
  rcc_osc_init.PLL.PLLState = RCC_PLL_ON;
  rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  rcc_osc_init.PLL.PLLMUL = RCC_PLL_MUL6;
  rcc_osc_init.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&rcc_osc_init) != HAL_OK) {
    error();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  rcc_clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
  rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_2) != HAL_OK) {
    error();
  }
  rcc_periph_clk_init.PeriphClockSelection =
      RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C3 |
      RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM16 | RCC_PERIPHCLK_USART1 |
      RCC_PERIPHCLK_USART2;
  rcc_periph_clk_init.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  rcc_periph_clk_init.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  rcc_periph_clk_init.I2c3ClockSelection = RCC_I2C3CLKSOURCE_SYSCLK;
  rcc_periph_clk_init.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  rcc_periph_clk_init.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  rcc_periph_clk_init.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  rcc_periph_clk_init.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&rcc_periph_clk_init) != HAL_OK) {
    error();
  }
}
