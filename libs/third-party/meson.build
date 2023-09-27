drivers_inc = include_directories(
  'STM32F3xx_HAL_Driver' / 'Inc',
  'STM32F3xx_HAL_Driver' / 'Inc' / 'Legacy',
  'CMSIS' / 'Device' / 'ST' / 'STM32F3xx' / 'Include',
  'CMSIS' / 'Include',
)

drivers_src = files(
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_adc.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_adc_ex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_rcc.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_rcc_ex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_gpio.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_dma.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_cortex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_pwr.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_pwr_ex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_flash.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_flash_ex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_i2c.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_i2c_ex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_exti.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_can.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_spi.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_spi_ex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_tim.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_tim_ex.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_uart.c',
  'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_uart_ex.c',
)

freertos_inc = include_directories(
  'FreeRTOS' / 'Source' / 'include',
  'FreeRTOS' / 'Source' / 'portable' / 'GCC' / 'ARM_CM4F',
)

freertos_src = files(
  'FreeRTOS' / 'Source' / 'croutine.c',
  'FreeRTOS' / 'Source' / 'event_groups.c',
  'FreeRTOS' / 'Source' / 'list.c',
  'FreeRTOS' / 'Source' / 'queue.c',
  'FreeRTOS' / 'Source' / 'stream_buffer.c',
  'FreeRTOS' / 'Source' / 'tasks.c',
  'FreeRTOS' / 'Source' / 'timers.c',
  'FreeRTOS' / 'Source' / 'portable/MemMang/heap_4.c',
  'FreeRTOS' / 'Source' / 'portable/GCC/ARM_CM4F/port.c',
)

littlefs_inc = include_directories('littlefs')

littlefs_src = files(
  'littlefs' / 'lfs.c',
  'littlefs' / 'lfs_util.c',
)
