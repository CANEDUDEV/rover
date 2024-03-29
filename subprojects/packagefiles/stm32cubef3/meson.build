project(
  'stm32cubef3',
  'c',
  version: '1.11.4',
)

# Define variables for the source and include files used by the Rover project.
# No compilation happens here.

stm32f3_hal_inc = include_directories(
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Inc',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Inc' / 'Legacy',
  'Drivers' / 'CMSIS' / 'Device' / 'ST' / 'STM32F3xx' / 'Include',
  'Drivers' / 'CMSIS' / 'Include',
)

stm32f3_hal_src = files(
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_adc.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_adc_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_rcc.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_rcc_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_gpio.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_dma.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_cortex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_pwr.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_pwr_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_flash.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_flash_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_i2c.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_i2c_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_exti.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_can.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_crc.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_crc_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_spi.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_spi_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_tim.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_tim_ex.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_uart.c',
  'Drivers' / 'STM32F3xx_HAL_Driver' / 'Src' / 'stm32f3xx_hal_uart_ex.c',
)

stm32f3_hal_dep = declare_dependency(
  include_directories: stm32f3_hal_inc,
  sources: stm32f3_hal_src,
)

freertos_inc = include_directories(
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'include',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'portable' / 'GCC' / 'ARM_CM4F',
)

freertos_src = files(
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'croutine.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'event_groups.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'list.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'queue.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'stream_buffer.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'tasks.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'timers.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'portable/MemMang/heap_4.c',
  'Middlewares' / 'Third_Party' / 'FreeRTOS' / 'Source' / 'portable/GCC/ARM_CM4F/port.c',
)

freertos_dep = declare_dependency(
  include_directories: freertos_inc,
  sources: freertos_src,
)
