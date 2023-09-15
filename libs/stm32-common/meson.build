stm32_common_inc = include_directories('include')

stm32_lib_src =  files(
  'src' / 'assert.c',
  'src' / 'clock.c',
  'src' / 'common-interrupts.c',
  'src' / 'common-peripherals.c',
  'src' / 'error.c',
  'src' / 'flash.c',
  'src' / 'freertos.c',
  'src' / 'print.c',
)

stm32_startup_src = files(
  'src' / 'startup_stm32f302xe.s',
  'src' / 'system_stm32f3xx.c',
)

stm32_common_src = [stm32_lib_src, stm32_startup_src]

stm32_common_tidy_files = [stm32_lib_src]
