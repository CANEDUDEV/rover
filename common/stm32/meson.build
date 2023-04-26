stm32_linker_script = configure_file(
  copy: true,
  input: files('src' / 'STM32F302RETx_FLASH.ld'),
  output: '@PLAINNAME@',
)

stm32_common_src = files(
  'src' / 'startup_stm32f302xe.s',
  'src' / 'system_stm32f3xx.c',
)
