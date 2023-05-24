stm32_common_inc = include_directories('include')
stm32_common_src = files(
  'src' / 'assert.c',
  'src' / 'common-interrupts.c',
  'src' / 'startup_stm32f302xe.s',
  'src' / 'system_stm32f3xx.c',
)

stm32_linker_script = configure_file(
  copy: true,
  input: files('src' / 'STM32F302RETx_FLASH.ld'),
  output: '@PLAINNAME@',
)

stm32_common_tidy_files = files(
  'src' / 'assert.c',
  'src' / 'common-interrupts.c',
)
