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

stm32_bootloader_src = files('src' / 'bootloader.c')

stm32_bootloader_linker_script = configure_file(
  copy: true,
  input: files('src' / 'bootloader.ld'),
  output: '@PLAINNAME@',
)

stm32_app_linker_script = configure_file(
  copy: true,
  input: files('src' / 'app.ld'),
  output: '@PLAINNAME@',
)

stm32_bootloader_elf = executable(
  'stm32-bootloader.elf',
  [stm32_bootloader_src, stm32_common_src, drivers_src, freertos_src, printf_src],
  include_directories: [stm32_common_inc, drivers_inc, freertos_inc, printf_inc],
  link_depends: stm32_bootloader_linker_script,
  link_args: ['-T@0@'.format(stm32_bootloader_linker_script)],
)

stm32_bootloader_bin = custom_target(
  'stm32-bootloader.bin',
  output: 'stm32-bootloader.bin',
  input: stm32_bootloader_elf,
  command: [objcopy, '-O', 'binary', '-S', '--pad-to', '0x08010000', '@INPUT@', '@OUTPUT@'],
  depends: stm32_bootloader_elf,
  build_by_default: true,
)

stm32_common_tidy_files = [stm32_bootloader_src, stm32_lib_src]
