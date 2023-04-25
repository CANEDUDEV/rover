battery_node_inc = include_directories('Core' / 'Inc')
battery_node_src = files(
  'Core' / 'Src' / 'freertos.c',
  'Core' / 'Src' / 'main.c',
  'Core' / 'Src' / 'stm32f3xx_hal_msp.c',
  'Core' / 'Src' / 'stm32f3xx_it.c',
  'Core' / 'Src' / 'system_stm32f3xx.c',
  'startup_stm32f302xe.s',
)

ld_script = configure_file(
  copy: true,
  input: files('STM32F302RETx_FLASH.ld'),
  output: '@PLAINNAME@',
)

battery_node_elf = executable(
  'battery_node.elf',
  [drivers_src, freertos_src, battery_node_src],
  include_directories: [drivers_inc, freertos_inc, battery_node_inc],
  link_depends: ld_script,
  link_args: ['-T@0@'.format(ld_script)],
)

battery_node_bin = custom_target(
  'battery_node.bin',
  output: 'battery_node.bin',
  input: battery_node_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: battery_node_elf,
  build_by_default: true,
)

# List of files that should be analyzed by clang-tidy.
# We typically ignore all auto-generated files that we don't modify.
battery_node_tidy_files = files(
  'Core' / 'Src' / 'main.c',
)
