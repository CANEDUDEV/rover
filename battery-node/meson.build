battery_node_inc = include_directories('Core' / 'Inc')
battery_node_src = files(
  'Core' / 'Src' / 'app.c',
  'Core' / 'Src' / 'freertos.c',
  'Core' / 'Src' / 'main.c',
  'Core' / 'Src' / 'stm32f3xx_hal_msp.c',
  'Core' / 'Src' / 'stm32f3xx_it.c',
)

# Override the werror option since some of the warnings come from the STM32 HAL driver.
battery_node_elf = executable(
  'battery_node.elf',
  [common_src, battery_node_src],
  include_directories: [common_inc, battery_node_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
  override_options: ['werror=false'],
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
  'Core' / 'Src' / 'app.c',
  'Core' / 'Src' / 'main.c',
)
