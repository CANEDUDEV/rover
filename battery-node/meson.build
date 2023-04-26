battery_node_inc = include_directories('Core' / 'Inc')
battery_node_src = files(
  'Core' / 'Src' / 'app.c',
  'Core' / 'Src' / 'freertos.c',
  'Core' / 'Src' / 'main.c',
  'Core' / 'Src' / 'stm32f3xx_hal_msp.c',
  'Core' / 'Src' / 'stm32f3xx_it.c',
)

battery_node_elf = executable(
  'battery_node.elf',
  [stm32_common_src, utils_src, drivers_src, freertos_src, battery_node_src],
  include_directories: [utils_inc, drivers_inc, freertos_inc, battery_node_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
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
