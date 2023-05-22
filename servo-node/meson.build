servo_node_inc = include_directories('Core' / 'Inc')
servo_node_src = files(
  'Core' / 'Src' / 'app.c',
  'Core' / 'Src' / 'main.c',
  'Core' / 'Src' / 'stm32f3xx_hal_msp.c',
  'Core' / 'Src' / 'stm32f3xx_it.c',
)

# Override the werror option since some of the warnings come from the STM32 HAL driver.
servo_node_elf = executable(
  'servo_node.elf',
  [common_src, servo_node_src],
  include_directories: [common_inc, servo_node_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
  override_options: ['werror=false'],
)

servo_node_bin = custom_target(
  'servo_node.bin',
  output: 'servo_node.bin',
  input: servo_node_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: servo_node_elf,
  build_by_default: true,
)

# List of files that should be analyzed by clang-tidy.
# We typically ignore all auto-generated files that we don't modify.
servo_node_tidy_files = files(
  'Core' / 'Src' / 'main.c',
)
