battery_node_inc = include_directories('include')
battery_node_src = files(
  'src' / 'app.c',
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
)

battery_node_elf = executable(
  'battery-node.elf',
  [common_libs_src, battery_node_src],
  include_directories: [common_libs_inc, battery_node_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
)

battery_node_bin = custom_target(
  'battery-node.bin',
  output: 'battery-node.bin',
  input: battery_node_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: battery_node_elf,
  build_by_default: true,
)

battery_node_tidy_files = battery_node_src
