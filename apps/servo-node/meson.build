servo_node_inc = include_directories('include')
servo_node_src = files(
  'src' / 'app.c',
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
)

servo_node_elf = executable(
  'servo-node.elf',
  [common_libs_src, servo_node_src],
  include_directories: [common_libs_inc, servo_node_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
)

servo_node_bin = custom_target(
  'servo-node.bin',
  output: 'servo-node.bin',
  input: servo_node_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: servo_node_elf,
  build_by_default: true,
)

servo_node_tidy_files = servo_node_src
