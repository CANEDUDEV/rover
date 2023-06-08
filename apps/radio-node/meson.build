radio_node_inc = include_directories('include')
radio_node_src = files(
  'src' / 'main.c',
  'src' / 'peripherals.c',
)

radio_node_elf = executable(
  'radio-node.elf',
  [common_libs_src, radio_node_src],
  include_directories: [common_libs_inc, radio_node_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
)

radio_node_bin = custom_target(
  'radio-node.bin',
  output: 'radio-node.bin',
  input: radio_node_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: radio_node_elf,
  build_by_default: true,
)

radio_node_tidy_files = radio_node_src
