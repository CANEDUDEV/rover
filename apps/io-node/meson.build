io_node_inc = include_directories('include')
io_node_src = files(
  'src' / 'app.c',
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
)

io_node_elf = executable(
  'io-node.elf',
  [common_libs_src, io_node_src],
  include_directories: [common_libs_inc, io_node_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
)

io_node_bin = custom_target(
  'io-node.bin',
  output: 'io-node.bin',
  input: io_node_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: io_node_elf,
  build_by_default: true,
)

io_node_tidy_files = io_node_src
