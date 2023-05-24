io_node_inc = include_directories('include')
io_node_src = files(
  'src' / 'app.c',
  'src' / 'main.c',
  'src' / 'stm32f3xx_hal_msp.c',
  'src' / 'stm32f3xx_it.c',
)

io_node_elf = executable(
  'io-node.elf',
  [common_src, third_party_src, io_node_src],
  include_directories: [common_inc, third_party_inc, io_node_inc],
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

# List of files that should be analyzed by clang-tidy.
# We typically ignore all auto-generated files that we don't modify.
io_node_tidy_files = files(
  'src' / 'app.c',
  'src' / 'main.c',
)
