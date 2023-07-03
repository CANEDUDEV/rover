sbus_receiver_inc = include_directories('include')
sbus_receiver_src = files(
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
  'src' / 'sbus.c',
)

sbus_receiver_elf = executable(
  'sbus-receiver.elf',
  [common_libs_src, sbus_receiver_src],
  include_directories: [common_libs_inc, sbus_receiver_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
)

sbus_receiver_bin = custom_target(
  'sbus-receiver.bin',
  output: 'sbus-receiver.bin',
  input: sbus_receiver_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: sbus_receiver_elf,
  build_by_default: true,
)

sbus_receiver_tidy_files = sbus_receiver_src
