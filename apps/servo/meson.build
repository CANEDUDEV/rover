servo_inc = include_directories('include')
servo_src = files(
  'src' / 'adc.c',
  'src' / 'app.c',
  'src' / 'ck-data.c',
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
)

servo_elf = executable(
  'servo.elf',
  [common_libs_src, servo_src],
  include_directories: [common_libs_inc, servo_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
)

servo_bin = custom_target(
  'servo.bin',
  output: 'servo.bin',
  input: servo_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: servo_elf,
  build_by_default: true,
)

servo_tidy_files = servo_src
