servo_inc = include_directories('include')
servo_src = files(
  'src' / 'adc.c',
  'src' / 'ck-data.c',
  'src' / 'ck-rx-letters.c',
  'src' / 'freertos-tasks.c',
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
  'src' / 'potentiometer.c',
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

motor_elf = executable(
  'motor.elf',
  [common_libs_src, servo_src],
  include_directories: [common_libs_inc, servo_inc],
  link_depends: stm32_linker_script,
  link_args: ['-T@0@'.format(stm32_linker_script)],
  c_args: ['-DMOTOR'],
)

motor_bin = custom_target(
  'motor.bin',
  output: 'motor.bin',
  input: motor_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: servo_elf,
  build_by_default: true,
)

servo_tidy_files = servo_src
