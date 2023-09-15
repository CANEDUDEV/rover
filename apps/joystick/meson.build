joystick_inc = include_directories('include')

joystick_src = files(
  'src' / 'app.c',
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
)

joystick_elf = executable(
  'joystick.elf',
  [common_libs_src, joystick_src],
  include_directories: [common_libs_inc, joystick_inc],
  link_depends: stm32_app_linker_script,
  link_args: ['-T@0@'.format(stm32_app_linker_script)],
)

joystick_bin = custom_target(
  'joystick.bin',
  output: 'joystick.bin',
  input: joystick_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: joystick_elf,
  build_by_default: true,
)

joystick_tidy_files = joystick_src
