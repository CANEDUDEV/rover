joystick_inc = include_directories('include')

joystick_src = files(
  'src' / 'app.c',
  'src' / 'interrupts.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
)

joystick_elf = executable(
  'joystick.elf',
  joystick_src,
  include_directories: [common_libs_inc, joystick_inc],
  link_with: common_libs,
  link_depends: app_linker_script,
  link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

joystick_bin = custom_target(
  output: 'joystick.bin',
  input: joystick_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: joystick_elf,
  build_by_default: true,
)

joystick_tidy_files = joystick_src
