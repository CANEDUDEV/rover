joystick_adapter_inc = include_directories('include')

joystick_adapter_src = files(
  'src' / 'main.c',
)

joystick_adapter_elf = executable(
  'joystick-adapter.elf',
  [common_libs_src, joystick_adapter_src],
  include_directories: [common_libs_inc, joystick_adapter_inc],
  link_with: common_libs,
  link_depends: app_linker_script,
  link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

joystick_adapter_bin = custom_target(
  output: 'joystick-adapter.bin',
  input: joystick_adapter_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: joystick_adapter_elf,
  build_by_default: true,
)

joystick_adapter_tidy_files = joystick_adapter_src
