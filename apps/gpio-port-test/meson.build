gpio_port_test_src = files(
  'src' / 'main.c',
)

gpio_port_test_elf = executable(
  'gpio-port-test.elf',
  gpio_port_test_src,
  dependencies: [common_deps],
  link_depends: app_linker_script,
  link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

gpio_port_test_bin = custom_target(
  output: 'gpio-port-test.bin',
  input: gpio_port_test_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: gpio_port_test_elf,
  build_by_default: true,
)
