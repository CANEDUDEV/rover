battery_monitor_inc = include_directories('include')

battery_monitor_src = files(
  'src' / 'adc.c',
  'src' / 'battery.c',
  'src' / 'ck-data.c',
  'src' / 'ck-rx-letters.c',
  'src' / 'freertos-tasks.c',
  'src' / 'interrupts.c',
  'src' / 'led.c',
  'src' / 'main.c',
  'src' / 'peripherals.c',
  'src' / 'potentiometer.c',
)

battery_monitor_elf = executable(
  'battery-monitor.elf',
  [common_libs_src, battery_monitor_src],
  include_directories: [common_libs_inc, battery_monitor_inc],
  link_depends: app_linker_script,
  link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

battery_monitor_bin = custom_target(
  output: 'battery-monitor.bin',
  input: battery_monitor_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: battery_monitor_elf,
  build_by_default: true,
)

battery_monitor_tidy_files = battery_monitor_src
