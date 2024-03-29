project(
  'Rover',
  'c',
  meson_version: '>=1.2.0',
  version: '0.1',
  default_options: [
    'buildtype=debugoptimized',
    'default_library=static',
    'warning_level=3',
    'werror=true',
    'c_std=c17',
  ],
)

objcopy = find_program('objcopy')
clang_tidy = find_program('clang-tidy')
cat = find_program('cat')

stm32f3_hal_dep = dependency('stm32f3-hal')
freertos_dep = dependency('freertos')
littlefs_dep = dependency('littlefs')
fff_dep = dependency('fff')

common_deps = [
  stm32f3_hal_dep,
  freertos_dep,
  littlefs_dep,
]

# Order is important
fs = import('fs')
subdir('libs')
subdir('bootloader')
subdir('apps')
subdir('docs')

tidy_files = [
  apps_tidy_files,
  bootloader_tidy_files,
  common_libs_tidy_files,
]

# Override default meson clang-tidy target
run_target(
  'clang-tidy',
  command: [
    clang_tidy,
    '--config-file', meson.project_source_root() / '.clang-tidy',
    '--quiet',
    '-p', meson.project_build_root(),
    tidy_files,
  ],
)

binary_files = [battery_monitor_bin, joystick_bin, sbus_receiver_bin, servo_bin, motor_bin]

rover_dbc = fs.copyfile(files('rover.dbc'))

run_target(
  'release',
  command: [
    'bash',
    meson.project_source_root() / 'scripts' / 'create-release.sh',
  ],
  depends: [binary_files, rover_dbc],
)
