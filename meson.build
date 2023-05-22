project(
  'Rover',
  'c',
  meson_version: '==0.61.2',
  version: '0.1',
  default_options: [
    'buildtype=debugoptimized',
    'default_library=static',
    'warning_level=3',
    'werror=true',
    'c_std=c17',
  ]
)

objcopy = find_program('objcopy')
clang_tidy = find_program('clang-tidy')

subdir('third-party') # Pulls in freertos and drivers
subdir('common') # Pulls in can kingdom and stm32 utils

common_inc = [
  can_kingdom_inc,
  stm32_common_inc,
  stm32_postmaster_inc,
  stm32_utils_inc,

  drivers_inc,
  freertos_inc,
]

common_src = [
  can_kingdom_src,
  stm32_common_src,
  stm32_postmaster_src,
  stm32_utils_src,

  drivers_src,
  freertos_src,
]

subdir('battery-node')
subdir('io-node')
subdir('radio-node')
subdir('servo-node')

subdir('docs')

tidy_files = [
  battery_node_tidy_files,
  common_tidy_files,
  io_node_tidy_files,
  radio_node_tidy_files,
  servo_node_tidy_files,
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
  ]
)
