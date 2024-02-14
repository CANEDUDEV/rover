subdir('test-utils')
subdir('can-kingdom')
subdir('rover')
subdir('stm32-common')

drivers_inc = stm32cubef3_proj.get_variable('drivers_inc')
drivers_src = stm32cubef3_proj.get_variable('drivers_src')
freertos_inc = stm32cubef3_proj.get_variable('freertos_inc')
freertos_src = stm32cubef3_proj.get_variable('freertos_src')
littlefs_dep = dependency('littlefs')

# Convenience source file list for all targets
common_libs_src = [
  can_kingdom_mayor_src,
  stm32_common_src,

  # third-party
  drivers_src,
  freertos_src,
]

# Convenience include dir list
common_libs_inc = [
  can_kingdom_inc,
  rover_inc,
  stm32_common_inc,

  # third-party
  drivers_inc,
  freertos_inc,
]

common_deps = [
  littlefs_dep,
]

common_libs_tidy_files = [
  can_kingdom_tidy_files,
  rover_tidy_files,
  stm32_common_tidy_files,
]

rover_king_src = [can_kingdom_king_src, rover_src]
