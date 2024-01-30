subdir('third-party')
subdir('can-kingdom')
subdir('rover')
subdir('stm32-common')

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
  littlefs_inc,
]

common_libs = [
  # third-party
  littlefs_lib,
]

common_libs_tidy_files = [
  can_kingdom_tidy_files,
  rover_tidy_files,
  stm32_common_tidy_files,
]

rover_king_src = [can_kingdom_king_src, rover_src]
