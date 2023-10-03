subdir('third-party')
subdir('can-kingdom')
subdir('rover')
subdir('stm32-common')
subdir('stm32-postmaster')

# All libs that depend on other libs are built here. Libs that don't have
# dependencies are built in their respective subdir.

drivers_lib = library(
  'drivers',
  drivers_src,
  include_directories: [drivers_inc, stm32_common_inc],
)

freertos_lib = library(
  'freertos',
  freertos_src,
  include_directories: [freertos_inc, stm32_common_inc],
)

stm32_common_lib = library(
  'stm32-common',
  stm32_common_src,
  include_directories: [stm32_common_inc, drivers_inc, freertos_inc],
  link_with: drivers_lib,
)

stm32_postmaster_lib = library(
  'stm32-postmaster',
  stm32_postmaster_src,
  include_directories: [stm32_postmaster_inc, stm32_common_inc, drivers_inc, can_kingdom_inc],
  link_with: drivers_lib,
)

can_kingdom_lib = library(
  'can-kingdom',
  can_kingdom_src,
  include_directories: [can_kingdom_inc, stm32_postmaster_inc],
  link_with: stm32_postmaster_lib,
)

# Convenience include dir list
common_libs_inc = [
  can_kingdom_inc,
  rover_inc,
  stm32_common_inc,
  stm32_postmaster_inc,

  # third-party
  drivers_inc,
  freertos_inc,
  printf_inc,
]

# Convenience lib list
common_libs = [
  can_kingdom_lib,
  drivers_lib,
  freertos_lib,
  printf_lib,
  rover_lib,
  stm32_common_lib,
  stm32_postmaster_lib,
]

common_libs_tidy_files = [
  can_kingdom_tidy_files,
  rover_tidy_files,
  stm32_common_tidy_files,
  stm32_postmaster_tidy_files,
]
