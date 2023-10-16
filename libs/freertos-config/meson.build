freertos_config_inc = include_directories('include')

freertos_lib = library(
  'freertos',
  [files('src' / 'freertos-config.c'), freertos_src],
  include_directories: [freertos_inc, freertos_config_inc],
)
