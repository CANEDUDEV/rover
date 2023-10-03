rover_inc = include_directories('include')

rover_src = files('src' / 'rover.c')

rover_lib = library(
  'rover',
  rover_src,
  include_directories: rover_inc,
)

rover_tidy_files = [rover_src]
