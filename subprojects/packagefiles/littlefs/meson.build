project(
  'littlefs',
  'c',
  version: '2.9.0',
)

littlefs_inc = include_directories('.')

littlefs_lib = library(
  'littlefs',
  files('lfs.c', 'lfs_util.c'),
  include_directories: littlefs_inc,
)

littlefs_dep = declare_dependency(
  include_directories: littlefs_inc,
  link_with: littlefs_lib,
)
