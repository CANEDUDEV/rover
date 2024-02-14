littlefs_inc = include_directories('littlefs')

littlefs_lib = library(
  'littlefs',
  files(
    'littlefs' / 'lfs.c',
    'littlefs' / 'lfs_util.c',
  ),
  include_directories: littlefs_inc,
)
