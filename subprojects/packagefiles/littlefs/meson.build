project(
  'littlefs',
  'c',
  license: 'BSD-3-Clause',
  version: '2.9.0',
  default_options: [
    'c_std=c99',
  ],
)

cc = meson.get_compiler('c')

if cc.get_id() == 'msvc'
  add_project_arguments('/Zc:preprocessor', language: 'c')
endif

littlefs_inc = include_directories('.')

littlefs_src = files('lfs.c', 'lfs_util.c')

littlefs_dep = declare_dependency(
  include_directories: littlefs_inc,
  sources: littlefs_src,
)
