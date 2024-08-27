json_inc = include_directories('include')

json_src = files('src' / 'json.c')

json_lib = library(
    'json',
    json_src,
    include_directories: json_inc,
    dependencies: arena_dep,
)

json_dep = declare_dependency(
    include_directories: json_inc,
    link_with: json_lib,
    dependencies: arena_dep,
)

subdir('tests')

json_tidy_files = [json_src, json_test_tidy_files]
