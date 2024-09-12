json_inc = include_directories('include')

json_src = files('src' / 'json.c')

json_dep = declare_dependency(
    include_directories: json_inc,
    sources: json_src,
    dependencies: arena_dep,
)

subdir('tests')

json_tidy_files = [json_src, json_test_tidy_files]
