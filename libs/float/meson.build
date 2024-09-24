float_inc = include_directories('include')

float_src = files('src' / 'float.c')

float_dep = declare_dependency(
    include_directories: float_inc,
    sources: float_src,
)

subdir('tests')

float_tidy_files = [float_src, float_test_tidy_files]
