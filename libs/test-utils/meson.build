test_utils_inc = include_directories('include')
test_utils_src = files('src' / 'test.c')

test_utils_lib = library(
    'test-utils',
    test_utils_src,
    include_directories: test_utils_inc,
    native: true,
)

test_utils_dep = declare_dependency(
    dependencies: fff_dep,
    include_directories: test_utils_inc,
    link_with: test_utils_lib,
)
