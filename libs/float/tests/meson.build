# Build native lib for unit testing
float_native_lib = library(
    'float-native',
    float_src,
    include_directories: float_inc,
    native: true,
)

float_native_dep = declare_dependency(
    link_with: float_native_lib,
    include_directories: float_inc,
)

float_test_suite = 'float'

float_test_src = files('float-test.c')

float_test = executable(
    'float-test',
    float_test_src,
    dependencies: [float_native_dep, test_utils_dep],
    native: true,
)

test('float', float_test, suite: float_test_suite)

float_test_tidy_files = [float_test_src]
