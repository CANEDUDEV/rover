# Build native lib for unit testing
rover_native_lib = library(
    'rover-native',
    rover_helpers_src,
    include_directories: rover_inc,
    dependencies: can_kingdom_native_dep,
    native: true,
)

rover_native_dep = declare_dependency(
    dependencies: can_kingdom_native_dep,
    link_with: rover_native_lib,
    include_directories: rover_inc,
)

rover_test_suite = 'rover'

rover_test_src = files('rover-test.c')

rover_test = executable(
    'rover-test',
    rover_test_src,
    dependencies: [rover_native_dep, test_utils_dep],
    native: true,
)

test('rover', rover_test, suite: rover_test_suite)

rover_test_tidy_files = [rover_test_src]
