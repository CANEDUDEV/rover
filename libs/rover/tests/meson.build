rover_test_suite = 'rover'

rover_test_src = [files('rover-test.c'), rover_helpers_src]

rover_test = executable(
    'rover-test',
    rover_test_src,
    include_directories: rover_inc,
    dependencies: [can_kingdom_native_dep, test_utils_dep],
    native: true,
)

test('rover', rover_test, suite: rover_test_suite)

rover_test_tidy_files = [rover_test_src]
