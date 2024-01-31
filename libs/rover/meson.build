rover_inc = include_directories('include')

rover_helpers_src = files('src' / 'rover-helpers.c')

rover_src = [files('src' / 'rover.c'), rover_helpers_src]

# Build native lib for unit testing
rover_native_lib = library(
  'rover-native',
  [rover_helpers_src, can_kingdom_postmaster_dummy_src],
  include_directories: [can_kingdom_inc, rover_inc],
  link_with: can_kingdom_native_lib,
  native: true,
)

rover_test_suite = 'rover'

rover_test_src = files('tests' / 'rover-test.c')

rover_test = executable(
  'rover-test',
  rover_test_src,
  include_directories: [can_kingdom_inc, rover_inc, test_utils_inc],
  link_with: [rover_native_lib],
  native: true,
)

test('rover', rover_test, suite: rover_test_suite)

rover_tidy_files = [rover_src]
