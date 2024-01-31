can_kingdom_inc = include_directories('include')

can_kingdom_postmaster_dummy_src = files('src' / 'postmaster-dummy.c')

can_kingdom_mayor_src = files(
  'src' / 'mayor.c',
  'src' / 'types.c',
)

can_kingdom_king_src = files(
  'src' / 'king.c',
  'src' / 'types.c',
)

# Build native lib for unit testing
can_kingdom_native_lib = library(
  'can-kingdom-native',
  [can_kingdom_postmaster_dummy_src, can_kingdom_mayor_src, can_kingdom_king_src],
  include_directories: can_kingdom_inc,
  native: true,
)

can_kingdom_test_suite = 'can-kingdom'

can_kingdom_king_test_src = files('tests' / 'king-test.c')

can_kingdom_king_test = executable(
  'can-kingdom-king-test',
  can_kingdom_king_test_src,
  include_directories: [test_utils_inc, can_kingdom_inc],
  link_with: can_kingdom_native_lib,
  native: true,
)

test('ck-king', can_kingdom_king_test, suite: can_kingdom_test_suite)

can_kingdom_mayor_test_src = files('tests' / 'mayor-test.c')

can_kingdom_mayor_test = executable(
  'can-kingdom-mayor-test',
  can_kingdom_mayor_test_src,
  include_directories: [test_utils_inc, can_kingdom_inc],
  link_with: can_kingdom_native_lib,
  native: true,
)

test('ck-mayor', can_kingdom_mayor_test, suite: can_kingdom_test_suite)

can_kingdom_types_test_src = files('tests' / 'types-test.c')

can_kingdom_types_test = executable(
  'can-kingdom-types-test',
  can_kingdom_types_test_src,
  include_directories: [test_utils_inc, can_kingdom_inc],
  link_with: can_kingdom_native_lib,
  native: true,
)

test('ck-types', can_kingdom_types_test, suite: can_kingdom_test_suite)

can_kingdom_tidy_files = [
  can_kingdom_mayor_src,
  can_kingdom_king_src,
  can_kingdom_king_test_src,
  can_kingdom_mayor_test_src,
  can_kingdom_types_test_src,
  can_kingdom_postmaster_dummy_src,
]
