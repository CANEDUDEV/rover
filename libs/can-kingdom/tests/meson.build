can_kingdom_postmaster_dummy_src = files('postmaster-mock.c')

# Build native lib for unit testing
can_kingdom_native_lib = library(
  'can-kingdom-native',
  can_kingdom_postmaster_dummy_src,
  dependencies: [can_kingdom_king_dep, can_kingdom_mayor_dep],
  native: true,
)

can_kingdom_native_dep = declare_dependency(
  link_with: can_kingdom_native_lib,
  include_directories: can_kingdom_inc,
)

can_kingdom_test_suite = 'can-kingdom'

can_kingdom_king_test_src = files('king-test.c')

can_kingdom_king_test = executable(
  'can-kingdom-king-test',
  can_kingdom_king_test_src,
  dependencies: [can_kingdom_native_dep, test_utils_dep],
  native: true,
)

test('ck-king', can_kingdom_king_test, suite: can_kingdom_test_suite)

can_kingdom_mayor_test_src = files('mayor-test.c')

can_kingdom_mayor_test = executable(
  'can-kingdom-mayor-test',
  can_kingdom_mayor_test_src,
  dependencies: [can_kingdom_native_dep, test_utils_dep],
  native: true,
)

test('ck-mayor', can_kingdom_mayor_test, suite: can_kingdom_test_suite)

can_kingdom_types_test_src = files('types-test.c')

can_kingdom_types_test = executable(
  'can-kingdom-types-test',
  can_kingdom_types_test_src,
  dependencies: [can_kingdom_native_dep, test_utils_dep],
  native: true,
)

test('ck-types', can_kingdom_types_test, suite: can_kingdom_test_suite)

can_kingdom_test_tidy_files = [
  can_kingdom_king_test_src,
  can_kingdom_mayor_test_src,
  can_kingdom_types_test_src,
]
