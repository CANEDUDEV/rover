# Build native lib for unit testing
arena_native_lib = library(
    'arena-native',
    arena_src,
    include_directories: arena_inc,
    native: true,
)

arena_native_dep = declare_dependency(
    link_with: arena_native_lib,
    include_directories: arena_inc,
)

arena_test_suite = 'arena'

arena_test_src = files('arena-test.c')

arena_test = executable(
    'arena-test',
    arena_test_src,
    dependencies: [arena_native_dep, test_utils_dep],
    native: true,
)

test('arena', arena_test, suite: arena_test_suite)

arena_test_tidy_files = [arena_test_src]
