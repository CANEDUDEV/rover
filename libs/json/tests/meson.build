# Build native lib for unit testing
json_native_lib = library(
    'json-native',
    json_src,
    include_directories: json_inc,
    native: true,
    dependencies: arena_native_dep,
)

json_native_dep = declare_dependency(
    link_with: json_native_lib,
    include_directories: json_inc,
)

json_test_suite = 'json'

json_test_src = files('json-test.c')

json_test_data = files('test-data.json')

json_test = executable(
    'json-test',
    json_test_src,
    dependencies: [json_native_dep, test_utils_dep],
    native: true,
)

test('json', json_test, args: json_test_data, suite: json_test_suite)

json_test_tidy_files = [json_test_src]
