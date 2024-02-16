battery_test_suite = 'battery'

battery_adc_test_src = files(
  'test-adc.c',
  '..' / 'src' / 'adc.c',
  '..' / 'src' / 'jumpers.c',
)

cc_native = meson.get_compiler('c', native: true)

math_dep = cc_native.find_library('m')

battery_adc_test = executable(
  'battery-adc-test',
  battery_adc_test_src,
  include_directories: battery_monitor_inc,
  dependencies: [math_dep, test_utils_dep],
  native: true,
)

test('battery-adc', battery_adc_test, suite: battery_test_suite)

battery_test_tidy_files = battery_adc_test_src
