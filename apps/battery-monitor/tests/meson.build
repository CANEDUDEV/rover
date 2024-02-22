battery_test_suite = 'battery'

battery_adc_test_src = files(
  'battery-test-utils.c',
  'test-adc.c',
  '..' / 'src' / 'adc.c',
  '..' / 'src' / 'jumpers.c',
)

cc_native = meson.get_compiler('c', native: true)

math_dep = cc_native.find_library('m')

battery_adc_test = executable(
  'battery-adc-test',
  battery_adc_test_src,
  include_directories: [battery_monitor_inc, stm32_common_inc],
  dependencies: [math_dep, test_utils_dep],
  native: true,
)

test('battery-adc', battery_adc_test, suite: battery_test_suite)

battery_test_src = files(
  'battery-test-utils.c',
  'test-battery.c',
  '..' / 'src' / 'adc.c',
  '..' / 'src' / 'battery.c',
  '..' / 'src' / 'jumpers.c',
)

battery_test = executable(
  'battery-test',
  battery_test_src,
  include_directories: [battery_monitor_inc, stm32_common_inc],
  dependencies: [math_dep, test_utils_dep],
  native: true,
)

test('battery', battery_test, suite: battery_test_suite)

battery_test_tidy_files = [battery_adc_test_src, battery_test_src]
