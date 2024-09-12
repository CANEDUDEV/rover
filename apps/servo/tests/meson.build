servo_test_suite = 'servo'

servo_common_test_src = files('..' / 'src' / 'adc.c')

servo_adc_test_src = files('test-adc.c')

servo_adc_test = executable(
    'servo-adc-test',
    [servo_adc_test_src, servo_common_test_src],
    include_directories: servo_inc,
    dependencies: [math_dep, test_utils_dep, circuits_dep],
    native: true,
)

test('servo-adc', servo_adc_test, suite: servo_test_suite)

servo_test_src = files('..' / 'src' / 'servo.c', 'test-servo.c')

servo_test = executable(
    'servo-test',
    [servo_test_src, servo_common_test_src],
    include_directories: [servo_inc, stm32_common_inc],
    dependencies: [math_dep, test_utils_dep, circuits_dep],
    native: true,
)

test('servo', servo_test, suite: servo_test_suite)

servo_test_tidy_files = [
    servo_common_test_src,
    servo_adc_test_src,
    servo_test_src,
]
