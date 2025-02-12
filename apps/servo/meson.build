servo_inc = include_directories('include')

servo_src = files(
    'src' / 'adc.c',
    'src' / 'ck-data.c',
    'src' / 'ck-rx-letters.c',
    'src' / 'failsafe.c',
    'src' / 'freertos-tasks.c',
    'src' / 'interrupts.c',
    'src' / 'main.c',
    'src' / 'peripherals.c',
    'src' / 'potentiometer.c',
    'src' / 'pwm.c',
    'src' / 'servo.c',
    'src' / 'settings.c',
)

servo_elf = executable(
    'servo.elf',
    servo_src,
    include_directories: servo_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

servo_bin = custom_target(
    output: 'servo.bin',
    input: servo_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    depends: servo_elf,
    build_by_default: true,
)

motor_elf = executable(
    'motor.elf',
    servo_src,
    include_directories: servo_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
    c_args: ['-DMOTOR'],
)

motor_bin = custom_target(
    output: 'motor.bin',
    input: motor_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    depends: motor_elf,
    build_by_default: true,
)

subdir('tests')

run_target(
    'swd-flash-servo',
    command: [
        openocd,
        '-f',
        'interface/stlink.cfg',
        '-f',
        'target/stm32f3x.cfg',
        '-c',
        'adapter speed 100000',
        '-c',
        'program @0@ verify'.format(bootloader_elf.full_path()),
        '-c',
        'program @0@ verify reset exit'.format(servo_elf.full_path()),
    ],
)

run_target(
    'swd-flash-motor',
    command: [
        openocd,
        '-f',
        openocd_config,
        '-c',
        'program @0@ verify'.format(bootloader_elf.full_path()),
        '-c',
        'program @0@ verify reset exit'.format(motor_elf.full_path()),
    ],
)

release_files += [servo_elf, servo_bin, motor_elf, motor_bin]

servo_tidy_files = [servo_src, servo_test_tidy_files]
