obstacle_detector_inc = include_directories('include')

obstacle_detector_src = files(
    'src' / 'adc.c',
    'src' / 'ck.c',
    'src' / 'ck-data.c',
    'src' / 'interrupts.c',
    'src' / 'main.c',
    'src' / 'obstacle-detector.c',
    'src' / 'peripherals.c',
    'src' / 'report.c',
)

obstacle_detector_elf = executable(
    'obstacle-detector.elf',
    obstacle_detector_src,
    include_directories: obstacle_detector_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

obstacle_detector_bin = custom_target(
    output: 'obstacle-detector.bin',
    input: obstacle_detector_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    depends: obstacle_detector_elf,
    build_by_default: true,
)

obstacle_detector_tidy_files = [obstacle_detector_src]
