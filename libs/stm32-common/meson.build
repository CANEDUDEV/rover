stm32_common_inc = include_directories('include')

stm32_lib_src = files(
    'src' / 'assert.c',
    'src' / 'clock.c',
    'src' / 'common-interrupts.c',
    'src' / 'common-peripherals.c',
    'src' / 'device-id.c',
    'src' / 'error.c',
    'src' / 'freertos-config.c',
    'src' / 'letter-reader.c',
    'src' / 'lfs-wrapper.c',
    'src' / 'postmaster.c',
    'src' / 'print.c',
    'src' / 'spi-flash.c',
)

stm32_startup_src = files(
    'src' / 'startup_stm32f302xe.s',
    'src' / 'system_stm32f3xx.c',
)

stm32_common_src = [stm32_lib_src, stm32_startup_src]

stm32_common_dep = declare_dependency(
    include_directories: stm32_common_inc,
    sources: stm32_common_src,
)

stm32_common_tidy_files = [stm32_lib_src]
