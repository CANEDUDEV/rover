io_node_inc = include_directories('inc')
io_node_src = files(
  'src' / 'freertos.c',
  'src' / 'freertos.c',
  'src' / 'main.c',
  'src' / 'stm32f3xx_hal_msp.c',
  'src' / 'stm32f3xx_it.c',
  'src' / 'system_stm32f3xx.c',
  'startup_stm32f302xe.s',
)

ld_script = fs.copyfile('STM32F302RETx_FLASH.ld')

io_node_elf = executable(
  'io_node.elf',
  [drivers_src, freertos_src, io_node_src],
  include_directories: [drivers_inc, freertos_inc, io_node_inc],
  link_depends: ld_script,
  link_args: ['-T'+ld_script.full_path()],
)

io_node_bin = custom_target(
  'io_node.bin',
  output: 'io_node.bin',
  input: io_node_elf,
  command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
  depends: io_node_elf,
  build_by_default: true,
)
