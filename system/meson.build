system_gen_script = files('generate-system-json.py')

system_template = files('system-base.json')

system_src = files(
    'battery-monitor.json',
    'motor.json',
    'sbus-receiver.json',
    'servo.json',
    'wheel-front-left.json',
    'wheel-front-right.json',
    'wheel-rear-left.json',
    'wheel-rear-right.json',
)

system_config = custom_target(
    'system-config',
    input: system_src,
    output: 'system.json',
    command: [
        python,
        system_gen_script,
        '-i',
        system_template,
        '-o',
        '@OUTPUT@',
        '@INPUT@',
    ],
)

release_files += system_config
