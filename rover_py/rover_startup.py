from canlib import canlib

from rover import battery, rover, servo

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    rover.start(ch)

    # Default to reversing servo direction.
    ch.writeWait(servo.set_reverse_direction(), -1)
    # 5V on regulated output if possible.
    ch.writeWait(battery.set_reg_out_voltage_frame(5000), -1)

    ch.busOff()
