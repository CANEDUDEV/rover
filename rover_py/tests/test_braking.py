from time import sleep

from canlib import Frame, canlib

from ..rover import servo
from ..rover.rover import City, Rover


def set_silent_mode_frame(city):
    data = [city, 0, 0, 0x2, 0, 0, 0, 0]
    return Frame(id_=0, dlc=8, data=data)


with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    rover = Rover(ch)

    rover.start()

    # Disable SBUS receiver city's communication
    ch.writeWait(set_silent_mode_frame(City.SBUS_RECEIVER), -1)
    sleep(3)

    # Accelerate
    ch.writeWait(servo.set_throttle_pulse_frame(1600), -1)
    sleep(1)

    # Brake
    ch.writeWait(servo.set_throttle_pulse_frame(1000), -1)
    sleep(1)

    # Go back to neutral in order to start reversing
    ch.writeWait(servo.set_throttle_pulse_frame(1500), -1)
    sleep(1)

    # Reverse
    ch.writeWait(servo.set_throttle_pulse_frame(1400), -1)
    sleep(1)

    # Back to neutral. There is no braking in reverse mode.
    ch.writeWait(servo.set_throttle_pulse_frame(1500), -1)

    ch.busOff()
