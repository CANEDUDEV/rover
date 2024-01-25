from time import sleep

from canlib import canlib

from ..rover import rover, servo

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    rover.start(ch)

    # Disable SBUS receiver city's communication
    ch.writeWait(rover.set_silent_mode(city=rover.City.SBUS_RECEIVER), -1)
    sleep(3)

    # Disable failsafe
    ch.writeWait(servo.set_failsafe(servo.FAILSAFE_OFF, city=rover.City.MOTOR), -1)

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

    # Re-enable failsafe
    ch.writeWait(servo.set_failsafe(servo.FAILSAFE_ON, city=rover.City.MOTOR), -1)

    ch.busOff()
