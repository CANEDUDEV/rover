from canlib import canlib
from rover import Rover, Envelope
import servo
import keyboard
import time

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.canSetAcceptanceFilter(Envelope.STEERING, Envelope.STEERING)
    ch.busOn()
    rover = Rover(ch)
    rover.start()

    throttle = 1500
    steering = 1500

    throttle_max = 2000
    throttle_min = 1000

    steering_max = 2000
    steering_min = 1000

    step = 10

    radio_on = False

    try:
        while True:
            if keyboard.is_pressed("up"):
                if throttle < throttle_max:
                    throttle += step

            elif keyboard.is_pressed("down"):
                if throttle > throttle_min:
                    throttle -= step

            elif keyboard.is_pressed("left"):
                if steering > steering_min:
                    steering -= step

            elif keyboard.is_pressed("right"):
                if steering < steering_max:
                    steering += step

            try:
                ch.read(timeout=10)
                radio_on = True
            except canlib.exceptions.CanNoMsg:
                if radio_on:
                    time.sleep(1)
                radio_on = False

            if not radio_on:
                ch.writeWait(servo.set_throttle_pulse_frame(throttle), -1)
                ch.writeWait(servo.set_steering_pulse_frame(steering), -1)

    except KeyboardInterrupt:
        ch.busOff()
