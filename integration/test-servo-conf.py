import servo

from canlib import canlib
from time import sleep
from rover import Rover

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    rover = Rover(ch)
    rover.start()

    ch.writeWait(servo.set_servo_voltage_frame(7400), -1)
    ch.writeWait(servo.set_pwm_frequency_frame(333), -1)

    # Set trim using pulse
    ch.writeWait(servo.set_steering_trim_pulse_frame(200), -1)
    sleep(2)
    ch.writeWait(servo.set_steering_trim_pulse_frame(-200), -1)
    sleep(2)

    # Set trim using angle
    ch.writeWait(servo.set_steering_trim_angle_frame(15), -1)
    sleep(2)
    ch.writeWait(servo.set_steering_trim_angle_frame(-15), -1)
    sleep(2)

    # Reset trim
    ch.writeWait(servo.set_steering_trim_pulse_frame(0), -1)
    sleep(2)

    # Steer using pulse
    ch.writeWait(servo.set_steering_pulse_frame(2000), -1)
    sleep(2)
    ch.writeWait(servo.set_steering_pulse_frame(1000), -1)
    sleep(2)

    # Steer using angle
    ch.writeWait(servo.set_steering_angle_frame(45), -1)
    sleep(2)
    ch.writeWait(servo.set_steering_angle_frame(-45), -1)
    sleep(2)

    # This will cause two reports in a row to have the same measurements.
    ch.writeWait(servo.set_measure_period_frame(500), -1)
    sleep(2)
    ch.writeWait(servo.set_report_period_frame(1000), -1)