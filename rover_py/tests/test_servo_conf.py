import time

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

    # Set servo settings
    ch.writeWait(servo.set_failsafe(servo.FAILSAFE_OFF), -1)
    ch.writeWait(servo.set_servo_voltage_frame(7400), -1)
    ch.writeWait(servo.set_pwm_frequency_frame(333), -1)

    # Set report period to 1s
    ch.writeWait(servo.set_report_period_frame(1000), -1)

    # Measure time between two reports
    t_before = time.time()

    ch.iocontrol.flush_rx_buffer()
    ch.readSyncSpecific(rover.Envelope.SERVO_VOLTAGE, timeout=2000)

    ch.iocontrol.flush_rx_buffer()
    ch.readSyncSpecific(rover.Envelope.SERVO_VOLTAGE, timeout=2000)

    t_after = time.time()

    assert t_after - t_before > 1

    # Restore report period
    ch.writeWait(servo.set_report_period_frame(200), -1)

    # Steer using pulse
    ch.writeWait(servo.set_steering_pulse_frame(2000), -1)
    time.sleep(2)
    ch.writeWait(servo.set_steering_pulse_frame(1000), -1)
    time.sleep(2)

    # Steer using angle
    ch.writeWait(servo.set_steering_angle_frame(45), -1)
    time.sleep(2)
    ch.writeWait(servo.set_steering_angle_frame(-45), -1)
    time.sleep(2)

    # Reverse direction then set same steering angle as before.
    # This should move servo 90 degrees total.
    ch.writeWait(servo.set_reverse_direction(), -1)
    ch.writeWait(servo.set_steering_angle_frame(0), -1)
    time.sleep(2)
    ch.writeWait(servo.set_steering_angle_frame(-45), -1)
    time.sleep(2)

    # This should set the servo position to neutral by triggering the failsafe.
    ch.writeWait(
        servo.set_failsafe(servo.FAILSAFE_ON, timeout_ms=100, pulse_mus=1500), -1
    )

    # Restore settings
    ch.writeWait(servo.set_reverse_direction(), -1)
