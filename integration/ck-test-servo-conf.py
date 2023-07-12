import ck
import servo

from canlib import canlib
from time import sleep

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    ch.writeWait(ck.default_letter, -1)
    ch.writeWait(ck.give_base_no, -1)

    ch.writeWait(servo.assign_sensor_power_tx, -1)
    ch.writeWait(servo.assign_servo_current_tx, -1)
    ch.writeWait(servo.assign_battery_voltage_tx, -1)
    ch.writeWait(servo.assign_servo_voltage_tx, -1)
    ch.writeWait(servo.assign_h_bridge_current_tx, -1)
    ch.writeWait(servo.assign_servo_voltage_rx, -1)
    ch.writeWait(servo.assign_pwm_config_rx, -1)
    ch.writeWait(servo.assign_steering_rx, -1)
    ch.writeWait(servo.assign_steering_trim_rx, -1)
    ch.writeWait(servo.assign_report_freq_rx, -1)

    ch.writeWait(ck.communicate, -1)

    ch.writeWait(servo.set_servo_voltage_7400mv, -1)
    ch.writeWait(servo.set_pwm_conf_333hz, -1)

    ch.writeWait(servo.set_steer_trim_pulse_200, -1)
    sleep(2)
    ch.writeWait(servo.set_steer_trim_pulse_minus_200, -1)
    sleep(2)

    ch.writeWait(servo.set_steer_trim_angle_15, -1)
    sleep(2)
    ch.writeWait(servo.set_steer_trim_angle_minus_15, -1)
    sleep(2)

    # Reset trim
    ch.writeWait(servo.set_steer_trim_pulse_0, -1)
    sleep(2)

    ch.writeWait(servo.set_steer_pulse_2000, -1)
    sleep(2)
    ch.writeWait(servo.set_steer_pulse_1000, -1)
    sleep(2)

    ch.writeWait(servo.set_steer_angle_45, -1)
    sleep(2)
    ch.writeWait(servo.set_steer_angle_minus_45, -1)
    sleep(2)

    # This will cause two reports in a row to have the same measurements.
    ch.writeWait(servo.set_measure_freq_500ms, -1)
    sleep(2)
    ch.writeWait(servo.set_report_freq_1000ms, -1)
