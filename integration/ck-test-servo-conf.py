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
    ch.writeWait(servo.assign_folder2, -1)
    ch.writeWait(servo.assign_folder3, -1)
    ch.writeWait(servo.assign_folder4, -1)
    ch.writeWait(servo.assign_folder5, -1)
    ch.writeWait(servo.assign_folder6, -1)
    ch.writeWait(servo.assign_folder7, -1)
    ch.writeWait(servo.assign_folder8, -1)
    ch.writeWait(servo.assign_folder9, -1)
    ch.writeWait(servo.assign_folder10, -1)
    ch.writeWait(ck.communicate, -1)

    ch.writeWait(servo.set_potentiometer_35, -1)
    ch.writeWait(servo.set_pwm_conf_333hz, -1)

    ch.writeWait(servo.steer_pulse_2000, -1)
    sleep(2)
    ch.writeWait(servo.steer_pulse_1000, -1)
    sleep(2)
    ch.writeWait(servo.steer_angle_90, -1)
    sleep(2)
    ch.writeWait(servo.steer_angle_minus_90, -1)

    # This will cause two reports in a row to have the same measurements.
    ch.writeWait(servo.set_measure_freq_500ms, -1)
    sleep(2)
    ch.writeWait(servo.set_report_freq_1000ms, -1)
