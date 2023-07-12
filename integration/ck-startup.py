import ck
import battery
import sbus
import servo

from canlib import canlib

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    ch.writeWait(ck.default_letter, -1)
    ch.writeWait(ck.give_base_no, -1)

    ch.writeWait(battery.assign_cell_voltage_tx, -1)
    ch.writeWait(battery.assign_reg_out_current_tx, -1)
    ch.writeWait(battery.assign_vbat_out_current_tx, -1)
    ch.writeWait(battery.assign_jumper_and_fuse_conf_rx, -1)
    ch.writeWait(battery.assign_reg_out_voltage_rx, -1)
    ch.writeWait(battery.assign_output_on_off_rx, -1)
    ch.writeWait(battery.assign_report_freq_rx, -1)
    ch.writeWait(battery.assign_low_voltage_cutoff_rx, -1)

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

    ch.writeWait(sbus.assign_steering_tx, -1)
    ch.writeWait(sbus.assign_steering_trim_tx, -1)
    ch.writeWait(sbus.assign_throttle_tx, -1)
    ch.writeWait(sbus.assign_throttle_trim_tx, -1)

    ch.writeWait(ck.communicate, -1)
    ch.busOff()
