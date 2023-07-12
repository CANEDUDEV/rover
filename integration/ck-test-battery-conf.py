import ck
import battery

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

    ch.writeWait(battery.assign_cell_voltage_tx, -1)
    ch.writeWait(battery.assign_reg_out_current_tx, -1)
    ch.writeWait(battery.assign_vbat_out_current_tx, -1)
    ch.writeWait(battery.assign_jumper_and_fuse_conf_rx, -1)
    ch.writeWait(battery.assign_reg_out_voltage_rx, -1)
    ch.writeWait(battery.assign_output_on_off_rx, -1)
    ch.writeWait(battery.assign_report_freq_rx, -1)
    ch.writeWait(battery.assign_low_voltage_cutoff_rx, -1)

    ch.writeWait(ck.communicate, -1)

    ch.writeWait(battery.set_reg_out_voltage_5v, -1)

    sleep(2)
    ch.writeWait(battery.set_reg_pwr_off, -1)
    sleep(2)
    ch.writeWait(battery.set_reg_pwr_on, -1)

    sleep(2)
    ch.writeWait(battery.set_pwr_off, -1)
    sleep(2)
    ch.writeWait(battery.set_pwr_on, -1)

    # This should cause power to turn off due to over current protection
    # if anything is connected to the output ports.
    sleep(2)
    ch.writeWait(battery.set_0ma_current, -1)
    sleep(2)
    ch.writeWait(battery.set_49500ma_current, -1)
    ch.writeWait(battery.set_pwr_on, -1)

    sleep(2)
    ch.writeWait(battery.set_4200mv_cutoff, -1)
    sleep(2)
    ch.writeWait(battery.set_3200mv_cutoff, -1)
    ch.writeWait(battery.set_pwr_on, -1)
    sleep(2)

    # This will cause two reports in a row to have the same measurements.
    ch.writeWait(battery.set_monitor_freq_500ms, -1)
    sleep(2)
    ch.writeWait(battery.set_report_freq_1000ms, -1)
