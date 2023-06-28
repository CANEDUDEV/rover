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
    ch.writeWait(battery.assign_folder2, -1)
    ch.writeWait(battery.assign_folder3, -1)
    ch.writeWait(battery.assign_folder4, -1)
    ch.writeWait(battery.assign_folder5, -1)
    ch.writeWait(battery.assign_folder7, -1)
    ch.writeWait(battery.assign_folder8, -1)
    ch.writeWait(battery.assign_folder9, -1)
    ch.writeWait(ck.communicate, -1)

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
