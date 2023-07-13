import battery

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

    # Set regulated output voltage to 5 V
    ch.writeWait(battery.set_reg_out_voltage_frame(5000), -1)
    sleep(2)

    # Toggle regulated output power on / off
    ch.writeWait(battery.set_reg_pwr_off_frame, -1)
    sleep(2)
    ch.writeWait(battery.set_reg_pwr_on_frame, -1)
    sleep(2)

    # Toggle main power on / off
    ch.writeWait(battery.set_pwr_off_frame, -1)
    sleep(2)
    ch.writeWait(battery.set_pwr_on_frame, -1)
    sleep(2)

    # This should cause power to turn off due to over current protection
    ch.writeWait(battery.set_over_current_threshold_frame(0), -1)
    sleep(2)
    ch.writeWait(battery.set_over_current_threshold_frame(49500), -1)
    ch.writeWait(battery.set_pwr_on_frame, -1)
    sleep(2)

    # This should cause power to turn off due to low-voltage cutoff
    ch.writeWait(battery.set_low_voltage_cutoff_frame(4200), -1)
    sleep(2)
    ch.writeWait(battery.set_low_voltage_cutoff_frame(3200), -1)
    ch.writeWait(battery.set_pwr_on_frame, -1)
    sleep(2)

    # This will cause two reports in a row to have the same measurements.
    ch.writeWait(battery.set_monitor_period_frame(500), -1)
    sleep(2)
    ch.writeWait(battery.set_report_period_frame(1000), -1)
