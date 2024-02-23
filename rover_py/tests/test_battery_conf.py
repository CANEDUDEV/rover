import time

from canlib import canlib

from ..rover import battery, rover

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    rover.start(ch)

    # Set regulated output voltage to 5 V
    ch.writeWait(battery.set_reg_out_voltage_frame(5000), -1)
    time.sleep(2)

    # Set report period to 1s
    ch.writeWait(battery.set_report_period_frame(1000), -1)

    # Measure time between two reports
    t_before = time.time()

    ch.iocontrol.flush_rx_buffer()
    ch.readSyncSpecific(rover.Envelope.BATTERY_BATTERY_OUTPUT_ENVELOPE, timeout=2000)

    ch.iocontrol.flush_rx_buffer()
    ch.readSyncSpecific(rover.Envelope.BATTERY_BATTERY_OUTPUT_ENVELOPE, timeout=2000)

    t_after = time.time()

    assert t_after - t_before > 1

    # Restore report period
    ch.writeWait(battery.set_report_period_frame(200), -1)

    # Toggle regulated output power on / off
    ch.writeWait(battery.set_reg_pwr_off_frame, -1)
    time.sleep(2)
    ch.writeWait(battery.set_reg_pwr_on_frame, -1)
    time.sleep(2)

    # Toggle main power on / off
    ch.writeWait(battery.set_pwr_off_frame, -1)
    time.sleep(2)
    ch.writeWait(battery.set_pwr_on_frame, -1)
    time.sleep(2)

    # This should cause reg output to turn off due to over current protection
    # NOTE: load must be connected to reg out to see this.
    ch.writeWait(battery.set_reg_out_overcurrent_threshold_frame(0), -1)
    time.sleep(2)
    ch.writeWait(battery.set_reg_out_overcurrent_threshold_frame(8000), -1)
    ch.writeWait(battery.set_reg_pwr_on_frame, -1)
    time.sleep(2)

    # This should cause vbat out to turn off due to over current protection
    # NOTE: load must be connected to vbat out to see this.
    ch.writeWait(battery.set_vbat_out_overcurrent_threshold_frame(0), -1)
    time.sleep(2)
    ch.writeWait(battery.set_vbat_out_overcurrent_threshold_frame(49500), -1)
    ch.writeWait(battery.set_pwr_on_frame, -1)
    time.sleep(2)

    # This should cause power to turn off due to low-voltage cutoff
    # NOTE: cells must be connected for this to work.
    ch.writeWait(battery.set_low_voltage_cutoff_frame(4200), -1)
    time.sleep(2)

    # Restore power
    ch.writeWait(battery.set_low_voltage_cutoff_frame(3200), -1)
    ch.writeWait(battery.set_pwr_on_frame, -1)
    ch.writeWait(battery.set_reg_pwr_on_frame, -1)
