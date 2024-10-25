import time

from canlib import canlib

from ..rover import Envelope, brake, rover

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    rover.start(ch)

    print("Testing report period...")
    # Set report period to 1s
    ch.writeWait(brake.set_report_period_frame(1000), -1)

    # Measure time between two reports
    t_before = time.time()

    ch.iocontrol.flush_rx_buffer()  # pyright: ignore [reportCallIssue]
    ch.readSyncSpecific(Envelope.WHEEL_FRONT_LEFT_SPEED, timeout=2000)

    ch.iocontrol.flush_rx_buffer()  # pyright: ignore [reportCallIssue]
    ch.readSyncSpecific(Envelope.WHEEL_FRONT_LEFT_SPEED, timeout=2000)

    t_after = time.time()

    assert t_after - t_before > 1

    # Restore report period
    ch.writeWait(brake.set_report_period_frame(200), -1)

    print(
        "Testing setting incorrect wheel parameters. Data should be abnormally high. Check using logger."
    )
    ch.writeWait(brake.set_wheel_parameters_frame(5, 1), -1)

    input("Press Enter to continue...")

    # Restore defaults
    ch.writeWait(brake.set_wheel_parameters_frame(45, 0.16), -1)
