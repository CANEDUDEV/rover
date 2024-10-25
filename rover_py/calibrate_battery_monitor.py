import sys

from canlib import canlib

from rover import Envelope, battery


def main():
    calibration_voltage_mv = 24_000
    input(
        f"""Connect battery monitor board to DC power supply with {calibration_voltage_mv/1000}V on all cells.
Turn power on and press Enter to continue.
    """
    )

    with canlib.openChannel(
        channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.canBITRATE_125K
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        try:
            start_voltage_mv = read_cell0_voltage(ch)
            accepted_error = 1000
            error = abs(calibration_voltage_mv - start_voltage_mv)

            if error > accepted_error:
                ans = input(
                    f"""warning: measured startup voltage is {start_voltage_mv}mv.
Make sure you set the voltage to {calibration_voltage_mv/1000}V. 
Continue? [y/N] >
"""
                )
                if ans.lower() != "y":
                    sys.exit(0)

            ch.writeWait(
                battery.calibration_frame(voltage=calibration_voltage_mv), timeout=1000
            )

            voltage_mv = read_cell0_voltage(ch)

            accepted_error = 100
            error = abs(calibration_voltage_mv - voltage_mv)
            if error > accepted_error:
                print(
                    f"error: calibration failed: calibration error {error}mv is too large."
                )
                sys.exit(1)

            print("Calibration succeeded.")

        except canlib.exceptions.CanTimeout:
            print("error: no response from battery monitor. Exiting.", file=sys.stderr)


def read_cell0_voltage(ch):
    # Cell message is sent in two frames.
    # We want the first frame, where byte 0 == 0.
    ch.readSyncSpecific(Envelope.BATTERY_CELL_VOLTAGES, timeout=1000)
    received = ch.readSpecificSkip(Envelope.BATTERY_CELL_VOLTAGES)

    if received.data[0] == 1:
        ch.readSyncSpecific(Envelope.BATTERY_CELL_VOLTAGES, timeout=1000)
        received = ch.readSpecificSkip(Envelope.BATTERY_CELL_VOLTAGES)

    return int.from_bytes(received.data[1:3], byteorder="little")


if __name__ == "__main__":
    main()
