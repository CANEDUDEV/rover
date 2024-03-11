import csv
import datetime
import time

import numpy
from canlib import canlib, kvadblib

from ..rover import battery, rover

# Test uses 3S battery, capacity 6000mAh and discharge rate 70C.
#
# Resistor configurations:
# 1: 0.500 ohm
# 2: 0.250 ohm
# 3: 0.167 ohm
# 4: 0.125 ohm
#
# Expected currents for 3S battery:
# 1: ~23A
# 2: ~45A
# 3: ~65A
# 4: ~83A


def main():
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        setup_test(ch)
        run_test(ch)
        teardown_test(ch)


def setup_test(ch):
    rover.start(ch)

    # Turn off power
    ch.writeWait(battery.set_reg_pwr_off_frame, -1)
    ch.writeWait(battery.set_pwr_off_frame, -1)

    # Set low voltage cutoff parameters
    ch.writeWait(battery.set_low_voltage_cutoff_frame(3700, 3000, 10_000), -1)

    # Set overcurrent protection to 100 A
    ch.writeWait(battery.set_vbat_out_overcurrent_threshold_frame(95_000), -1)

    # Set jumper conf
    ch.writeWait(battery.set_jumper_conf_frame(battery.JumperConfig.X11_ON_X12_ON), -1)

    # Set report period to 10 ms
    ch.writeWait(battery.set_report_period_frame(10), -1)

    # Wait for reg output to go to 0 volt
    time.sleep(2)


def run_test(ch):
    signal_value_map = {
        "CELL_1_VOLTAGE": [],
        "CELL_2_VOLTAGE": [],
        "CELL_3_VOLTAGE": [],
        "BATTERY_OUTPUT_VOLTAGE": [],
        "BATTERY_OUTPUT_CURRENT": [],
    }

    collect_data(ch, signal_value_map)
    print_data(signal_value_map)
    write_csv(signal_value_map)


def collect_data(ch, signal_value_map):
    db = kvadblib.Dbc("rover.dbc")

    # Start test
    ch.writeWait(battery.set_pwr_on_frame, -1)

    ch.iocontrol.flush_rx_buffer()

    # Run for 3 seconds
    start_time = time.time()
    while time.time() - start_time < 3:
        try:
            frame = ch.read(timeout=1000)
            message = db.interpret(frame)
            for signal in message:
                if signal.value == 0:
                    continue
                if signal.name in signal_value_map:
                    signal_value_map[signal.name].append(signal.value)

        except KeyboardInterrupt:
            break
        except canlib.CanNoMsg:
            break
        except kvadblib.KvdNoMessage:
            pass

    # Stop test
    ch.writeWait(battery.set_pwr_off_frame, -1)


def print_data(signal_value_map):
    # Print min, max, median, mean for all signals
    for signal in signal_value_map:
        values = signal_value_map[signal]
        print(
            f"{signal}: min: {min(values)}, max: {max(values)}, median: {numpy.median(values)}, mean: {numpy.mean(values)}"
        )


def write_csv(signal_value_map):
    # Write CSV file
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
    with open(
        f"battery-stress-test_{formatted_datetime}.csv", "w", newline=""
    ) as csv_file:
        writer = csv.writer(csv_file)
        header = []
        values = []

        for signal in signal_value_map:
            header.append(signal)
            values.append(signal_value_map[signal])

        writer.writerow(header)
        writer.writerows(zip(*values))


def teardown_test(ch):
    # Restore defaults
    ch.writeWait(battery.set_low_voltage_cutoff_frame(3700, 3200, 10_000), -1)
    ch.writeWait(battery.set_report_period_frame(200), -1)


if __name__ == "__main__":
    main()
