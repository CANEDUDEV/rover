import argparse
import sys
import time
from pathlib import Path

import can
import cantools


def main():
    parser = argparse.ArgumentParser(
        description="dump rover CAN messages in DBC format"
    )
    parser.add_argument(
        "-l",
        "--list",
        action="store_true",
        help="list connected CAN interfaces and exit",
    )
    parser.add_argument(
        "-i", "--interface", help="type of interface to use, defaults to first found"
    )
    parser.add_argument(
        "-c", "--channel", help="channel to use, defaults to first found"
    )
    parser.add_argument(
        "-b",
        "--bitrate",
        type=int,
        default=125000,
        help="CAN bus bitrate (default: 125000)",
    )
    parser.add_argument(
        "--dbc", metavar="DBC_FILE", help="CAN DBC file for decoding messages"
    )
    parser.add_argument("--log", metavar="FILE", help="log output to file")

    args = parser.parse_args()
    Dumper().from_args(args).start()


class Dumper:
    def __init__(
        self, interface=None, channel=None, db=None, logger=None, bitrate=None
    ):
        self.interface = interface
        self.channel = channel
        self.db = db
        self.logger = logger
        self.bitrate = bitrate

    def start(self):
        try:
            bus = can.ThreadSafeBus(
                interface=self.interface, channel=self.channel, bitrate=self.bitrate
            )
        except can.exceptions.CanError as e:
            print(f"error: failed to connect to CAN bus: {e}", file=sys.stderr)
            print("\nconfig:", file=sys.stderr)
            print(f"\tinterface: {self.interface}", file=sys.stderr)
            print(f"\tchannel: {self.channel}", file=sys.stderr)
            print(f"\tbitrate: {self.bitrate}", file=sys.stderr)
            sys.exit(1)

        notifier = can.Notifier(bus, [CanPrinter(self.db)])
        if self.logger:
            notifier.add_listener(self.logger)

        try:
            while True:
                time.sleep(1)

        except KeyboardInterrupt:
            notifier.stop(timeout=30)  # in seconds
            bus.shutdown()

    def from_args(self, args):
        configs = can.detect_available_configs(interfaces=["kvaser", "socketcan"])

        if args.list:
            print("Found interfaces:")
            for interface in configs:
                print(f"\t{interface}")
            sys.exit(0)

        if len(configs) < 1:
            print("No CAN interfaces connected")
            sys.exit(1)

        self.__parse_interface(configs, args.interface, args.channel)
        self.__parse_db(args.dbc)
        self.__parse_logfile(args.log)
        self.bitrate = args.bitrate

        return self

    def __parse_interface(self, configs, interface, channel):
        if not (interface or channel):
            self.interface = configs[0]["interface"]
            self.channel = configs[0]["channel"]
            return

        found_interfaces = [c["interface"] for c in configs]
        found_channels = [c["channel"] for c in configs]

        if interface and interface not in found_interfaces:
            print(
                f"error: interface {interface} not found in {found_interfaces}",
                file=sys.stderr,
            )
            sys.exit(1)

        if channel and channel not in found_channels:
            print(
                f"error: channel {channel} not found in {found_channels}",
                file=sys.stderr,
            )
            sys.exit(1)

        for c in configs:
            # Check that the found interface and channel belong to the same config
            if c["interface"] == interface and c["channel"] == channel:
                self.interface = interface
                self.channel = channel
                return

        print(
            f"error: the combination ({interface}, {channel}) was not found in configs. configs: {configs}",
            file=sys.stderr,
        )
        sys.exit(1)

    def __parse_db(self, dbc):
        if not dbc:
            return

        try:
            self.db = cantools.db.load_file(dbc)
        except cantools.db.UnsupportedDatabaseFormatError:
            print(f"error: {dbc} is not a valid DBC file", file=sys.stderr)
            sys.exit(1)

    def __parse_logfile(self, log_file):
        if not log_file:
            return

        suffix = Path(log_file).suffix

        supported_suffixes = [
            ".asc",
            ".blf",
            ".csv",
            ".db",
            ".log",
            ".mf4",
            ".trc",
            ".txt",
        ]

        if suffix not in supported_suffixes:
            print(
                f"error: unsupported log file format: {suffix}. Must be one of {supported_suffixes}",
                file=sys.stderr,
            )
            sys.exit(1)

        try:
            self.logger = can.Logger(log_file)
        except OSError as e:
            print(f"error: couldn't open log file: {e}", file=sys.stderr)
            sys.exit(1)


class CanPrinter(can.Listener):
    def __init__(self, db):
        self.db = db

    def on_message_received(self, msg):
        print(msg)
        if self.db:
            try:
                print("\t", self.db.decode_message(msg.arbitration_id, msg.data))
            except KeyError:  # not decodable
                pass


if __name__ == "__main__":
    main()
