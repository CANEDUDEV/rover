import argparse
import sys

import can
import cantools

parser = argparse.ArgumentParser(description="dump rover CAN messages in DBC format")
parser.add_argument(
    "-l", "--list", action="store_true", help="list connected CAN interfaces and exit"
)
parser.add_argument(
    "-i", "--interface", help="type of interface to use, defaults to first found"
)
parser.add_argument("-c", "--channel", help="channel to use, defaults to first found")
parser.add_argument(
    "-b",
    "--bitrate",
    type=int,
    default=125000,
    help="CAN bus bitrate (default: 125000)",
)
parser.add_argument("--dbc", help="CAN DBC file for decoding messages")

args = parser.parse_args()

configs = can.detect_available_configs(interfaces=["kvaser", "socketcan"])
if len(configs) < 1:
    print("No CAN interfaces connected")
    sys.exit(1)


if args.list:
    print("Found interfaces:")
    for interface in configs:
        print(f"\t{interface}")
    sys.exit(0)

db = None
if args.dbc:
    try:
        db = cantools.db.load_file(args.dbc)
    except:
        print(f"error: {args.dbc} is not a valid DBC file", file=sys.stderr)
        sys.exit(1)

if args.interface or args.channel:
    interface = args.interface
    channel = args.channel
else:
    interface = configs[0]["interface"]
    channel = configs[0]["channel"]

with can.ThreadSafeBus(
    interface=interface, channel=channel, bitrate=args.bitrate
) as bus:
    try:
        for msg in bus:
            if db:
                print(
                    db.decode_message(msg.arbitration_id, msg.data)  # pyright: ignore
                )
            else:
                print(msg)

    except KeyboardInterrupt:
        pass
