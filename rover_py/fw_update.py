import argparse
import sys

from canlib import canlib
from flasher import flasher

from rover import rover

parser = argparse.ArgumentParser(
    description="Flash binaries and configuration files of Rover ECUs over CAN.",
)

parser.add_argument(
    "--channel", default=0, type=int, help="CANlib CAN channel to use (default: 0)"
)

subparsers = parser.add_subparsers(required=False, dest="subcommand")

parser_system = subparsers.add_parser("system", help="Flash a complete Rover system")
parser_system.add_argument(
    "--config",
    default="system.json",
    help="Rover flasher configuration file (default: system.json)",
)
parser_system.add_argument(
    "--binary-dir",
    default="binaries",
    help="Directory for binaries (default: binaries)",
)

parser_single = subparsers.add_parser(
    "single",
    description="Interact with a single ECU.",
    help="(Advanced) Interact with a single ECU",
)
parser_single.add_argument("--id", type=int, help="Rover node id", required=True)
parser_single.add_argument("--binary", help="Path to node binary file")
parser_single.add_argument("--config", help="Path to node config file")
parser_single.add_argument(
    "--format-fs", action="store_true", help="Format node filesystem"
)

args = parser.parse_args()

config = None

if args.subcommand == "single":
    if args.id == 0:
        print("id 0 is reserved by the flasher.", file=sys.stderr)
        sys.exit(1)
    if not (args.binary or args.config or args.format_fs):
        print(
            "Incorrect arguments. At least one of --binary, --config or --format-fs must be given.",
            file=sys.stderr,
        )
        sys.exit(1)

else:
    print("Verifying system configuration...")
    try:
        config = flasher.FlasherConfig(args.config, args.binary_dir)
    except Exception as e:
        print(f"error verifying {args.config}: {e}", file=sys.stderr)
        sys.exit(1)

try:
    with canlib.openChannel(
        channel=args.channel,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:

        print("Running flasher...")

        try:
            if config:
                flasher.Flasher(ch, config).run()
            elif args.format_fs:
                flasher.Flasher(ch).format_fs(args.id)
            else:
                flasher.Flasher(ch).run_single(
                    args.id, binary_file=args.binary, config_file=args.config
                )

        except Exception as e:
            # restart all nodes
            ch.writeWait(rover.set_action_mode(mode=rover.ActionMode.RESET), 100)
            print(f"error: flashing failed: {e}", file=sys.stderr)
            sys.exit(1)

        print("Finished successfully.")

except canlib.exceptions.CanNotFound as e:
    print(f"error: CAN channel {args.channel} not found.", file=sys.stderr)
    sys.exit(1)

except canlib.exceptions.CanTimeout as e:
    print(
        f"error: CAN channel {args.channel} timed out.",
        "Check that device is connected to the Rover CAN bus and that the Rover is on.",
        file=sys.stderr,
    )
    sys.exit(1)
