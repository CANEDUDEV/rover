import argparse
import sys

from canlib import canlib
from flasher import flasher

from rover import rover

default_config = "system.json"
default_bindir = "binaries"

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
    default=default_config,
    help="Rover flasher configuration file (default: system.json)",
)
parser_system.add_argument(
    "--binary-dir",
    default=default_bindir,
    help="Directory for binaries (default: binaries)",
)

parser_single = subparsers.add_parser(
    "single",
    description="Interact with a single ECU.",
    help="(Advanced) Interact with a single ECU",
)
parser_single.add_argument("--id", type=int, help="Rover node id")
parser_single.add_argument("--binary", help="Path to node binary file")
parser_single.add_argument("--config", help="Path to node config file")
parser_single.add_argument(
    "--format-fs", action="store_true", help="Format node filesystem"
)

parser_recovery = subparsers.add_parser(
    "recover",
    description="(Advanced) Recover node stuck in bootloader",
    help="(Advanced) Recover node stuck in bootloader",
)
parser_recovery.add_argument("--binary", help="Path to node binary file", required=True)
parser_recovery.add_argument("--config", help="Path to node config file", required=True)

args = parser.parse_args()

with canlib.openChannel(
    channel=args.channel,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    try:

        print("Running flasher...")

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

            f = flasher.Flasher(ch)
            if args.format_fs:
                f.format_fs(args.id)
            else:
                f.run_single(args.id, binary_file=args.binary, config_file=args.config)

        elif args.subcommand == "recover":
            print("Try to enter recovery mode...")
            flasher.Flasher(ch).enter_recovery_mode(args.binary, args.config)
            sys.exit(0)

        else:  # Both system subcommand and no subcommand
            print("Verifying system configuration...")
            bindir_arg = default_bindir
            config_arg = default_config
            if args.subcommand == "system":
                bindir_arg = args.binary_dir
                config_arg = args.config

            try:
                config = flasher.FlasherConfig(config_arg, bindir_arg)
            except Exception as e:
                print(f"error verifying {args.config}: {e}", file=sys.stderr)
                sys.exit(1)

            flasher.Flasher(ch, config).run()

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

    except Exception as e:
        # restart all nodes
        ch.writeWait(rover.set_action_mode(mode=rover.ActionMode.RESET), 100)
        print(f"error: flashing failed: {e}", file=sys.stderr)
        sys.exit(1)
