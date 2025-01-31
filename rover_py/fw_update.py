import argparse
import sys
import time

from canlib import canlib
from flasher import flasher

from rover import rover

default_config = "config/system.json"
default_bindir = "binaries"


def main():
    args = parse_args()

    if args.bitrate != "125k":
        change_bitrate_125k(args)

    run_flasher(args)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Configure and flash Rover ECUs over CAN.",
    )

    parser.add_argument(
        "--channel", default=0, type=int, help="CANlib CAN channel to use (default: 0)"
    )
    parser.add_argument(
        "--bitrate",
        default="125k",
        choices=["125k", "250k", "500k", "1m"],
        help="use this option if you have changed the default bitrate",
    )
    parser.add_argument(
        "-l", "--list-online", action="store_true", help="list online nodes"
    )

    subparsers = parser.add_subparsers(required=False, dest="subcommand")

    parser_system = subparsers.add_parser(
        "system", help="flash a complete Rover system"
    )
    parser_system.add_argument(
        "--config",
        default=default_config,
        help="flasher configuration file (default: system.json)",
    )
    parser_system.add_argument(
        "--binary-dir",
        default=default_bindir,
        help="binary file directory (default: binaries)",
    )

    parser_single = subparsers.add_parser(
        "single",
        description="Flash, configure or format single ECU.",
        help="(advanced) interact with a single ECU",
    )
    parser_single.add_argument("--id", type=int, help="node ID", required=True)
    parser_single.add_argument("--binary", help="path to node binary file")
    parser_single.add_argument("--config", help="path to node config file")
    parser_single.add_argument(
        "--format-fs", action="store_true", help="format node filesystem"
    )

    parser_recovery = subparsers.add_parser(
        "recover",
        description="(Advanced) Recover node stuck in bootloader",
        help="(advanced) recover node stuck in bootloader",
    )
    parser_recovery.add_argument(
        "--binary", help="path to node binary file", required=True
    )
    parser_recovery.add_argument(
        "--config", help="path to node config file", required=True
    )

    return parser.parse_args()


def change_bitrate_125k(args):
    try:
        with canlib.openChannel(
            channel=args.channel,
            flags=canlib.Open.REQUIRE_INIT_ACCESS,
            bitrate=parse_bitrate_arg(args.bitrate),
        ) as ch:
            ch.setBusOutputControl(canlib.Driver.NORMAL)
            ch.busOn()
            ch.write(rover.change_bitrate_125kbit())
            ch.writeWait(
                rover.restart_communication(
                    skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
                ),
                100,
            )
            ch.busOff()

    except canlib.exceptions.CanTimeout as e:
        print(
            f"error: couldn't change bitrate from {args.bitrate} to 125k: {e}",
            file=sys.stderr,
        )
        sys.exit(1)

    time.sleep(0.1)


def run_flasher(args):
    with canlib.openChannel(
        channel=args.channel,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        try:
            print("Running flasher...")

            if args.list_online:
                f = flasher.Flasher(ch)
                node_ids = f.detect_online_nodes()
                print("Found nodes:")
                for id in node_ids:
                    print(f"  {id}: {rover.City(id).name}")

                sys.exit(0)

            elif args.subcommand == "single":
                run_single(ch, args)

            elif args.subcommand == "recover":
                print("Try to enter recovery mode...")
                flasher.Flasher(ch).enter_recovery_mode(args.binary, args.config)
                sys.exit(0)

            # Both system subcommand and no subcommand
            else:
                run_default(ch, args)

        except canlib.exceptions.CanNotFound:
            print(f"error: CAN channel {args.channel} not found.", file=sys.stderr)
            sys.exit(1)

        except canlib.exceptions.CanTimeout:
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


def run_single(ch, args):
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


def run_default(ch, args):
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


def parse_bitrate_arg(bitrate):
    if bitrate == "125k":
        return canlib.Bitrate.BITRATE_125K
    if bitrate == "250k":
        return canlib.Bitrate.BITRATE_250K
    if bitrate == "500k":
        return canlib.Bitrate.BITRATE_500K
    if bitrate == "1m":
        return canlib.Bitrate.BITRATE_1M


if __name__ == "__main__":
    main()
