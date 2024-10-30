import argparse

from . import bridge


def main():
    parser = argparse.ArgumentParser(
        description="Rover CAN to ROS2 bridge using socketCAN"
    )
    parser.add_argument(
        "--simulate",
        metavar="LOGFILE",
        dest="logfile",
        help="publish ROS2 messages from CAN log file",
    )
    parser.add_argument(
        "--log-level",
        help="set logging level (default: info)",
        default="info",
        choices=["error", "warn", "info"],
    )
    args = parser.parse_args()

    b = bridge.RosBridge(verbosity=args.log_level)

    try:
        if args.logfile is not None:
            b.simulate(args.logfile)
        else:
            b.start()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
