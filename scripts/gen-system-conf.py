import argparse
import json
import shutil
import sys
from pathlib import Path

import rover


def main():
    args = parse_args()
    system = gen_system_base(args)
    gen_node_config(system)
    set_default_settings(system)
    write_files(system, args.output_dir)


def parse_args():
    parser = argparse.ArgumentParser()

    apps = [
        "battery-monitor",
        "brake",
        "obstacle-detector",
        "sbus-receiver",
        "servo",
    ]

    for app in apps:
        parser.add_argument(
            f"--{app}-bin",
            required=True,
            help=f"{app} binary file",
        )

    parser.add_argument(
        "-o", "--output-dir", required=True, help="directory to put generated files in"
    )
    parser.add_argument(
        "-f", "--force", action="store_true", help="delete target dir and repopulate"
    )

    return parser.parse_args()


def gen_system_base(args):
    dir = Path(args.output_dir)

    if dir.exists():
        if not args.force:
            print(f"error: {dir} is not empty", file=sys.stderr)
            sys.exit(1)

        shutil.rmtree(dir)

    dir.mkdir()

    battery_monitor = Path(args.battery_monitor_bin)
    brake = Path(args.brake_bin)
    obstacle_detector = Path(args.obstacle_detector_bin)
    sbus_receiver = Path(args.sbus_receiver_bin)
    servo = Path(args.servo_bin)

    binaries = [battery_monitor, brake, sbus_receiver, servo]

    for bin in binaries:
        if not bin.exists():
            print(f"error: {bin} does not exist", file=sys.stderr)
            sys.exit(1)

    return {
        "battery-monitor": {
            "id": rover.City.BATTERY_MONITOR,
            "binary": battery_monitor.name,
        },
        "servo": {
            "id": rover.City.SERVO,
            "binary": servo.name,
        },
        "motor": {
            "id": rover.City.MOTOR,
            "binary": servo.name,
        },
        "sbus-receiver": {
            "id": rover.City.SBUS_RECEIVER,
            "binary": sbus_receiver.name,
        },
        "wheel-front-left": {
            "id": rover.City.WHEEL_FRONT_LEFT,
            "binary": brake.name,
        },
        "wheel-front-right": {
            "id": rover.City.WHEEL_FRONT_RIGHT,
            "binary": brake.name,
        },
        "wheel-rear-left": {
            "id": rover.City.WHEEL_REAR_LEFT,
            "binary": brake.name,
        },
        "wheel-rear-right": {
            "id": rover.City.WHEEL_REAR_RIGHT,
            "binary": brake.name,
        },
        "obstacle-detector-front": {
            "id": rover.City.OBSTACLE_DETECTOR_FRONT,
            "binary": obstacle_detector.name,
        },
        "obstacle-detector-rear": {
            "id": rover.City.OBSTACLE_DETECTOR_REAR,
            "binary": obstacle_detector.name,
        },
        "ad-battery-monitor": {
            "id": rover.City.AD_BATTERY_MONITOR,
            "binary": battery_monitor.name,
        },
    }


def gen_node_config(system):
    for _, node in system.items():
        node["config"] = {
            "ck_id": {
                "city_address": node["id"],
                "base_no": rover.BASE_NUMBER,
                "base_no_has_extended_id": False,
                "base_no_is_known": True,
            },
            "assignments": [],
        }
    for assignment in rover.APP_ASSIGNMENTS:
        assignment_to_config(system, assignment)


def assignment_to_config(system, assignment):
    for _, node in system.items():
        if node["id"] == assignment.city:
            node["config"]["assignments"] += [
                {"folder": assignment.folder, "envelope": assignment.envelope}
            ]
            return


def set_default_settings(system):
    system["servo"]["config"]["settings"] = {
        "reverse": True,
    }


def write_files(system, output_dir):
    dir = Path(output_dir)
    for node, items in system.items():
        config_file = dir / f"{node}.json"
        with open(config_file, "w") as f:
            json.dump(items["config"], f, indent=4)
            f.write("\n")

    with open(dir / "system.json", "w") as f:
        json.dump(system, f, indent=4)
        f.write("\n")


if __name__ == "__main__":
    main()
