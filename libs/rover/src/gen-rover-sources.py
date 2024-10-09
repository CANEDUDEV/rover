import argparse
import io
from pathlib import Path

from rover import rover

source_file_template = """#include "rover-assignments.h"
#include "rover-defs.h"

#define ASSIGNMENT_COUNT {assignment_count}

static rover_assignment_t assignments[ASSIGNMENT_COUNT];

static rover_kingdom_t kingdom = {{
    .assignments = assignments,
    .assignment_count = ASSIGNMENT_COUNT,
}};

rover_kingdom_t *get_rover_kingdom(void) {{
  return &kingdom;
}}

void init_rover_kingdom(void) {{
{source_contents}
}}
"""

header_file_template = """#ifndef {header_def}
#define {header_def}

#ifdef __cplusplus
extern "C" {{
#endif

{header_contents}

#ifdef __cplusplus
}}
#endif

#endif /* {header_def} */
"""

rover_prefix = "ROVER"
city_prefix = f"{rover_prefix}_CITY"
envelope_prefix = f"{rover_prefix}_ENVELOPE"
folder_prefix = f"{rover_prefix}_FOLDER"


def main():
    parser = argparse.ArgumentParser(description="Generate Rover system sources")
    parser.add_argument(
        "--header", type=argparse.FileType("w"), help="output header file"
    )
    parser.add_argument(
        "--source", type=argparse.FileType("w"), help="output source file"
    )

    args = parser.parse_args()

    source_file = args.source
    header_file = args.header

    if header_file:
        header_contents = generate_header()
        header_name = Path(header_file.name).name
        header_def = header_name.upper().replace("-", "_").replace(".", "_")
        header_file.write(
            header_file_template.format(
                header_def=header_def,
                header_contents=header_contents,
            )
        )

    if source_file:
        source_contents = generate_source()
        source_file.write(
            source_file_template.format(
                assignment_count=len(rover.APP_ASSIGNMENTS),
                source_contents=source_contents,
            )
        )


def generate_source():
    output = io.StringIO()
    output.write("  size_t index = 0;\n")

    for a in rover.APP_ASSIGNMENTS:
        output.write(f"\n  assignments[index].city = {city_prefix}_{a.city.name};\n")
        output.write(
            f"  assignments[index].envelope = {envelope_prefix}_{a.envelope.name};\n"
        )
        output.write(
            f"  assignments[index].folder = {folder_prefix}_{a.folder.prefix()}_{a.folder.name};\n"
        )
        output.write("  index++;\n")

    contents = output.getvalue()
    output.close()
    return contents


def generate_header():
    output = io.StringIO()
    output.write(f"#define {rover_prefix}_BASE_NUMBER 0x{rover.BASE_NUMBER:X}\n")

    output.write(f"\n#define {city_prefix}_COUNT {len(rover.City)}\n")
    for city in rover.City:
        output.write(f"#define {city_prefix}_{city.name} {city}\n")

    output.write(f"\n#define {envelope_prefix}_COUNT {len(rover.Envelope)}\n")
    for envelope in rover.Envelope:
        output.write(f"#define {envelope_prefix}_{envelope.name} 0x{envelope:X}\n")

    output.write(
        f"\n#define {folder_prefix}_BOOTLOADER_COUNT {len(rover.BootloaderFolder)}\n"
    )
    for folder in rover.BootloaderFolder:
        output.write(f"#define {folder_prefix}_BOOTLOADER_{folder.name} {folder}\n")

    output.write(f"\n#define {folder_prefix}_SERVO_COUNT {len(rover.ServoFolder)}\n")
    for folder in rover.ServoFolder:
        output.write(f"#define {folder_prefix}_SERVO_{folder.name} {folder}\n")

    output.write(
        f"\n#define {folder_prefix}_SBUS_RECEIVER_COUNT {len(rover.SbusReceiverFolder)}\n"
    )
    for folder in rover.SbusReceiverFolder:
        output.write(f"#define {folder_prefix}_SBUS_RECEIVER_{folder.name} {folder}\n")

    output.write(
        f"\n#define {folder_prefix}_BATTERY_MONITOR_COUNT {len(rover.BatteryMonitorFolder)}\n"
    )
    for folder in rover.BatteryMonitorFolder:
        output.write(
            f"#define {folder_prefix}_BATTERY_MONITOR_{folder.name} {folder}\n"
        )

    output.write(f"\n#define {folder_prefix}_BRAKE_COUNT {len(rover.BrakeFolder)}\n")
    for folder in rover.BrakeFolder:
        output.write(f"#define {folder_prefix}_BRAKE_{folder.name} {folder}\n")

    output.write(
        f"\n#define {folder_prefix}_OBSTACLE_DETECTOR_COUNT {len(rover.ObstacleDetectorFolder)}\n"
    )
    for folder in rover.ObstacleDetectorFolder:
        output.write(
            f"#define {folder_prefix}_OBSTACLE_DETECTOR_{folder.name} {folder}\n"
        )

    contents = output.getvalue()
    output.close()
    return contents


if __name__ == "__main__":
    main()
