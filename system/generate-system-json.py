import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()

parser.add_argument(
    "-i",
    "--system-in",
    metavar="input.json",
    type=argparse.FileType("r"),
    required=True,
)
parser.add_argument(
    "-o",
    "--system-out",
    metavar="output.json",
    type=argparse.FileType("w"),
    required=True,
)
parser.add_argument(
    "inputs",
    metavar="config.json",
    type=argparse.FileType("r"),
    nargs="+",
    help="embeddable config",
)

args = parser.parse_args()

template = json.load(args.system_in)

for file in args.inputs:
    if file.name == args.system_in.name:
        continue

    node_name = Path(file.name).stem
    template[node_name]["config"] = json.load(file)


json.dump(template, args.system_out, indent=4)
args.system_out.write("\n")  # Ending newline
