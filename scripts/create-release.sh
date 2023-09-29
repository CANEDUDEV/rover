#!/usr/bin/env bash

set -eo pipefail

TAG=$(git describe --tags)
RELEASE_FILE="rover-release-${TAG}.zip"

# Add all .bin files
find . -type f -name "*.bin" | zip -q -j "${RELEASE_FILE}" -@

# Add DBC file
zip -q -u "${RELEASE_FILE}" rover.dbc

echo "Finished creating release."
