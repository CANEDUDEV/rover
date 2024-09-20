#!/usr/bin/env bash

set -eo pipefail

TAG=$(git describe --tags --always)
RELEASE_FILE="rover-release-${TAG}.zip"

echo
echo "Creating release..."

if [[ -f ${RELEASE_FILE} ]]; then
	echo
	echo "ERROR: File ${PWD}/${RELEASE_FILE} already exists."
	echo "Remove it or create a new tag before retrying."
	echo "Exiting..."
	echo
	exit 1
fi

# Add all .bin files
find . -type f -name "*.bin" | zip -q -j "${RELEASE_FILE}" -@

# Add DBC file
zip -q -u "${RELEASE_FILE}" rover.dbc

echo "Finished creating release."
