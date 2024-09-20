#!/usr/bin/env bash

set -eo pipefail

TAG=$(git describe --tags --always)
RELEASE_FILE="rover-release-${TAG}.zip"

echo "Creating release ${TAG}..."

BINARIES=()
OTHER_FILES=()
FLASHER_PATH=""

while [[ $# -gt 0 ]]; do
    case "$1" in
    --flasher-path)
        FLASHER_PATH="$2"
        shift 2
        ;;
    *)

        if [[ $1 == *.bin ]]; then
            BINARIES+=("$1")
        else
            OTHER_FILES+=("$1")
        fi
        shift
        ;;

    esac
done

if [[ -z ${FLASHER_PATH} ]]; then
    echo "Error: --flasher-path is required."
    exit 1
fi

if [[ ! -d ${FLASHER_PATH} ]]; then
    echo "Error: ${FLASHER_PATH} is not a valid directory."
    exit 1
fi

rm -rf release
rm -f "rover-release"*

mkdir release
cp "${OTHER_FILES[@]}" release
cp "${FLASHER_PATH}/fw_update.py" release

mkdir release/binaries
cp "${BINARIES[@]}" release/binaries

mkdir release/rover
cp -r "${FLASHER_PATH}/rover/"*.py release/rover

zip -q -r "${RELEASE_FILE}" release

echo "Finished creating release."
