#!/usr/bin/env bash

set -eo pipefail

TAG=$(git describe --tags --always)
RELEASE_FILE="rover-release-${TAG}.zip"

echo "Creating release ${TAG}..."

BINARIES=()
OTHER_FILES=()
CONFIG_DIR=""
FLASHER_PATH=""
RELEASE_DIR=""

while [[ $# -gt 0 ]]; do
    case "$1" in
    --config-dir)
        CONFIG_DIR="$2"
        shift 2
        ;;
    --flasher-path)
        FLASHER_PATH="$2"
        shift 2
        ;;

    --release-dir)
        RELEASE_DIR="$2"
        shift 2
        ;;
    *)

        if [[ $1 == *.bin || $1 == *.elf ]]; then
            BINARIES+=("$1")
        else
            OTHER_FILES+=("$1")
        fi
        shift
        ;;

    esac
done

if [[ -z ${CONFIG_DIR} ]]; then
    echo "Error: --flasher-path is required."
    exit 1
fi

if [[ -z ${FLASHER_PATH} ]]; then
    echo "Error: --flasher-path is required."
    exit 1
fi

if [[ -z ${RELEASE_DIR} ]]; then
    echo "Error: --release-dir is required."
    exit 1
fi

if [[ ! -d ${FLASHER_PATH} ]]; then
    echo "Error: ${FLASHER_PATH} is not a valid directory."
    exit 1
fi

rm -rf "${RELEASE_DIR}"
rm -f "rover-release"*

mkdir release
cp "${OTHER_FILES[@]}" "${RELEASE_DIR}"
cp "${FLASHER_PATH}/fw_update.py" "${RELEASE_DIR}"
cp -r "${CONFIG_DIR}" "${RELEASE_DIR}"

mkdir "${RELEASE_DIR}"/binaries
cp "${BINARIES[@]}" "${RELEASE_DIR}"/binaries

mkdir "${RELEASE_DIR}"/rover
cp -r "${FLASHER_PATH}/rover"/*.py "${RELEASE_DIR}"/rover

mkdir "${RELEASE_DIR}"/flasher
cp -r "${FLASHER_PATH}/flasher"/*.py "${RELEASE_DIR}"/flasher

zip -q -r "${RELEASE_FILE}" release

echo "Finished creating release."
