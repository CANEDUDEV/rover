#!/usr/bin/env bash

set -eo pipefail

usage() {
    cat <<EOF

$(basename "$0") [--help] [--rover-root DIR] [--dockerfile FILE] [--push DEST]

Build the ros-gateway docker image. Requires docker to be installed
and usable without sudo.

    --help                show this help
    --rover-root DIR      path to rover repository (default: .)
    --dockerfile FILE     path to ros-gateway dockerfile (default: ros-gateway/Dockerfile)
    --push DEST           push container to DEST

EOF

}

if ! command -v docker >/dev/null; then
    echo "error: docker executable not found"
    exit 1
fi

ROVER_ROOT=.
DOCKERFILE=ros-gateway/Dockerfile

while [[ $# -gt 0 ]]; do
    case "$1" in
    -h) ;&
    --help)
        usage
        exit 0
        ;;

    --rover-root)
        ROVER_ROOT="$2"
        shift 2
        ;;

    --dockerfile)
        DOCKERFILE="$2"
        shift 2
        ;;

    --push)
        PUSH="--push --tag $2"
        shift 2
        ;;

    *)
        usage
        exit 1
        ;;

    esac
done

if [[ -z ${ROVER_ROOT} || -z ${DOCKERFILE} ]]; then
    usage
    exit 1
fi

ROVER_BUILDER=ced-rover-builder

if ! docker buildx ls | grep "${ROVER_BUILDER}" >/dev/null; then
    docker buildx create \
        --name "${ROVER_BUILDER}" \
        --platform linux/amd64,linux/arm64 \
        --driver docker-container \
        --bootstrap
fi

# PUSH shouldn't be double quoted
# shellcheck disable=SC2086
docker buildx --builder "${ROVER_BUILDER}" build \
    -f "${DOCKERFILE}" \
    --platform linux/amd64,linux/arm64 \
    ${PUSH} "${ROVER_ROOT}"
