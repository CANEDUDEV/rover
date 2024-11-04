#!/usr/bin/env bash

set -eo pipefail

usage() {
    cat <<EOF

$(basename "$0") [--help] [--rover-root DIR] [--dockerfile FILE] [--push DEST]

Build the ros-gateway docker image. Requires docker to be installed
and usable without sudo.

args:
    --help                show this help
    --push                push containers

default env vars:
    ROS_DISTRO=jazzy
    PLATFORMS=linux/amd64,linux/arm64
    PACKAGE_BASENAME=ghcr.io/canedudev/rover/ros-gateway
    BUILDER=ced-rover-builder
    BUILD_CONTEXT=.
    DOCKERFILE=./ros-gateway/Dockerfile
    VERSION_TAG=latest
EOF

}

if ! command -v docker >/dev/null; then
    echo "error: docker executable not found"
    exit 1
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
    -h) ;&
    --help)
        usage
        exit 0
        ;;

    --push)
        PUSH="true"
        shift 1
        ;;
    *)
        usage
        exit 1
        ;;

    esac
done

PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
PACKAGE_BASENAME="${PACKAGE_BASENAME:-ghcr.io/canedudev/rover/ros-gateway}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BUILDER="${BUILDER:-ced-rover-builder}"
BUILD_CONTEXT="${BUILD_CONTEXT:-.}"
DOCKERFILE="${DOCKERFILE:-ros-gateway/Dockerfile}"
VERSION_TAG="${VERSION_TAG:-latest}"

PACKAGE="${PACKAGE_BASENAME}-${ROS_DISTRO}"
TAG="${PACKAGE}:${VERSION_TAG}"

CI_ARGS=()
if [[ -n ${CI} ]]; then
    CI_ARGS=(
        "--cache-from" "type=registry,ref=${PACKAGE}:buildcache"
        "--cache-to" "type=registry,ref=${PACKAGE}:buildcache,mode=max"
    )
fi

OUTPUT_ARGS=("--output" "type=cacheonly")
if [[ -n ${PUSH} ]]; then
    OUTPUT_ARGS=("--output" "type=registry")
fi

if ! docker buildx ls | grep "${BUILDER}" >/dev/null; then
    docker buildx create \
        --name "${BUILDER}" \
        --platform "${PLATFORMS}" \
        --driver docker-container \
        --bootstrap
fi

docker buildx --builder "${BUILDER}" build \
    -f "${DOCKERFILE}" \
    --platform "${PLATFORMS}" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --tag "${TAG}" \
    "${OUTPUT_ARGS[@]}" "${CI_ARGS[@]}" "${BUILD_CONTEXT}"
