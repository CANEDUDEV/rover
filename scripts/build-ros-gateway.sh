#!/usr/bin/env bash

set -eo pipefail

usage() {
    cat <<EOF

$(basename "$0") [--help] [--rover-root DIR] [--dockerfile FILE] [--push DEST]

Build the ros-gateway docker image. Requires docker to be installed
and usable without sudo.

args:
    --help      show this help
    --local     build local version for testing with tag "rover-ros-gateway-test"
    --push      push containers

default env vars:
    ROS_DISTRO=jazzy                                        ROS distro codename to build
    PLATFORMS=linux/amd64,linux/arm64                       Target platforms
    PACKAGE_BASENAME=ghcr.io/canedudev/rover/ros-gateway    Docker container base name
    BUILDER=ced-rover-builder                               Docker buildx builder
    BUILD_CONTEXT=.                                         Docker build context
    DOCKERFILE=./ros-gateway/Dockerfile                     Path to dockerfile
    VERSION_TAGS=                                           Space separated tags
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

    --local)
        LOCAL="true"
        shift 1
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
PACKAGE="${PACKAGE_BASENAME}-${ROS_DISTRO}"
VERSION_TAGS="${VERSION_TAGS:-}"

if [[ -n ${VERSION_TAGS} ]]; then
    TAG_ARGS=("--tag" "${PACKAGE}:latest")
    for tag in ${VERSION_TAGS}; do
        TAG_ARGS+=("--tag" "${PACKAGE}:${tag}")
    done
fi

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

if [[ -z ${LOCAL} ]]; then
    docker buildx --builder "${BUILDER}" build \
        -f "${DOCKERFILE}" \
        --platform "${PLATFORMS}" \
        --build-arg ROS_DISTRO="${ROS_DISTRO}" \
        "${TAG_ARGS[@]}" "${OUTPUT_ARGS[@]}" "${CI_ARGS[@]}" \
        "${BUILD_CONTEXT}"
else
    docker build \
        -f "${DOCKERFILE}" \
        -t rover-ros-gateway-test \
        --build-arg ROS_DISTRO="${ROS_DISTRO}" \
        "${BUILD_CONTEXT}"
fi
