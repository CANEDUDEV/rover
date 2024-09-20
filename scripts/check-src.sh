#!/usr/bin/env bash

set -eo pipefail

main() {
    SRC_DIR=$1
    BUILD_DIR=$2

    if [[ -z ${SRC_DIR} ]]; then
        SRC_DIR=.
    fi

    if [[ -z ${BUILD_DIR} ]]; then
        BUILD_DIR=build
    fi

    pushd "${SRC_DIR}" >/dev/null

    echo "Formatting..."
    shfmt -w -s -i 4 .
    .bin/yamlfmt -formatter type=basic,retain_line_breaks=true . .clang-tidy .clang-format
    isort --gitignore --profile black .
    black .
    meson fmt -i -r .
    ninja -C "${BUILD_DIR}" clang-format

    # Check
    echo "Linting..."
    shfmt -f . | grep -v ^subprojects | xargs shellcheck -o all
    pyright rover_py
    ninja -C "${BUILD_DIR}" clang-tidy
    check_git_index

    popd >/dev/null
}

# Check for untracked/modified git files
check_git_index() {
    status="$(git status --porcelain)"
    if [[ -n ${status} ]]; then
        echo "Working directory is dirty:"
        echo "${status}"
        return 1
    fi
}

main "$@"
