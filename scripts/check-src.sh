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
    find_shell_files | xargs shfmt -w -s -i 4
    .bin/yamlfmt -quiet -formatter type=basic,retain_line_breaks=true . .clang-tidy .clang-format
    isort --quiet --gitignore --profile black .
    black --quiet .
    meson fmt -i -r .
    ninja --quiet -C "${BUILD_DIR}" clang-format
    pip freeze --exclude-editable >requirements.txt

    # Check
    echo "Linting..."
    find_shell_files | xargs shellcheck -o all
    pyright .
    ninja --quiet -C "${BUILD_DIR}" clang-tidy
    check_git_index

    popd >/dev/null
}

find_shell_files() {
    git ls-files | xargs file -i | grep text/x-shellscript | cut -d ":" -f 1
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
