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

	pushd "${SRC_DIR}"

	shfmt -w -s .
	.bin/yamlfmt -formatter type=basic,retain_line_breaks=true . .clang-tidy .clang-format
	isort --gitignore --profile black .
	black .
	meson fmt -i -r .
	ninja -C "${BUILD_DIR}" clang-format

	# Check
	shfmt -f . | grep -v ^subprojects | xargs shellcheck -o all
	pyright rover_py
	ninja -C "${BUILD_DIR}" clang-tidy
	check_git_index

	popd
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
