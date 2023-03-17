#!/usr/bin/env bash

set -eo pipefail

# Check for untracked/modified git files
check_git_index() {
	status="$(git status --porcelain)"
	if [[ -n ${status} ]]; then
		echo "Working directory is dirty:"
		echo "${status}"
		return 1
	fi
}

# Format
shfmt -w -s .
.bin/yamlfmt . .clang-tidy .clang-format
ninja -C build clang-format

# Check
shfmt -f . | xargs shellcheck -o all
ninja -C build clang-tidy
check_git_index
