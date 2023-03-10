#!/usr/bin/env bash

set -eo pipefail

# Check for CRLF line endings
check_line_endings() {
	retcode=0
	files=$(git ls-files)
	for f in ${files}; do
		if file "${f}" | grep -q CRLF; then
			echo "file ${f} has windows line endings."
			retcode=1
		fi
	done
	return "${retcode}"
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

# Format
shfmt -w -s .
.bin/yamlfmt .
ninja -C build clang-format

# Check
shfmt -f . | xargs shellcheck -o all
check_line_endings
check_git_index
