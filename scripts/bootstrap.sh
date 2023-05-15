#!/usr/bin/env bash

set -eo pipefail

YAMLFMT=https://github.com/google/yamlfmt/releases/download/v0.7.1/yamlfmt_0.7.1_Linux_x86_64.tar.gz

sudo apt-get -qq update
sudo apt-get -qq install -y \
	curl \
	clang-format \
	clang-tidy \
	doxygen \
	gcc-arm-none-eabi \
	libc6-dev-armhf-cross \
	meson \
	ninja-build \
	python3-breathe \
	python3-sphinx \
	python3-sphinx-rtd-theme \
	shellcheck \
	shfmt

if [[ ! -x .bin/yamlfmt ]]; then
	mkdir -p .bin
	curl -L -s "${YAMLFMT}" | tar -C .bin -xzf - yamlfmt
	chmod +x .bin/yamlfmt
fi

meson setup --cross-file stm32f302ret6.ini build
