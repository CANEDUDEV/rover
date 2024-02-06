#!/usr/bin/env bash

set -eo pipefail

YAMLFMT=https://github.com/google/yamlfmt/releases/download/v0.10.0/yamlfmt_0.10.0_Linux_x86_64.tar.gz

sudo apt-get -qq update
sudo apt-get -qq install --no-upgrade -y \
	curl \
	clang-format \
	clang-tidy \
	doxygen \
	gcc-arm-none-eabi \
	libc6-dev-armhf-cross \
	python3-pip \
	zip

pip install -q -r requirements.txt

if [[ ! -x .bin/yamlfmt ]]; then
	mkdir -p .bin
	curl -L -s "${YAMLFMT}" | tar -C .bin -xzf - yamlfmt
	chmod +x .bin/yamlfmt
fi

meson setup --cross-file stm32f302ret6.ini build
