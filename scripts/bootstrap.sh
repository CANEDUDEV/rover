#!/usr/bin/env bash

set -euo pipefail

YAMLFMT=https://github.com/google/yamlfmt/releases/download/v0.13.0/yamlfmt_0.13.0_Linux_x86_64.tar.gz

echo "Installing APT dependencies..."
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

echo "Downloading yamlfmt..."
if [[ ! -x .bin/yamlfmt ]]; then
	mkdir -p .bin
	curl -L -s "${YAMLFMT}" | tar -C .bin -xzf - yamlfmt
	chmod +x .bin/yamlfmt
fi

if [[ ! -d .venv ]]; then
	echo "Creating Python virtualenv..."
	python3 -m venv .venv
fi

# shellcheck disable=SC1091,SC2312
source .venv/bin/activate

if [[ -z ${VIRTUAL_ENV} ]]; then
	exit 1
fi

echo "Populating virtualenv..."
pip3 install -q -r requirements.txt

echo "Setting up build dir..."
meson setup --cross-file stm32f302ret6.ini --wipe build

echo "
Finished bootstrapping. Next steps:

1. Enter Python virtual environment using 'source .venv/bin/activate'
2. Build source using 'meson compile -C build'
"
