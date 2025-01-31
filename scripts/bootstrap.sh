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
    zip

echo "Installing uv..."
if ! command -v uv; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi

echo "Downloading yamlfmt..."
if [[ ! -x .bin/yamlfmt ]]; then
    mkdir -p .bin
    curl -L -s "${YAMLFMT}" | tar -C .bin -xzf - yamlfmt
    chmod +x .bin/yamlfmt
fi

uv sync

# shellcheck disable=SC1091,SC2312
source .venv/bin/activate

echo "Setting up build dir..."
meson setup --cross-file stm32f302ret6.ini --wipe build

echo "
Finished bootstrapping. Next steps:

1. Enter Python virtual environment using 'source .venv/bin/activate'
2. Build source using 'meson compile -C build'
"
