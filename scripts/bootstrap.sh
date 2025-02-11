#!/usr/bin/env bash

set -euo pipefail

TOOL_DIR=.tools

TOOLCHAIN_URL=https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz
YAMLFMT_URL=https://github.com/google/yamlfmt/releases/download/v0.13.0/yamlfmt_0.13.0_Linux_x86_64.tar.gz

echo "Installing APT dependencies..."
sudo apt-get -qq update
sudo apt-get -qq install --no-upgrade -y \
    curl \
    clang-format \
    clang-tidy \
    doxygen \
    zip

if [[ ! -e "${TOOL_DIR}/.complete" ]]; then
    echo "Installing tools..."

    mkdir -p "${TOOL_DIR}"
    pushd "${TOOL_DIR}" >/dev/null

    curl --progress-bar -LSf "${TOOLCHAIN_URL}" | tar xJf -
    curl --progress-bar -LSf "${YAMLFMT_URL}" | tar xzf - yamlfmt

    touch .complete
    popd >/dev/null
fi

if ! command -v uv; then
    echo "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
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
