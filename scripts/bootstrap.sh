#!/usr/bin/env bash

set -eo pipefail

sudo apt -q update
sudo apt -q install -y crossbuild-essential-armhf binutils-arm-none-eabi gcc-arm-none-eabi make

pip3 install --user meson==1.0.1
meson setup --cross-file stm32f302ret6.ini build
