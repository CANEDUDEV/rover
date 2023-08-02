# Rover

This repository contains the default code for all boards in the CANEDUDEV Rover.

Board applications utilize [FreeRTOS](https://www.freertos.org/).

## Building the Board Applications

This project is built using [Meson](https://mesonbuild.com/) with the Ninja backend.

Supported OS: Ubuntu 22.04. If you're using Windows, you can utilize the Ubuntu 22.04 [WSL](https://learn.microsoft.com/en-us/windows/wsl/) distribution.

### Build Steps

1. Run `./scripts/bootstrap.sh` to set up the build environment. This step installs all dependencies and prepares the build folder.
2. Execute `meson compile -C build` to initiate the build process.
3. The build output can be found in the `build` folder.

## Building the Documentation

To build the documentation, run `meson compile -C build docs`. The HTML output is located in the `build/docs/html` folder.

## Building a Release

Generate a zip file containing the board binaries by running `meson compile -C build release`. The output is stored in the `build` folder.

## Running Tests

To run all unit tests, execute `meson test -C build`. Additionally, there are integration tests that run against the boards. Note that these tests require hardware that supports canlib, such as the [Kvaser Leaf Light](https://www.kvaser.com/product/kvaser-leaf-light-hs-v2/).

## Using STM32CubeMX to Generate Code

The hardware initialization code for various boards was initially generated using [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). STM32CubeMX projects are defined in `.ioc` files located in the `boards` directory.

Follow these steps to generate code:
1. Load a project in STM32CubeMX.
2. Click "GENERATE CODE" in the top right corner.
3. If prompted to download firmware, proceed with the download.

## Flashing Binaries onto the Board

Binaries are flashed onto the board using the [ST-LINK/V2 programmer](https://www.st.com/en/development-tools/st-link-v2.html) along with the [STM32CubeProgrammer software](https://www.st.com/en/development-tools/stm32cubeprog.html).
