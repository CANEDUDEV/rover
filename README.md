# Rover

Project contains default code for all boards in the CANEDUDEV Rover.

Board applications use [FreeRTOS](https://www.freertos.org/).

## Building the board applications

This project is built using [meson](https://mesonbuild.com/) with the ninja backend.

Only Ubuntu-22.04 is supported. If using Windows, use the Ubuntu-22.04 [WSL](https://learn.microsoft.com/en-us/windows/wsl/) distro.

To build:

1. Run `./scripts/bootstrap.sh` to prepare the build environment. This installs all dependencies and prepares the build folder.
2. Run `meson compile -C build` to build.
3. Build output is in the `build` folder.

## Building the documentation

Run `meson compile -C build docs` to build the documentation. The html output is in the `build/docs/html` folder.

## Running tests

Run `meson test -C build` to run all tests.

## Using STM32CubeMX to generate code

Hardware initialization code for the various boards is generated using the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) program. STM32CubeMX projects are defined in `.ioc` files. To modify a board's hardware initialization, load the project's `.ioc` file from within STM32CubeMX. The rover's `.ioc` files are in the `boards` directory.

After loading a project, generate code by clicking on "GENERATE CODE" in the top right. If asked to download firmware, always download.

## Flashing binaries onto the board

Binaries are flashed onto the board using the [ST-LINK/V2 programmer](https://www.st.com/en/development-tools/st-link-v2.html) and the [STM32CubeProgrammer software](https://www.st.com/en/development-tools/stm32cubeprog.html).
