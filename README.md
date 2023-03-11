# Devkit

Project contains default code for all CANEDUDEV boards.

Board applications use [FreeRTOS](https://www.freertos.org/).

## Building

This project is built using [meson](https://mesonbuild.com/) with the ninja backend.

Only Ubuntu-22.04 is supported. If using Windows, use the Ubuntu-22.04 [WSL](https://learn.microsoft.com/en-us/windows/wsl/) distro.

To build:

1. Run `./scripts/bootstrap.sh` to prepare the build environment. This installs all dependencies and prepares the build folder.
2. Run `ninja -C build` to build.
3. Build output is in the `build` folder.

## Using STM32CubeMX to generate code

Hardware initialization code for the various boards is generated using the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) program. STM32CubeMX projects consist of two files, one `.ioc` file and one `.mxproject` file. To modify a board's hardware initialization, load the project from within STM32CubeMX.

After loading a project, generate code by clicking on "GENERATE CODE" in the top right. If asked to download firmware, always download.

After modifying code generated with STM32CubeMX, put your code inside `USER_CODE BEGIN` and `USER_CODE END`. This will prevent STM32CubeMX from deleting your code on next auto-generation.

## Flashing binaries onto the board

Binaries are flashed onto the board using the [ST-LINK/V2 programmer](https://www.st.com/en/development-tools/st-link-v2.html) and the [STM32CubeProgrammer software](https://www.st.com/en/development-tools/stm32cubeprog.html).
