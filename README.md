# AstroPSU-Pico
Firmware for the open-source astronomical Power Supply Unit (PSU), AstroPSU.

## External Libraries
All external libraries come either in this repository or as a submodule, so make sure to `git clone` with `--recursive` argument.

Apart from that, you need to install the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk/).
On Windows, you may use the Raspberry Pi Pico Visual Studio Code extension instead, which is available within the Visual Studio Code's Extension Marketplace.

## ASCOM Driver
AstroPSU comes with an ASCOM driver, which communicates over the Universal Serial Bus with the device. Driver is written in C# and is available for download [here](https://github.com/mytja/AstroPSU-ASCOM/releases).

This project can relatively easily be turned into an ASCOM Alpaca device, by just replacing the Raspberry Pi Pico 1 with Raspberry Pi Pico W. However, that'd require writing an additional ASCOM driver specifically for Alpaca, and modifying Pico's source code to enable Alpaca.

## INDI Driver
INDI driver is not planned for now.

## License
Mozilla Public License 2.0