# PicoGPSLogger
MicroPython code for building a GPS logger using a Raspberry Pi Pico

Dependencies:

- MicroPython for the Raspbery Pi Pico/RP2040
  https://www.raspberrypi.com/documentation/microcontrollers/micropython.html
- my fork of micropyGPS at https://github.com/lordsutch/micropyGPS


Supports either interfacing with a GPS device via UART or I2C/Qwiic.
Includes binary initialization code for Allystar Cynosure III or
u-blox M8/M9 GPS devices.

Tested with:
- Sparkfun NEO-M8U breakout board.
- GoouuuTech GT-U13 module (Allystar HD804x).

If you don't have an SD card interface, gps_interface.py is designed
to at least give realtime output from the GPS via a SSD1306 display.

Put Python files in the root folder of the RPi Pico (e.g. using Thonny).
If you want logging to start on boot, rename log_to_sd.py as main.py.


## Hookup Guide

The code is configured to use the following interfaces:

GPS either uses UART0 on the default pins (GPIO 0/1 - pins 1 and 2 on
the standard board) at 115200 baud, or I2C0 on GPIO 16/17 (pins 21 and
22) with address 0x42 (SparkFun u-blox breakouts all use this address
by default).

SSD1306 is also on I2C0, GPIO 16/17 (pins 21 and 22)

SD card reader is on SPI0, GPIO 4-7 (pins 6, 7, 9, and 10)
