# PicoGPSLogger
MicroPython code for building a GPS logger using a Raspberry Pi Pico

Dependencies:

- MicroPython for the Raspbery Pi Pico/RP2040
  https://www.raspberrypi.com/documentation/microcontrollers/micropython.html
- my fork of micropyGPS at https://github.com/lordsutch/micropyGPS
- ssd1306.py from https://raw.githubusercontent.com/micropython/micropython/master/drivers/display/ssd1306.py
- sdcard.py from https://raw.githubusercontent.com/micropython/micropython/master/drivers/sdcard/sdcard.py

Supports either interfacing with a GPS device via UART or I2C/Qwiic.
Includes binary initialization code for Allystar Cynosure III or
u-blox M8/M9 GPS devices.

Tested with the following RP2040 boards:
- Waveshare Pico clone boards.
- Sparkfun RP2040 Thing Plus board.

GPSes:
- Sparkfun NEO-M8U breakout board.
- Sparkfun NEO-M9N breakout board.
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


## Hookup using the Sparkfun Thing Plus

If you want to use the Sparkfun RP2040 Thing Plus for a more compact
setup, you'll need to tweak things in the code:

- The Qwiic interface is on I2C1 using SDA=6, SCL=7.
- The SD card reader is on SPI1 using SCK=14, MOSI=15, MISO=12, CS=9.
