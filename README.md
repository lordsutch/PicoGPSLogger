# PicoGPSLogger
MicroPython code for building a GPS logger using a Raspberry Pi Pico

Dependencies:

- MicroPython for the Raspbery Pi Pico/RP2040
  https://www.raspberrypi.com/documentation/microcontrollers/micropython.html
- my fork of micropyGPS at https://github.com/lordsutch/micropyGPS


Supports either interfacing with a GPS device via UART or I2C/Qwiic.
Includes binary initialization code for Allystar Cynosure III or
u-blox M8/M9 GPS devices.

If you don't have an SD card interface, gps_interface.py is designed
to at least give realtime output from the GPS via a SSD1306 display.

Put Python files in the root folder of the RPi Pico (e.g. using Thonny).
If you want logging to start on boot, rename log_to_sd.py as main.py.
