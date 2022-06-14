import sdcard
import os
from machine import UART, Pin, SPI, I2C
import time

from ssd1306 import SSD1306_I2C
from gps_interface import mainloop

OUTFMT = 'nmea_%04d.log'

def main():
    i2c = I2C(0, scl=Pin(17), sda=Pin(16))
    display = SSD1306_I2C(128, 32, i2c)
    # Intialize SPI peripheral (start with 10 MHz)
    spi = SPI(0,
              baudrate=10_000_000,
              polarity=0, phase=0,
              sck=machine.Pin(6),
              mosi=machine.Pin(7),
              miso=machine.Pin(4))
    # Assign chip select (CS) pin (and start it high)
    cs = Pin(5, Pin.OUT)
    while True:
        try:
            sd = sdcard.SDCard(spi, cs)
        except:
            display.fill(0)
            display.text('No SD card',  0, 12)
            display.show()
            time.sleep(1)
            continue
        break

    vfs = os.VfsFat(sd)

    # Mount filesystem
    os.mount(vfs, "/sd")
#     print(os.listdir('/sd'))
    for i in range(1, 10000):
        outfilename = OUTFMT % i
        try:
            st = os.stat('/sd/'+outfilename)
        except OSError:
            break

    print('Logging to', outfilename)

    with open(f'/sd/{outfilename}', 'wt') as f:
        mainloop(f, i2c, display)


if __name__ == '__main__':
    main()
