from machine import UART, Pin, RTC, I2C
from micropyGPS import MicropyGPS
from ssd1306 import SSD1306_I2C

import math
import struct
import time

# TODO - refactor this so the choice of interface (I2C/UART) is
# separate from the choice of initialization code

GPS = 'U-blox'
# GPS = 'Allystar'

def dayofweek(year, month, day):
    x, y = month, day
    if x == 1:
        x = 13
        year -= 1
    elif x == 2:
        x = 14
        year -= 1
    rem = year % 100
    div = year // 100
    f = (y + int(13*(x + 1)/5.0) + rem + int(rem/4.0))
    fg = f + int(div/4.0) - 2 * div
    fdiv = f + 5 - div
    if year > 1582:
        h = fg % 7
    else:
        h = fdiv % 7
    if h == 0:
        h = 7
    return (h+5) % 7 # Convert to Python's [0, 6] range


class UBXParser:
    def __init__(self, header = b'\xB5\x62', nmea_parser=None):
        self.state = 0
        self.header0, self.header1 = header[0], header[1]
        self.msglength = self.length = 0
        self.message = b''
        self.ck_a = self.ck_b = 0
        self.msg_ck_a = self.msg_ck_b = 0
        self.messages = []
        self.nmea_messages = []
        self.nmea_parser = nmea_parser

    def verify_checksum(self):
        if self.ck_a == self.msg_ck_a and self.ck_b == self.msg_ck_b:
            return True
        else:
            print(f'Checksum discrepancy: Calc: {self.ck_a} {self.ck_b}  Message: {self.msg_ck_a} {self.msg_ck_b}')
            return False

    def update_checksum(self, byte):
        self.ck_a = (self.ck_a + byte) & 0xFF
        self.ck_b = (self.ck_b + self.ck_a) & 0xFF
        # print('CSUM:', self.ck_a, self.ck_b)

    def verify_nmea_checksum(self):
        msg, checksum = self.message[:-3], self.message[-3:]
        if not checksum.startswith(b'*'):
            print('Weird sentence:', self.message)
        else:
            csum = 0
            for ch in msg[1:]:
                csum ^= ch
            msg_csum = int(checksum[1:], 16)
            if csum != msg_csum:
                print(f'Checksum discrepancy: Calc: {csum:02X}  Message: {msg_csum:02X}')
                print(self.message)
                return False
            return True
        return False

    def parse_bytes(self, msg):
        for byte in msg:
            # print(byte, self.state)
            if self.state == 0:
                if byte == self.header0:
                    self.state = 1
                elif byte == 36:
                    self.state = 100
                    self.message = b'$'
                    if self.nmea_parser:
                        self.nmea_parser.update(chr(byte))
                #       print('In NMEA')
                else:
                    continue
            elif self.state == 100: # NMEA
                if byte == 36:
                    self.message = b'$'
                    print('New NMEA message started while reading message')
                    if self.nmea_parser:
                        self.nmea_parser.update(chr(byte))
                elif byte in range(32, 128):
                    self.message += struct.pack('<B', byte)
                    if self.nmea_parser:
                        self.nmea_parser.update(chr(byte))
                elif byte in b'\r\n':
                    if self.nmea_parser:
                        self.nmea_parser.update(chr(byte))
                    self.verify_nmea_checksum()
                    if self.message:
                        self.nmea_messages.append(self.message.decode('us-ascii'))
                        self.message = b''
                    self.state = 0
                elif byte == self.header0:
                    self.state = 1
                else:
                    self.state = 0
            elif self.state == 1:
                if byte == self.header1:
                    self.state = 2
                else:
                    self.state = 0
                    continue
            elif self.state == 2:
                self.ck_a = self.ck_b = 0
                self.groupID = byte
                self.state = 3
                self.update_checksum(byte)
            elif self.state == 3:
                self.subID = byte
                self.state = 4
                self.update_checksum(byte)
            elif self.state == 4:
                self.length = byte
                self.state = 5
                self.update_checksum(byte)
            elif self.state == 5:
                self.length |= byte*256
                self.msglength = self.length
                self.message = b''
                self.update_checksum(byte)
                self.state = 6
            elif self.state == 6:
                if not self.length:
                    self.state = 7
                else:
                    self.message += struct.pack('<B', byte)
                    self.length -= 1
                    self.update_checksum(byte)
            if self.state == 7:
                self.msg_ck_a = byte
                self.state = 8
            elif self.state == 8:
                self.msg_ck_b = byte
                self.verify_checksum()
                self.state = 0
                self.messages.append( (self.groupID, self.subID, self.msglength, self.message,
                                       self.msg_ck_a, self.msg_ck_b) )


def raw_ubx_msg(message: bytes, header=b'\xB5\x62') -> bytes:
    """Convert UBX message to the raw bytestream format including length
    and checksum.
    """
    cmd, payload = message[:2], message[2:]
    message = cmd + struct.pack('<H', len(payload)) + payload

    ck_a = ck_b = 0
    for ch in message:
        ck_a = (ck_a + ch) & 0xff
        ck_b = (ck_b + ck_a) & 0xff

    return header + message + struct.pack('<BB', ck_a, ck_b)


def hexify(msg):
    return ' '.join(f'{c:02X}' for c in msg)


def format_raw_message(msg):
    response = [hexify(msg[:4]), f'{int.from_bytes(msg[4:6], "little"):04X}']
    if msg[6:-2]:
        response.append(hexify(msg[6:-2]))
    response.append('*' + hexify(msg[-2:]))
    return ' '.join(response)


def send_ubx_msg(stream, message, header=b'\xB5\x62'):
    message = raw_ubx_msg(message, header)
    print('->', format_raw_message(message))
    stream.write(message)


class UBXI2CProtocol:
    def __init__(self, i2c, address=0x42):
        self.i2c = i2c
        self.address = address

    def any(self):
        try:
            raw_avail = self.i2c.readfrom_mem(self.address, 0xFD, 2)
        except OSError:
            return None

        return int.from_bytes(raw_avail, 'big')

    def read(self, count=None):
        avail = self.any()
        if not avail:
            return None
        try:
            data = self.i2c.readfrom(self.address, min(avail, count or avail))
        except OSError:
            return self.read(count)
#         data = self.i2c.readfrom_mem(self.address, 0xFF, avail)
#         while avail > 0:
#             data += self.i2c.readfrom_mem(self.address, 0xFF, 1)
#             avail -= 1
        return data

    def write(self, data):
        try:
            self.i2c.writeto_mem(self.address, 0xFF, data)
        except OSError:
            self.write(data)


def init_allystar(gps_io, header=b'\xF1\xD9'):
    # payload = b'\x06\x40\x00'
    messages = (
        (0x00, 0x01),  # xxGGA
        (0x01, 0x00),  # xxGLL - suppress, nothing here not in xxRMC
        (0x02, 0x01),  # xxGSA
        (0x03, 0x00),  # xxGRS - disable
        (0x05, 0x01),  # xxRMC
        (0x03, 0x01),  # xxGSV
        (0x0D, 0x01),  # xxGNS
        (0x07, 0x01),  # xxGST
        (0x08, 0x00),  # xxZDA - suppress, only needed for century
        (0x05, 0x01),  # xxVTG
        (0x0E, 0x01),  # xxTHS
        (0x09, 0x00),  # xxGBS - disable
        )

    for nmea_msg, count in messages:
        msg = struct.pack('<BBBBB', 0x06, 0x01, 0xF0, nmea_msg, count)
        send_ubx_msg(gps_io, msg, header)

    # for ubx_msg in (0x02, 0x04, 0x06, 0x07, 0x21, 0x30, 0x12):
    #     msg = struct.pack('<BBBBB', 0x6, 0x1, 0x01, ubx_msg, False)
    #     send_ubx_msg(gps_io, msg, header)

    # Use NMEA 4.10
    send_ubx_msg(gps_io, b'\x06\x43\03', header)

    # payload = struct.pack('<BB', 0x06, 0x0C)
    payload = struct.pack('<BBI', 0x06, 0x0C, 0x04108237 | 0x40)
    # payload = struct.pack('<BBI', 0x06, 0x0C, 0x01|0x02|0x04|0x10)
    send_ubx_msg(gps_io, payload, header)

    # payload = b'\x06\x0E'
    # for sat in (133, 135, 138):
    #     payload += struct.pack('<BB', sat, 1)
    # for sat in (120, 134, 126, 136, 127, 128, 129, 137, 140, 125):
    #     payload += struct.pack('<BB', sat, 0)

    # payload = b'\x06\x0E'
    # send_ubx_msg(gps_io, payload, header)

    # send_ubx_msg(gps_io, b'\x01\x21', header)

    # send_ubx_msg(gps_io, b'\x0A\x04', header)
    # Request NAV-TIMEUTC
    send_ubx_msg(gps_io, b'\x01\x21', header)


def init_ublox(gps_io, header=b'\xB5\x62'):
    # Reset IMU - UBX-CFG-ESFALG
    payload = struct.pack('<BBIIhh', 0x06, 0x56, 1 << 8, 0, 0, 0)
    send_ubx_msg(gps_io, payload, header)

    # Enable Galileo
#     cfg = (
#         (0x00, 0x08, 0x10, 0x00, 0x00010001),
#         (0x01, 0x01, 0x03, 0x00, 0x00010001),
#         (0x02, 0x04, 0x08, 0x00, 0x00010001),
#         (0x03, 0x00, 0x00, 0x00, 0x00000000),
#         (0x04, 0x00, 0x00, 0x00, 0x00000000),
#         (0x05, 0x01, 0x03, 0x00, 0x00010001),
#         (0x06, 0x08, 0x0E, 0x00, 0x00010001),
#         )

    cfg = [
        (0x00, 0x08, 0x10, 0x00, 0x01010001),
        (0x01, 0x01, 0x03, 0x00, 0x01010001),
#         (0x02, 0x04, 0x08, 0x00, 0x01000000),
        (0x02, 0x04, 0x08, 0x00, 0x01010001),
        (0x03, 0x00, 0x00, 0x00, 0x00000000),
        (0x04, 0x00, 0x00, 0x00, 0x00000000),
        (0x05, 0x00, 0x03, 0x00, 0x01050001),
#         (0x06, 0x08, 0x0E, 0x00, 0x01000000),
        (0x06, 0x08, 0x0E, 0x00, 0x01010001),
        ]
    payload = struct.pack('<BBBBBB', 0x06, 0x3E, 0x00, 0x00, 0xFF, len(cfg))
    for gnss in cfg:
        payload += struct.pack('<BBBBI', *gnss)

#     payload = struct.pack('<BB', 0x06, 0x3E)
    send_ubx_msg(gps_io, payload, header)

    # CFG-NMEA
    payload = struct.pack('<BB4BI4B2B6B', 0x06, 0x17, 0x00, 0x4B, 0x00, 0x0A, 0x00000000,
                          0x01, 0x00, 0x00, 0x00,
                          0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
#    payload = struct.pack('<BB', 0x06, 0x17)
    send_ubx_msg(gps_io, payload, header)

    # CFG-ITFM - jamming/spoofing
#     payload = struct.pack('<BBII', 0x06, 0x39, config, config2)
    payload = struct.pack('<BB', 0x06, 0x39)
    send_ubx_msg(gps_io, payload, header)

    # CFG-SBAS
    prns = (131, 133, 135, 138)
    scanmode1 = scanmode2 = 0x00
    for prn in prns:
        if 120 <= prn <= 151:
            scanmode1 |= 1 << (prn-120)
        elif 152 <= prn <= 158:
            scanmode2 |= 1 << (prn-152)

    payload = struct.pack('<BBBBBBI', 0x06, 0x16, 0x01, 0x03, 0x03, scanmode2, scanmode1)
    send_ubx_msg(gps_io, payload, header)

    messages = (
        (0x04, 0x01),  # xxRMC
        (0x02, 0x01),  # xxGSA
        (0x03, 0x01),  # xxGSV
        (0x0D, 0x01),  # xxGNS
        (0x07, 0x01),  # xxGST
        (0x08, 0x00),  # xxZDA - suppress, only needed for century
        (0x01, 0x00),  # xxGLL - suppress, nothing here not in xxRMC
        (0x05, 0x01),  # xxVTG
        (0x0E, 0x01),  # xxTHS
        (0x09, 0x00),  # xxGBS - disable
        (0x06, 0x00),  # xxGRS - disable
        )

    for (nmea_msg, freq) in messages: # (0x07, 0x08):
        msg = struct.pack('<BBBBB', 0x6, 0x1, 0xF0, nmea_msg, freq)
        send_ubx_msg(gps_io, msg, header)

    # Request NAV-TIMEUTC
    send_ubx_msg(gps_io, b'\x01\x21', header)
    return


def setup_gps(i2c):
    if GPS == 'Allystar':
        header = b'\xF1\xD9'
        gps_io = UART(0, 115200) # , tx=Pin(0), rx=Pin(1))
        init_allystar(gps_io)
    elif GPS == 'U-blox':
        header = b'\xB5\x62'
        while True:
            result = i2c.scan()
            if result:
                break
            time.sleep(0.25)

        gps_io = UBXI2CProtocol(i2c, 0x42)
        init_ublox(gps_io)

    return (gps_io, header)


def set_date(message, rtc, ts):
    ctime = time.gmtime( time.mktime( ts ) )
    weekday = (ctime[6] + 1) % 7
    ttuple = ctime[0:3] + (weekday,) + ctime[3:6] + (0,)
    print('Setting time using', message,'to', ttuple)
    rtc.datetime(ttuple)


def format_binary_message(msg):
    payload = ' '.join(f'{c:02X}' for c in msg[3])
    return f'{msg[0]:02X} {msg[1]:02X} {msg[2]:04X} {payload} *{msg[4]:02X} {msg[5]:02X}'


def mainloop(output=None, i2c=None, display=None):
    rtc = RTC()
    time_set = False

    if not i2c:
        i2c = I2C(0, scl=Pin(17), sda=Pin(16))
    gps_io, header = setup_gps(i2c)

    nmea_parser = MicropyGPS(location_formatting='dd')
    parser = UBXParser(header)

    #print(gps_io)
    last_output = (-1, -1, -1)
    last_display = time.time()

    if not display:
        display = SSD1306_I2C(128, 32, i2c)

    while True:
        count = gps_io.any()
        if count:
            sentence = gps_io.read(1024)
            if sentence:
                parser.parse_bytes(sentence)
            continue

        if len(parser.messages):
            msg = parser.messages.pop(0)
            formatted = format_binary_message(msg)
            print(formatted)
            if output:
                print(formatted, file=output)
            if msg[:2] == (0x01, 0x21):
                # NAV-TIMEUTC
                data = struct.unpack('<IIiHBBBBBB', msg[3])
                iTow, tAcc, nano, year, month, day, hour, minute, sec, valid = data
                systime = time.gmtime()
                if (valid & 0x07) == 0x07 and (not time_set or systime[0:3] != (year, month, day)):
                    ts = (year, month, day, hour, minute, sec, 0, 0)
                    print(data)
                    set_date('NAV-TIMEUTC', rtc, ts)
                    time_set = True

        if len(parser.nmea_messages):
            msg = parser.nmea_messages.pop(0)
            for ch in msg:
                nmea_parser.update(ch)
#             if 'GBS' in msg:
#                 print(msg)
#             print(msg)
            if output:
                print(msg, file=output)

        day, month, year = nmea_parser.date
        h, m, s = nmea_parser.timestamp

        new_timestamp = nmea_parser.timestamp
        if nmea_parser.timestamp != last_output or (time.time() - last_display) >= 2:
            if output:
                output.flush()

            systime = time.gmtime()
            if (not time_set or systime[0:3] != (year, month, day)) and day > 0:
                ts = (year, month, day, h, m, int(s), 0, 0)
                set_date('NMEA', rtc, ts)
                time_set = True

            last_output = new_timestamp
            last_display = time.time()
            lat, latHem = nmea_parser.latitude
            lng, lngHem = nmea_parser.longitude
            speed = int(nmea_parser.speed[2])
            heading = nmea_parser.compass_direction()
#             sats = nmea_parser.satellites_visible()
            timestr = nmea_parser.time_string()

            display.fill(0)
            if nmea_parser.fix_stat >= 1:
                display.text(f'{lat:8.4f}', 0, 0)
                display.text(latHem, 65, 0)
                display.text(f'{lng:8.4f}', 0, 8)
                display.text(lngHem, 65, 8)
    #             display.text(f'{heading}', int((13.5-len(heading)/2)*8), 0)
                display.text(f'{nmea_parser.altitude:4.0f}', 87, 0)
                display.text('m', 120, 0)
                display.text(f'{nmea_parser.satellites_in_use:02d}/{nmea_parser.satellites_in_view:02d}',
                             88, 8)
                display.text(timestr, 0, 24)
                display.text('Z', 65, 24)
                display.text(f'{speed:3.0f}', 79, 24)
                display.text('kph', 104, 24)
    #             display.text(f'{speed:2d}mph', 11*8, 8)
            else:
                display.text('No fix', 0, 8)
                display.text(str(time.time() - nmea_parser.fix_time), 0, 16)
            display.show()
            print(nmea_parser.date_string('iso'), timestr,
                  nmea_parser.fix_stat,
                  nmea_parser.fix_type,
                  nmea_parser.rangeRms,
                  nmea_parser.stdLat, nmea_parser.stdLong, nmea_parser.stdAlt,
#                   nmea_parser.hdop,
                  nmea_parser.satellites_in_use,
                  nmea_parser.satellites_in_view,
                  # nmea_parser.satellites_used, nmea_parser.satellite_data,
                  )

if __name__ == '__main__':
    mainloop()
