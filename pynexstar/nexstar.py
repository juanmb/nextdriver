from math import pi
import struct
import serial
import time
import ephem

# device IDs:
MAIN_DEV = 0x01
HC_DEV = 0x04
RA_DEV = 0x10
DEC_DEV = 0x11
GPS_DEV = 0xb0
RTC_DEV = 0xb2


def to_hex(data):
    return ' '.join(['%02X' % ord(c) for c in data])


def nex2rad(angle, precise=False):
    max_value = 0x100000000 if precise else 0x10000
    return angle*2*pi/max_value


def normalize_angle(h):
    ih = int(h/2/pi)
    h = h - ih*2*pi
    return h + 2*pi if h < 0 else h


def rad2nex(angle, precise=False):
    angle = normalize_angle(angle)
    max_value = 0x100000000 if precise else 0x10000
    if angle < 0:
        angle += 2*pi
    return int(angle*max_value/(2*pi))


def normalize_dec(angle):
    if angle > pi:
        return angle - 2*pi
    return angle


class NexStar(object):
    def __init__(self, port, debug=False):
        self.debug = debug
        self.serial = serial.Serial(port, 9600, timeout=1.0)
        self.serial.flushInput()
        #print "Wait until Arduino is ready..."
        time.sleep(2)

    def close(self):
        self.serial.close()

    def send_packet(self, data, resp_len):
        if self.debug:
            print "Sent:", to_hex(data) if data[0] in 'PWH' else data
        self.serial.write(data)
        self.serial.flushInput()
        resp = self.serial.read(resp_len)
        if self.debug:
            print "Recv:", resp if data[0] in 'eEzZ' else to_hex(resp)
        return resp

    def send_passthrough(self, dest, Id, payload='', resp_len=0):
        if len(payload) > 4:
            raise ValueError("Invalid payload size")

        length = len(payload) + 1
        fill = '\0'*(3 - len(payload))
        fmt = 'cBBB%ds%dsB' % (len(payload), len(fill))
        data = struct.pack(fmt, 'P', length, dest, Id, payload, fill, resp_len-1)
        return  self.send_packet(data, resp_len)

    def get_version(self):
        ret = self.send_packet('V', 3)
        return '%d.%02d' % (ord(ret[0]), ord(ret[1]))

    def get_variant(self):
        ret = self.send_packet('v', 3)
        return ret[0]

    def get_model(self):
        ret = self.send_packet('m', 2)
        return ord(ret[0])

    def echo(self, char):
        ret = self.send_packet('K%c' % char, 2)
        return ret[0]

    def get_device_version(self, dest):
        ret = self.send_passthrough(dest, 0xfe, resp_len=3)
        return '%d.%02d' % (ord(ret[0]), ord(ret[1]))

    def slew_ra(self, rate, direction):
        _id = 0x24 if direction else 0x25
        payload = struct.pack('B', rate)
        self.send_passthrough(RA_DEV, _id, payload)

    def get_eq_coords(self, precise=False):
        '''Get equatorial coords in radians'''
        if precise:
            ret = self.send_packet('e', 18)
        else:
            ret = self.send_packet('E', 10)

        ra, dec = [int(a, 16) for a in ret.strip('#').split(',')]
        return nex2rad(ra, precise), normalize_dec(nex2rad(dec, precise))

    def get_az_coords(self, precise=False):
        if precise:
            ret = self.send_packet('z', 18)
        else:
            ret = self.send_packet('Z', 10)

        az, alt = [int(a, 16) for a in ret.strip('#').split(',')]
        return nex2rad(az, precise), normalize_dec(nex2rad(alt, precise))

    def sync_eq_coords(self, ra, dec, precise=False):
        coords = rad2nex(ra, precise), rad2nex(dec, precise)
        if precise:
            self.send_packet('s%08X,%08X' % coords, 1)
        else:
            self.send_packet('S%04X,%04X' % coords, 1)

    def goto_eq_coords(self, ra, dec, precise=False):
        coords = rad2nex(ra, precise), rad2nex(dec, precise)
        print "Goto eq:", ra, dec, coords
        if precise:
            self.send_packet('r%08X,%08X' % coords, 1)
        else:
            self.send_packet('R%04X,%04X' % coords, 1)

    def set_location(self, lat, lon):
        ''' Set location in radians '''
        lat = ephem.degrees(lat)
        lon = ephem.degrees(lon)

        lat_split = [int(float(s)) for s in str(lat).split(':')]
        lon_split = [int(float(s)) for s in str(lon).split(':')]
        lat_sign = int(lat_split[0] < 0)
        lon_sign = int(lon_split[0] < 0)
        lat_split[0] = abs(lat_split[0])
        lon_split[0] = abs(lon_split[0])

        args = lat_split + [lat_sign] + lon_split + [lon_sign]
        payload = struct.pack('8B', *args)
        self.send_packet('W%s' % payload, 1);

    def get_location(self):
        ret = self.send_packet('w', 9)
        lat_deg, lat_min, lat_sec, lat_sign, lon_deg, lon_min, lon_sec, lon_sign, _ = \
            struct.unpack('BBBBBBBBB', ret)

        if lat_sign:
            lat_deg *= -1

        if lon_sign:
            lon_deg *= -1

        lat = '%d:%d:%d' % (lat_deg, lat_min, lat_sec)
        lon = '%d:%d:%d' % (lon_deg, lon_min, lon_sec)
        return ephem.degrees(lat).real, ephem.degrees(lon).real

    def set_time(self, date_str):
        t = time.strptime(date_str, '%Y/%m/%d %H:%M:%S')
        payload = struct.pack('BBBBBBBB', t.tm_hour, t.tm_min, t.tm_sec,
                              t.tm_mon, t.tm_mday, t.tm_year % 2000, 0, 0)
        self.send_packet('H%s' % payload, 1);

    def get_time(self):
        ret = self.send_packet('h', 9)
        h, m, s, M, D, Y, offset, dst, _ = struct.unpack('BBBBBBBBB', ret)
        return '%d/%d/%d %d:%d:%d +%03d' % (2000+Y, M, D, h, m, s, offset)

    def cancel_goto(self):
        self.send_packet('M', 1)

    def hibernate(self):
        self.send_packet('x', 1)

    def is_slewing(self):
        ret = self.send_packet('L', 2)
        return ret[0] != '0'

    def get_tracking_mode(self):
        ret = self.send_packet('t', 2)
        return ord(ret[0])

    def set_tracking_mode(self, mode=0):
        self.send_packet('T%c' % mode, 1)

    def get_j2000_date(self):
        ret = self.send_packet('j', 12)
        return float(ret[:-1])

    def get_axis_pos(self):
        ret = self.send_packet('d', 18)
        ha_pos, dec_pos = ret[:-1].split(',')
        return float(ha_pos), float(dec_pos)

    def get_sidereal_time(self):
        ret = self.send_packet('D', 9)
        return float(ret[:-1])

    def get_long(self):
        ret = self.send_packet('?', 9)
        return float(ret[:-1])

if __name__ == '__main__':
    nex = NexStar('/dev/ttyACM0', False)
    while nex.echo('x') != 'x':
        pass

    nex.set_time(time.strftime('%Y/%m/%d %H:%M:%S'))
    nex.set_location(43.540*pi/180, -5.658*pi/180)

    print "Version:\t", nex.get_version()
    #print "Variant:\t", nex.get_variant()
    print "Model:\t", nex.get_model()
    print "Dec version:\t", nex.get_device_version(DEC_DEV)
    print "RA version:\t", nex.get_device_version(RA_DEV)
    print "Time:\t", nex.get_time()

    print nex.get_eq_coords(precise=True)

    for i in range(20):
        print nex.get_debug1(), nex.get_sidereal_time()
        time.sleep(2)

    #nex.sync_eq_coords(1.5, 0.3, precise=True)

    # nex.set_tracking_mode(0)
    # print "Tracking mode:", nex.get_tracking_mode()

    # set coords (in deg)
    # nex.goto_eq_coords(0, 0)
    # nex.cancel_goto()

    # goto coords (in deg)
    # nex.goto_eq_coords(10, 0, precise=True)
    # while nex.is_slewing():
        # time.sleep(1)
        # print nex.get_eq_coords(precise=True)
    # time.sleep(1)
    # print nex.get_eq_coords(precise=True)

    # nex.hibernate()
