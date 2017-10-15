import struct
import serial
import time


# device IDs:
MAIN_DEV = 0x01
HC_DEV = 0x04
RA_DEV = 0x10
DEC_DEV = 0x11
GPS_DEV = 0xb0
RTC_DEV = 0xb2


def to_hex(data):
    return ' '.join(['%02X' % ord(c) for c in data])


class NexStarComm(object):
    def __init__(self, port, debug=False):
        self.debug = debug
        self.serial = serial.Serial(port, 9600, timeout=1)
        print "Wait until Arduino is ready..."
        time.sleep(2)

    def send_packet(self, data, resp_len):
        if self.debug:
            print "Sent:", data
        self.serial.write(data)
        self.serial.flushInput()
        resp = self.serial.read(resp_len)
        if self.debug:
            print "Recv:", resp
        return resp

    def send_passthrough(self, dest, Id, payload='', resp_len=0):
        if len(payload) > 4:
            raise ValueError("Invalid payload size")

        length = len(payload) + 1
        fill = '\0'*(4 - len(payload))
        fmt = 'cBBB%ds%ds' % (len(payload), len(fill))
        data = struct.pack(fmt, 'P', length, dest, Id, payload, fill)
        return  self.send_packet(data, resp_len)

    def get_version(self):
        ret = self.send_packet('V', 3)
        return '%d.%d' % (ord(ret[0]), ord(ret[1]))

    def get_model(self):
        ret = self.send_packet('m', 2)
        return ord(ret[0])

    def get_device_version(self, dest):
        ret = self.send_passthrough(dest, 0xfe, resp_len=3)
        return '%d.%d' % (ord(ret[0]), ord(ret[1]))

    def slew_ra(self, rate, direction):
        _id = 0x24 if direction else 0x25
        payload = struct.pack('B', rate)
        self.send_passthrough(RA_DEV, _id, payload)

    def get_eq_coords(self, precise=False):
        if precise:
            ret = self.send_packet('e', 18)
        else:
            ret = self.send_packet('E', 10)

        return [int(a, 16) for a in ret.strip('#').split(',')]

    def set_eq_coords(self, ra, dec, precise=False):
        if precise:
            self.send_packet('s%08X,%08X' % (ra, dec), 1)
        else:
            self.send_packet('S%04X,%04X' % (ra, dec), 1)

    def goto_eq_coords(self, ra, dec, precise=False):
        if precise:
            self.send_packet('r%08X,%08X' % (ra, dec), 1)
        else:
            self.send_packet('R%04X,%04X' % (ra, dec), 1)

    def cancel_goto(self):
        self.send_packet('M', 1)

    def is_slewing(self):
        ret = self.send_packet('L', 2)
        return ret[0] != '\0'


if __name__ == '__main__':
    nex = NexStarComm('/dev/ttyACM0', True)

    # print "Model:\t", nex.get_model()
    # print "Version:\t", nex.get_version()
    # print "RA version:\t", nex.get_device_version(RA_DEV)
    # print "Dec version:\t", nex.get_device_version(DEC_DEV)

    # nex.set_eq_coords(0xfedc, 0x5678)
    # nex.set_eq_coords(0xf2345600, 0xf3456700, precise=True)
    # print nex.get_eq_coords(precise=True)
    # print nex.get_eq_coords(precise=False)

    #nex.set_eq_coords(0x0000, 0x0000)
    nex.goto_eq_coords(0xff00, 0x0000)

    for i in range(10):
        #print nex.is_slewing()
        print nex.get_eq_coords(precise=True)
        time.sleep(1)

    # nex.cancel_goto()

    # time.sleep(0.1)
    # nex.cancel_goto()

    # nex.slew_ra(8, 0)
    # time.sleep(1)
    # nex.slew_ra(0, 0)

