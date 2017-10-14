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
    def __init__(self, port, debug=True):
        self.debug = debug
        self.serial = serial.Serial(port, 9600, timeout=1)
        print "Wait until Arduino is ready..."
        time.sleep(2)

    def send_packet(self, data, resp_len):
        if self.debug:
            print "Sent:", to_hex(data)
        self.serial.write(data)
        self.serial.flushInput()
        resp = self.serial.read(resp_len)
        if self.debug:
            print "Recv:", to_hex(resp)
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


if __name__ == '__main__':
    nex = NexStarComm('/dev/ttyACM0')

    print nex.get_model()
    print nex.get_version()
    print nex.get_device_version(RA_DEV)
    print nex.get_device_version(DEC_DEV)

    nex.slew_ra(8, 1)
    time.sleep(4)
    nex.slew_ra(8, 0)
    time.sleep(4)
    nex.slew_ra(0, 0)

