#!/usr/bin/env python3
import serial
import struct
from itertools import takewhile

def chunked(data, chunk_size):
    for i in range(0, len(data), chunk_size):
        yield data[i:i+chunk_size]

def frame_packet(data):
    if len(data) > 254:
        raise ValueError('Input too long')
    out = b''
    for run in data.split(b'\0'):
        out += bytes([len(run)+1])
        out += run
    out += b'\0'
    return out

def format_packet(data):
    out = b''
    for a, b, c, d, e, f, g, h in chunked(data, 8):
        ah, bh, ch, dh = a>>8,   b>>8,   c>>8,   d>>8
        eh, fh, gh, hh = e>>8,   f>>8,   g>>8,   h>>8
        al, bl, cl, dl = a&0xff, b&0xff, c&0xff, d&0xff
        el, fl, gl, hl = e&0xff, f&0xff, g&0xff, h&0xff
        # FIXME check order of high bits
        out += bytes([al, bl, cl, dl, el, fl, gl, hl,
            (ah<<6 | bh<<4 | ch<<2 | dh<<0)&0xff,
            (eh<<6 | fh<<4 | gh<<2 | hh<<0)&0xff])
    out += bytes([1, 0, 0, 0]) # global intensity
    return out

def chariter(ser):
    while True:
        yield ser.read(1)

def read_frame(ser):
    return b''.join(takewhile(lambda c: c and c[0], chariter(ser)))

def unstuff(data):
    out = b''
    while data:
        stuff = data[0]
        if out:
            out += b'\0'
        out += data[1:stuff]
        data = data[stuff:]
    return out

def receive_frame(ser):
    return unstuff(read_frame(ser))

def mac_frame(mac):
    return frame_packet(struct.pack('<I', mac))

def send_framebuffer(ser, mac, frame):
    formatted = format_packet(frame)
    framed = mac_frame(mac) + frame_packet(formatted[:162]) + frame_packet(formatted[162:])
    ser.write(framed)

def discover_macs(ser, count=20):
    found_macs = []
    while True:
        ser.flushInput()
        ser.write(b'\0')
        frame = receive_frame(ser)
        if len(frame) == 4:
            mac, = struct.unpack('<I', frame)
            if mac not in found_macs:
                print('Discovered new MAC: {:2} {:08x}'.format(len(found_macs), mac))
                found_macs.append(mac)
                if len(found_macs) == count:
                    return found_macs
        elif len(frame) != 0:
            print('Invalid frame of length {}:'.format(len(frame)), frame)
        time.sleep(0.05)

def parse_status_frame(frame):
    print('frame len:', len(frame))
    if not frame:
        return None
    (   firmware_version, 
        hardware_version, 
        digit_rows, 
        digit_cols, 
        uptime_s, 
        framerate_millifps, 
        uart_overruns, 
        frame_overruns, 
        invalid_frames, 
        vcc_mv, 
        temp_celsius, 
        nbits ) = struct.unpack('<4B5IhhB', frame)
    del frame
    return locals()

def fetch_status(ser, mac):
    ser.flushInput()
    ser.write(mac_frame(mac))
    ser.write(frame_packet(b'\x01'))
    return parse_status_frame(receive_frame(ser))

if __name__ == '__main__':
    import argparse
    import time
    from binascii import hexlify
    
    parser = argparse.ArgumentParser()
    parser.add_argument('serial')
    args = parser.parse_args()

    ser = serial.Serial(args.serial, 2000000, timeout=0.05)

    frame_len = 4*8*8
    black, red = [0]*frame_len, [255]*frame_len
    frames = \
            [black]
            #[[0]*i + [255]*(256-i) for i in range(257)]
            #[[(i + d)%256 for d in range(frame_len)] for i in range(256)]
            #[black]*10 +\
            #[red]*10 +\
            #[[i]*frame_len for i in range(256)] +\
            #[[(i + (d//8)*8) % 256*8 for d in range(frame_len)] for i in range(256)]

    #frames = [red, black]*5
    #frames = [ x for l in [[([0]*i+[255]+[0]*(7-i))*32]*2 for i in range(8)] for x in l ]
    found_macs = [0xdeadbeef] #discover_macs(ser, 1)
    mac, = found_macs

    import pprint
    while True:
        try:
            pprint.pprint(fetch_status(ser, mac))
        except e:
            print(e)
        for i, frame in enumerate(frames):
            send_framebuffer(ser, mac, frame)
            time.sleep(0.1)
        # to produce framing errors: ser.write(b'\02a\0')
