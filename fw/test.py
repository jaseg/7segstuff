#!/usr/bin/env python3
import serial
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

if __name__ == '__main__':
    import argparse
    import time
    import struct
    from binascii import hexlify
    
    parser = argparse.ArgumentParser()
    parser.add_argument('serial')
    args = parser.parse_args()

    ser = serial.Serial(args.serial, 2000000, timeout=0.05)

    frame_len = 4*8*8
    black, red = [0]*frame_len, [255]*frame_len
    frames = \
            [black]*10 +\
            [red]*10 +\
            [[i]*frame_len for i in range(256)] +\
            [[(i + (d//8)*8) % 256*8 for d in range(frame_len)] for i in range(256)]

    #frames = [red, black]*5
    #frames = [ x for l in [[([0]*i+[255]+[0]*(7-i))*32]*2 for i in range(8)] for x in l ]
    found_macs = set()
    while True:
        ser.write(b'\0')
        frame = receive_frame(ser)
        if len(frame) == 4:
            mac, = struct.unpack('<I', frame)
            if mac not in found_macs:
                found_macs.add(mac)
                print('Discovered new MAC: {:08x}'.format(mac))
                break
        elif len(frame) != 0:
            print('Invalid frame of length {}:'.format(len(frame)), frame)
        time.sleep(0.05)

    while True:
        for i, frame in enumerate(frames):
            formatted = format_packet(frame)
            mac, = found_macs
            mac_packet = struct.pack('<I', mac)
            framed = frame_packet(mac_packet) + frame_packet(formatted[:162]) + frame_packet(formatted[162:])
            print('sending', i, len(frame), len(formatted), len(framed))
            ser.write(framed)
            time.sleep(0.02)
        # to produce framing errors: ser.write(b'\02a\0')
