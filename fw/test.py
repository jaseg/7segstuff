#!/usr/bin/env python3
import serial

def chunked(data, chunk_size):
    for i in range(0, len(data), chunk_size):
        yield data[i:i+chunk_size]

def frame_packet(data, chunk_size=32, frame_char=b'\x42'):
    return frame_char + frame_char.join(chunked(data, chunk_size))

def sync_frame(sync_char=b'\x23', chunk_size=32):
    return sync_char*(chunk_size+1)

def format_packet(data):
    out = b''
    for a, b, c, d in chunked(data, 4):
        ah, bh, ch, dh = a>>8,   b>>8,   c>>8,   d>>8
        al, bl, cl, dl = a&0xff, b&0xff, c&0xff, d&0xff
        # FIXME check order of high bits
        out += bytes([al, bl, cl, dl, (ah<<6 | bh<<4 | ch<<2 | dh<<0)&0xff])
    return out

if __name__ == '__main__':
    import argparse
    import time
    
    parser = argparse.ArgumentParser()
    parser.add_argument('serial')
    args = parser.parse_args()

    ser = serial.Serial(args.serial, 2000000)

    frame_len = 4*8*8
    black, red = [0]*frame_len, [255]*frame_len
    frames = \
            [black]*10 +\
            [red]*10 +\
            [[i]*frame_len for i in range(0, 256, 4)] +\
            [[(i + (d//8)*8) % 256*8 for d in range(frame_len)] for i in range(0, 256, 16)]

    frames = [red, black]*5
    while True:
        print('Sending sync structure')
        ser.write(sync_frame())
        for i, frame in enumerate(frames):
            formatted = format_packet(frame)
            #formatted = format_packet(list(range(256)))
            framed = frame_packet(formatted)
            print('sending', i, len(frame), len(formatted), len(framed))
            ser.write(framed)
            time.sleep(0.1)
