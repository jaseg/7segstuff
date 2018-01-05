#ifndef __TRANSPOSE_H__
#define __TRANSPOSE_H__

#include <stdint.h>

enum Segment { SegA, SegB, SegC, SegD, SegE, SegF, SegG, SegDP, NSEGMENTS };
enum {
    NROWS = 4,
    NCOLS = 8,
    MAX_BITS = 10,
    MIN_BITS = 6,
};

enum {
    FRAME_SIZE_WORDS = NROWS*NCOLS*NSEGMENTS/32,
};

/* Framebuffer data format pre-formatted for BCM ISRs */
struct framebuf {
    /* Multiplexing order: first Digits, then Time/bits, last Segments */
    union {
        uint32_t data[MAX_BITS*FRAME_SIZE_WORDS];
        struct {
            struct {
                uint32_t data[FRAME_SIZE_WORDS];
            } frame[MAX_BITS];
        };
    };
    uint8_t brightness; /* 0 or 1; controls global brighntess control */
};

/* Efficiently-packed UART data format */
struct data_format {
    union {
        uint8_t high[8];
        struct { uint8_t ah, bh, ch, dh, eh, fh, gh, dph; };
    };
    union {
        uint16_t low;
        struct { uint8_t dpl:2, gl:2, fl:2, el:2, dl:2, cl:2, bl:2, al:2; };
    };
} __attribute__((packed));


void transpose_data(volatile uint8_t *rx_buf, volatile struct framebuf *out_fb);
void untranspose_data(struct framebuf *fb, uint8_t *txbuf);

#endif//__TRANSPOSE_H__
