
#include <unistd.h>
#include <string.h>

#include "transpose.h"

uint8_t digit_map[33] = {
     0, 1, 2, 3,       28,29,30,31,
      4, 5, 6, 7,     24,25,26,27,
       8, 9,10,11,   20,21,22,23,
       12,13,14,15, 16,17,18,19
};
void transpose_data(volatile uint8_t *rx_buf, volatile struct framebuf *out_fb) {
    memset((uint8_t *)out_fb, 0, sizeof(*out_fb));
    struct data_format *rxp = (struct data_format *)rx_buf;
    for (int bit=0; bit<8; bit++) { /* bits */
        uint32_t bit_mask = 1U<<bit;
        volatile uint32_t *frame_data = out_fb->frame[bit+2].data;
        uint8_t *start_inp = rxp->high;
        for (volatile uint32_t *outp=frame_data; outp<frame_data+8; outp++) { /* segments */
            uint32_t acc = 0;
            uint8_t *inp = start_inp++;
            for (int digit=0; digit<32; digit++) {
                acc |= (*inp & bit_mask) >> bit << digit_map[digit];
                inp += sizeof(struct data_format);
            }
            *outp = acc;
        }
    }
    for (int bit=0; bit<2; bit++) { /* bits */
        volatile uint32_t *frame_data = out_fb->frame[bit].data;
        for (int seg=0; seg<8; seg++) { /* segments */
            uint16_t *inp = &rxp->low;
            uint32_t mask = 1 << bit << (seg*2);
            uint32_t acc = 0;
            for (int digit=0; digit<32; digit++) {
                acc |= (*inp & mask) >> bit >> (seg*2) << digit_map[digit];
                inp += sizeof(struct data_format)/sizeof(uint16_t);
            }
            frame_data[seg] = acc;
        }
    }
    out_fb->brightness = ((volatile struct framebuf *)rx_buf)->brightness;
}


void untranspose_data(struct framebuf *fb, uint8_t *txbuf) {
    memset(txbuf, 0, sizeof(*fb));

    struct data_format *tx = (struct data_format *) txbuf;

    for (size_t i=0; i<32; i++) { /* digit */
        for (size_t j=0; j<8; j++) { /* segment */
            for (size_t k=0; k<8; k++) { /* bit */
                tx[i].high[j] |= (fb->frame[k+2].data[j] & (1<<i)) ? (1<<k) : 0;
            }
        }
    }

    for (size_t i=0; i<32; i++) { /* digit */
        for (size_t j=0; j<8; j++) { /* segment */
            for (size_t k=0; k<2; k++) { /* bit */
                tx[i].low |= (fb->frame[k].data[j] & (1<<i)) ? (1<<k<<(j*2)) : 0;
            }
        }
    }
}

