
#include <unistd.h>
#include <string.h>

#include "transpose.h"

/* This file contains conversion routines that pre-format the brightness data
 * received from the UART such that the interrupt service routines only need to
 * push it out the SPI without further computation, making these ISRs nice and
 * tight.
 * 
 * To understand this code note the multiplexing scheme used on the board. The
 * circuit contains two MBI5026 shift-register LED drivers of 16 channels each
 * cascaded. Effectively this behaves like a 32-channel LED driver fed data
 * serially. Each output is connected to a single digit's COM pin. All digit's
 * segment anode pins are connected together in a large bus fed by one of the
 * two auxiliary shift registers.
 *
 * The firmware is selecting each segment in turn with a full BCM cycle for each
 * segment before the next one is selected.
 */

/* This array maps the 32 adressable digits on a board to the 32 bits shifted
 * out to the LED drivers. */
uint8_t digit_map[33] = {
     0, 1, 2, 3,       28,29,30,31,
      4, 5, 6, 7,     24,25,26,27,
       8, 9,10,11,   20,21,22,23,
       12,13,14,15, 16,17,18,19
};

/* This function produces a 10-bit output buffer ready for the modulation ISRs
 * from 10-bit input data encoded for the UART. For the precise data format, see
 * transpose.h.
 *
 * On the UART side we have digits in the order defined in digit_map, 10 byte
 * per digit. The first 8 bytes are the 8 LSBs of each segments brightness value
 * in the order [A, B, C, D, E, F, G, DECIMAL_POINT]. The two MSBs to make each
 * value 10-bit are bit-packed into the remaining two bytes in big-endian byte
 * order starting from DP.
 *
 * On the display frame buffer side, data is stored in multiplexing order:
 * first digits, then time/bits and finally segments. So for each segment you
 * have a large buffer containing all the bit periods and digits, and for each
 * bit period you have 32 bits for all 32 digits.
 */
void transpose_data(volatile uint8_t *rx_buf, volatile struct framebuf *out_fb)
{
    /* FIXME this can probably be removed. */
    memset((uint8_t *)out_fb, 0, sizeof(*out_fb));

    /* 8 MSB loop */
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

    /* 2 packed LSB loop */
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

    /* Global analog brightness value */
    out_fb->brightness = ((volatile struct framebuf *)rx_buf)->brightness;
}

/* This function was used for testing transpose_data. It does precisely the
 * reverse operation. */
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

