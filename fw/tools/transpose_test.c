
#include <stdio.h>
#include <stdint.h>
#include <malloc.h>
#include <string.h>
#include <assert.h>

#include "transpose.h"

void rxbuf_dump(struct data_format *rx) {
    fprintf(stdout, "\033[38;5;244m          high          |       low\n");
    fprintf(stdout, " a  b  c  d  e  f  g  . | a b c d e f g .\033[0m\n");

    for (int i=0; i<32; i++) {
        for (int j=0; j<8; j++)
            fprintf(stdout, "%02x ", rx[i].high[j]);
        fprintf(stdout, "\033[38;5;244m|\033[0m");
        for (int j=0; j<8; j++)
            fprintf(stdout, " %1x", (rx[i].low>>(2*j))&0x3);
        fprintf(stdout, "\n");
    }
}

void fb_dump(struct framebuf *fb) {
    fprintf(stdout, "\033[38;5;244mbit | ");
    for (int i=0; i<8; i++)
        fprintf(stdout, "       %d ", i);
    fprintf(stdout, "\033[0m\n");

    for (int i=0; i<10; i++) {
        fprintf(stdout, " %2d |", i);
        for (int j=0; j<8; j++)
           fprintf(stdout, " %08x", fb->frame[i].data[j]);
        fprintf(stdout, "\n");
    }
}

int main(int argc, char **argv) {
    FILE *randf = fopen("/dev/urandom", "r");
    if (!randf)
        return 2;

    struct framebuf *fb = malloc(sizeof(struct framebuf));
    if (!fb)
        return 2;
    assert(sizeof(*fb) == 32*sizeof(struct data_format)+4);

    uint8_t *rxbuf1 = malloc(sizeof(*fb));
    if (!rxbuf1)
        return 2;
    if (fread(rxbuf1, 1, sizeof(*fb), randf) != sizeof(*fb))
        return 2;

    uint8_t *rxbuf2 = malloc(sizeof(*fb));
    if (!rxbuf2)
        return 2;
    memcpy(rxbuf2, rxbuf1, sizeof(*fb));

    memset((void *)fb, 0, sizeof(*fb));

    transpose_data(rxbuf1, fb);
    untranspose_data(fb, rxbuf1);

    assert(!memcmp(rxbuf1, rxbuf2, sizeof(fb)));
    
    fprintf(stdout, "\n\033[93mDUMP of ORIGINAL\033[0m\n");
    rxbuf_dump((struct data_format *)rxbuf2);

    fprintf(stdout, "\n\033[93mDUMP of FRAME BUFFER\033[0m\n");
    fb_dump(fb);

    fprintf(stdout, "\n\033[93mDUMP of RESULT\033[0m\n");
    rxbuf_dump((struct data_format *)rxbuf1);

    fprintf(stderr, "PASS");
    return 0;
}
