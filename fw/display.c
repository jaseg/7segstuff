/* Megumin LED display firmware
 * Copyright (C) 2018 Sebastian Götte <code@jaseg.net>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "led.h"
#include "display.h"

volatile int frame_duration_us = 0;
volatile int nbits = MAX_BITS;

/* Modulation data */
volatile enum FB_OPERATION fb_op;
static volatile struct framebuf fb[2] = {0};
volatile struct framebuf *read_fb=fb+0, *write_fb=fb+1;

/* Auxiliary shift register values */
#define LED_COMM     0x0001
#define LED_ERROR    0x0002
#define LED_ID       0x0004
#define SR_ILED_HIGH 0x0080
#define SR_ILED_LOW  0x0040

/* This is a lookup table mapping segments to present a standard segment order on the UART interface. This is converted
 * into an internal representation once on startup in main(). The data type must be at least uint16. */
static uint32_t segment_map[8] = {5, 7, 6, 4, 1, 3, 0, 2};

static unsigned int active_bit = 0;
static int active_segment = 0;

/* Bit timing base value. This is the lowes bit interval used in TIM1/TIM3 timer counts. */
#define PERIOD_BASE 4

/* This value is a constant offset added to every bit period to allow for the timer IRQ handler to execute. This is set
 * empirically using a debugger and a logic analyzer.
 *
 * This value is in TIM1/TIM3 timer counts. */
#define TIMER_CYCLES_FOR_SPI_TRANSMISSIONS 9

/* This value sets the point when the LED strobe is asserted after the begin of the current bit cycle and IRQ
 * processing. This must be less than TIMER_CYCLES_FOR_SPI_TRANSMISSIONS but must be large enough to allow for the SPI
 * transmission to reliably finish.
 *
 * This value is in TIM1/TIM3 timer counts. */
#define TIMER_CYCLES_BEFORE_LED_STROBE 8

/* This value sets how long the TIM1 CC IRQ used for AUX register setting etc. is triggered before the end of the
 * longest cycle. This value should not be larger than PERIOD_BASE<<MIN_BITS to make sure the TIM1 CC IRQ does only
 * trigger in the longest cycle no matter what nbits is set to.
 *
 * This value is in TIM1/TIM3 timer counts. */
#define AUX_SPI_PRETRIGGER 64 /* trigger with about 24us margin to the end of cycle/next TIM3 IRQ */

/* This value sets how long a batch of ADC conversions used for temperature measurement is started before the end of the
 * longest cycle. Here too the above caveats apply.
 *
 * This value is in TIM1/TIM3 timer counts. */
#define ADC_PRETRIGGER 150 /* trigger with about 12us margin to TIM1 CC IRQ */

/* Defines for brevity */
#define A TIMER_CYCLES_FOR_SPI_TRANSMISSIONS
#define B PERIOD_BASE

/* This is a constant offset containing some empirically determined correction values */
#define C (0)

/* This lookup table maps bit positions to timer period values. This is a lookup table to allow for the compensation for
 * non-linear effects of ringing at lower bit durations.
 */
static uint16_t timer_period_lookup[MAX_BITS+1] = {
    /* LSB here */
    A - C + (B<< 0),
    A - C + (B<< 1),
    A - C + (B<< 2),
    A - C + (B<< 3),
    A - C + (B<< 4),
    A - C + (B<< 5),
    A - C + (B<< 6),
    A - C + (B<< 7),
    A - C + (B<< 8),
    A - C + (B<< 9),
    A - C + (B<< 0),
    /* MSB here */
};

/* Don't pollute the global namespace */
#undef A
#undef B
#undef C

void display_cfg_timers(void);
void display_cfg_spi(void);

void display_init() {
    display_cfg_spi();

    /* Pre-compute aux register values for timer ISR */
    for (int i=0; i<NSEGMENTS; i++) {
        segment_map[i] = 0xff00 ^ (0x100<<segment_map[i]);
    }

    /* Clear frame buffer */
    read_fb->brightness = 1;
    for (int i=0; i<sizeof(read_fb->data)/sizeof(uint32_t); i++) {
        read_fb->data[i] = 0xffffffff; /* FIXME this is a debug value. Should be 0x00000000; */
    }

    display_cfg_timers();
}

void display_cfg_spi() {
    /* Configure SPI controller */
    SPI1->I2SCFGR = 0;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 |= LL_SPI_DATAWIDTH_16BIT;

    /* Baud rate PCLK/4 -> 12.5MHz */
    SPI1->CR1 =
          SPI_CR1_BIDIMODE
        | SPI_CR1_BIDIOE
        | SPI_CR1_SSM
        | SPI_CR1_SSI
        | SPI_CR1_SPE
        | (1<<SPI_CR1_BR_Pos)
        | SPI_CR1_MSTR
        | SPI_CR1_CPOL
        | SPI_CR1_CPHA;
    /* FIXME maybe try w/o BIDI */
}

void display_cfg_timers() {
    /* Ok, so this part is unfortunately a bit involved.
     *
     * Because the GPIO alternate function assignments worked out that way, the LED driving logic uses timers 1 and 3.
     * Timer 1 is synchronized to timer 3. When timer 3 overflows, timer 1 is reset. Both use the same prescaler so both
     * are synchronous possibly modulo some propagation delay in the synchronization hardware.
     *
     * Timer 3:
     *  * The IRQ handler is set to trigger on overflow and
     *    * triggers the SPI transmissions to the LED drivers and
     *    * updates the timing logic with the delays for the next cycle
     *  * Compare unit 1 generates the !OE signal for the led drivers
     * Timer 1:
     *  * Compare unit 1 triggers the interrupt handler only in the longest bit cycle. The IRQ handler
     *    * transmits the data to the auxiliary shift registers and
     *    * swaps the frame buffers if pending
     *  * Compare unit 2 generates the led drivers' STROBE signal
     * 
     * The AUX_STROBE signal for the two auxiliary shift registers that deal with segment selection, current setting and
     * status leds is generated in software in both ISRs. TIM3's ISR indiscriminately resets this strobe every bit
     * cycle, and TIM1's ISR sets it every NBITSth bit cycle.
     *
     * The reason both timers' IRQ handlers are used is that this way no big if/else statement is necessary to
     * distinguish between both cases. Timer 1's IRQ handler is set via CC2 to trigger a few cycles earlier than the end
     * of the longest bit cycle. This means that if both timers perform bit cycles of length 1, 2, 4, 8, 16 and 32
     * TIM1_CC2 will be set to trigger at count e.g. 28. This means it is only triggered once in the last timer cycle.
     */

    TIM3->CR2   = (2<<TIM_CR2_MMS_Pos); /* master mode: update */
    TIM3->CCMR1 = (6<<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; /* PWM Mode 1, enable CCR preload */
    TIM3->CCER  = TIM_CCER_CC1E;
    TIM3->CCR1  = TIMER_CYCLES_FOR_SPI_TRANSMISSIONS;
    TIM3->DIER  = TIM_DIER_UIE;
    TIM3->PSC   = SystemCoreClock/5000000 * 2 - 1; /* 0.20us/tick */
    TIM3->ARR   = 0xffff;
    TIM3->EGR  |= TIM_EGR_UG;
    TIM3->CR1   = TIM_CR1_ARPE;
    TIM3->CR1  |= TIM_CR1_CEN;

    /* Slave TIM1 to TIM3. */
    TIM1->PSC   = TIM3->PSC;
    TIM1->SMCR  = (2<<TIM_SMCR_TS_Pos) | (4<<TIM_SMCR_SMS_Pos); /* Internal Trigger 2 (ITR2) -> TIM3; slave mode: reset */

    /* Setup CC1 and CC2. CC2 generates the LED drivers' STROBE, CC1 triggers the IRQ handler */
    TIM1->BDTR  = TIM_BDTR_MOE;
    TIM1->CCMR1 = (6<<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; /* PWM Mode 1, enable CCR preload for AUX_STROBE */
    TIM1->CCMR2 = (6<<TIM_CCMR2_OC4M_Pos); /* PWM Mode 1 */
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E;
    TIM1->CCR2  = TIMER_CYCLES_BEFORE_LED_STROBE;
    /* Trigger at the end of the longest bit cycle. This means this does not trigger in shorter bit cycles. */
    TIM1->CCR1  = timer_period_lookup[nbits-1] - AUX_SPI_PRETRIGGER;
    TIM1->CCR4  = timer_period_lookup[nbits-1] - ADC_PRETRIGGER;
    TIM1->DIER  = TIM_DIER_CC1IE;

    TIM1->ARR   = 0xffff; /* This is as large as possible since TIM1 is reset by TIM3. */
    /* Preload all values */
    TIM1->EGR  |= TIM_EGR_UG;
    TIM1->CR1   = TIM_CR1_ARPE;
    /* And... go! */
    TIM1->CR1  |= TIM_CR1_CEN;

    /* Sends aux data and swaps frame buffers if necessary */
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    NVIC_SetPriority(TIM1_CC_IRQn, 0);
    /* Sends LED data and sets up the next bit cycle's timings */
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0);
}

void TIM1_CC_IRQHandler() {
    //static int last_frame_time = 0;
    /* This handler takes about 1.5us */
    GPIOA->BSRR = GPIO_BSRR_BS_0; // Debug

    /* Set SPI baudrate to 12.5MBd for slow-ish 74HC(T)595. This is reset again in TIM3's IRQ handler.*/
    SPI1->CR1 |= (2<<SPI_CR1_BR_Pos);

    /* Advance bit counts and perform pending frame buffer swap */
    active_bit = 0;
    active_segment++;
    if (active_segment == NSEGMENTS) {
        active_segment = 0;

        /* Frame buffer swap */
        if (fb_op == FB_UPDATE) {
            volatile struct framebuf *tmp = read_fb;
            read_fb = write_fb;
            write_fb = tmp;
            fb_op = FB_WRITE;
        }
    }

    /* Reset aux strobe */
    GPIOA->BSRR = GPIO_BSRR_BR_10;
    /* Send AUX register data */
    uint32_t aux_reg = (read_fb->brightness ? SR_ILED_HIGH : SR_ILED_LOW) | (led_state<<1);
    SPI1->DR = aux_reg | segment_map[active_segment];

    /* TODO: Measure frame rate for status report */

    /* Clear interrupt flag */
    TIM1->SR &= ~TIM_SR_CC1IF_Msk;

    GPIOA->BSRR = GPIO_BSRR_BR_0; // Debug
}

void TIM3_IRQHandler() {
    /* This handler takes about 2.1us */
    GPIOA->BSRR = GPIO_BSRR_BS_0; // Debug

    /* Reset SPI baudrate to 25MBd for fast MBI5026. Every couple of cycles, TIM1's ISR will set this to a slower value
     * for the slower AUX registers.*/
    SPI1->CR1 &= ~SPI_CR1_BR_Msk;
    /* Assert aux strobe reset by TIM1's IRQ handler */
    GPIOA->BSRR = GPIO_BSRR_BS_10;

    /* Queue LED driver data into SPI peripheral */
    uint32_t spi_word = read_fb->data[active_bit*FRAME_SIZE_WORDS + active_segment];
    SPI1->DR = spi_word>>16;
    spi_word &= 0xFFFF;
    /* Note that this only waits until the internal FIFO is ready, not until all data has been sent. */
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = spi_word;

    /* Advance bit. This will overflow, but that is OK since before the next invocation of this ISR, the other ISR will
     * reset it. */
    active_bit++;
    /* Schedule next bit cycle */
    TIM3->ARR = timer_period_lookup[active_bit];

    /* Clear interrupt flag */
    TIM3->SR &= ~TIM_SR_UIF_Msk;

    GPIOA->BSRR = GPIO_BSRR_BR_0; // Debug
}

