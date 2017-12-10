

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#include <stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <stm32f0xx_ll_spi.h>
#pragma GCC diagnostic pop

#include <system_stm32f0xx.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include "transpose.h"

/* 
 * Part number: STM32F030F4C6
 */

typedef struct
{
  volatile uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  Control Register */
  volatile uint32_t CYCCNT;                 /*!< Offset: 0x004 (R/W)  Cycle Count Register */
  volatile uint32_t CPICNT;                 /*!< Offset: 0x008 (R/W)  CPI Count Register */
  volatile uint32_t EXCCNT;                 /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
  volatile uint32_t SLEEPCNT;               /*!< Offset: 0x010 (R/W)  Sleep Count Register */
  volatile uint32_t LSUCNT;                 /*!< Offset: 0x014 (R/W)  LSU Count Register */
  volatile uint32_t FOLDCNT;                /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
  volatile uint32_t PCSR;                   /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
  volatile uint32_t COMP0;                  /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
  volatile uint32_t MASK0;                  /*!< Offset: 0x024 (R/W)  Mask Register 0 */
  volatile uint32_t FUNCTION0;              /*!< Offset: 0x028 (R/W)  Function Register 0 */
           uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                  /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
  volatile uint32_t MASK1;                  /*!< Offset: 0x034 (R/W)  Mask Register 1 */
  volatile uint32_t FUNCTION1;              /*!< Offset: 0x038 (R/W)  Function Register 1 */
           uint32_t RESERVED1[1];
} DWT_Type;

#define DWT ((DWT_Type *)0xE0001000)
DWT_Type *dwt = DWT;

void dwt0_configure(volatile void *addr) {
    dwt->COMP0 = (uint32_t)addr;
    dwt->MASK0 = 0;
}

enum DWT_Function {
    DWT_R = 5,
    DWT_W = 6,
    DWT_RW = 7
};

void dwt0_enable(enum DWT_Function function) {
    dwt->FUNCTION0 = function;
}

/* Wait for about 0.2us */
static void tick(void) {
                    /* 1 */         /* 2 */         /* 3 */         /* 4 */         /* 5 */
    /*  5 */ __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop");
    /* 10 */ __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop");
}

void spi_send(int data) {
    SPI1->DR = data;
    while (SPI1->SR & SPI_SR_BSY);
}

void strobe_aux(void) {
    GPIOA->BSRR = GPIO_BSRR_BS_10;
    tick();
    GPIOA->BSRR = GPIO_BSRR_BR_10;
}

void strobe_leds(void) {
    GPIOA->BSRR = GPIO_BSRR_BS_9;
    tick();
    GPIOA->BSRR = GPIO_BSRR_BR_9;
}

#define FIRMWARE_VERSION 1
#define HARDWARE_VERSION 1

#define TS_CAL1 (*(uint16_t *)0x1FFFF7B8)
#define VREFINT_CAL (*(uint16_t *)0x1FFFF7BA)

volatile  int16_t adc_vcc_mv = 0;
volatile  int16_t adc_temp_celsius = 0;

volatile uint16_t adc_buf[2];

volatile unsigned int sys_time = 0;
volatile unsigned int sys_time_seconds = 0;

volatile struct framebuf fb[2] = {0};
volatile struct framebuf *read_fb=fb+0, *write_fb=fb+1;
volatile int led_state = 0;
volatile enum { FB_WRITE, FB_FORMAT, FB_UPDATE } fb_op;
volatile union {
    struct __attribute__((packed)) { struct framebuf fb; uint8_t end[0]; } set_fb_rq;
    struct __attribute__((packed)) { uint8_t nbits;      uint8_t end[0]; } set_nbits_rq;
    uint8_t byte_data[0];
} rx_buf;

volatile union {
    struct { uint32_t magic;    } ping_reply;
    struct __attribute__((packed)) {
        uint8_t  firmware_version,
                 hardware_version,
                 digit_rows,
                 digit_cols;
        uint32_t uptime;
        uint32_t millifps;
         int16_t vcc_mv,
                 temp_tenth_celsius;
        uint8_t  nbits;
    } desc_reply;
} tx_buf;

extern uint8_t bus_addr;

#define LED_COMM     0x0001
#define LED_ERROR    0x0002
#define LED_ID       0x0004
#define SR_ILED_HIGH 0x0080
#define SR_ILED_LOW  0x0040

unsigned int stk_start(void) {
    return SysTick->VAL;
}

unsigned int stk_end(unsigned int start) {
    return (start - SysTick->VAL) & 0xffffff;
}

unsigned int stk_microseconds(void) {
    return sys_time*1000 + (1000 - (SysTick->VAL / (SystemCoreClock/1000000)));
}

void cfg_spi1() {
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

/* This is a lookup table mapping segments to present a standard segment order on the UART interface. This is converted
 * into an internal representation once on startup in main(). The data type must be at least uint16. */
uint32_t segment_map[8] = {5, 7, 6, 4, 1, 3, 0, 2};

/* The value to be written into the aux register. This encompasses LED state as well as the current setting bits. */
static volatile uint32_t aux_reg = 0;
static volatile int frame_duration_us;
volatile int nbits = MAX_BITS;

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

void cfg_timers_led() {
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
    /* This handler takes about 1.5us */
    GPIOA->BSRR = GPIO_BSRR_BS_0; // Debug

    /* Set SPI baudrate to 12.5MBd for slow-ish 74HC(T)595. This is reset again in TIM3's IRQ handler.*/
    SPI1->CR1 |= (2<<SPI_CR1_BR_Pos);

    /* Advance bit counts and perform pending frame buffer swap */
    active_bit = 0;
    active_segment++;
    if (active_segment == NSEGMENTS) {
        active_segment = 0;

        /* FIXME remove this?
        int time = stk_microseconds();
        frame_duration_us = time - last_frame_time;
        last_frame_time = time;
        */
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
    SPI1->DR = aux_reg | segment_map[active_segment];

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

enum Command {
    CMD_PING,
    CMD_SET_FB,
    CMD_SET_NBITS,
    CMD_GET_DESC,
    N_CMDS
};

void uart_config(void) {
    USART1->CR1 = /* 8-bit -> M1, M0 clear */
        /* RTOIE clear */
          (8 << USART_CR1_DEAT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        | (8 << USART_CR1_DEDT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        /* OVER8 clear. Use default 16x oversampling */
        /* CMIF clear */
        | USART_CR1_MME
        /* WAKE clear */
        /* PCE, PS clear */
        | USART_CR1_RXNEIE /* Enable receive interrupt */
        /* other interrupts clear */
        | USART_CR1_TE
        | USART_CR1_RE;
    //USART1->CR2 = USART_CR2_RTOEN; /* Timeout enable */
    USART1->CR3 = USART_CR3_DEM; /* RS485 DE enable (output on RTS) */
    /* Set divider for 25MHz baud rate @50MHz system clock. */
    int usartdiv = 25;
    USART1->BRR = usartdiv;

    /* And... go! */
    USART1->CR1 |= USART_CR1_UE;

    /* Enable receive interrupt */
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 1);
}

/* Error counters for debugging */
static unsigned int overruns = 0;
static unsigned int frame_overruns = 0;
static unsigned int invalid = 0;

/* This is the higher-level protocol handler for the serial protocol. It gets passed the number of data bytes in this
 * frame (which may be zero) and returns a pointer to the buffer where the next frame should be stored.
 */
volatile uint8_t *packet_received(int len) {
    static int protocol_state = 0; 
    /* Use zero-length frames as delimiters to synchronize this protocol layer */
    if (len == 0) {
        protocol_state = 0;

    } else if (len == sizeof(rx_buf.set_fb_rq)/2) {
        if (protocol_state == 0) { /* First of two half-framebuffer data frames */
            protocol_state = 1;
            /* Return second half of receive buffer */
            return rx_buf.byte_data + (sizeof(rx_buf.set_fb_rq)/2);

        } else if (protocol_state == 1) { /* Second of two half-framebuffer data frames */
            /* Kick off buffer transfer. This triggers the main loop to copy data out of the receive buffer and paste it
             * properly formatted into the frame buffer. */
            if (fb_op == FB_WRITE) {
                fb_op = FB_FORMAT;
            } else {
                /* FIXME An overrun happend. What should we do? */
                frame_overruns++;
            }

            /* Go to "hang mode" until next zero-length packet. */
            protocol_state = 2;
        }

    } else {
        /* FIXME An invalid packet has been received. What should we do? */
        invalid++;
        protocol_state = 2; /* go into "hang mode" until next zero-length packet */
    }

    /* By default, return rx_buf.byte_data . This means if an invalid protocol state is reached ("hang mode"), the next
     * frame is still written to rx_buf. This is not a problem since whatever garbage is written at that point will be
     * overwritten before the next buffer transfer. */
    return rx_buf.byte_data;
}

void USART1_IRQHandler(void) {
    /* Since a large amount of data will be shoved down this UART interface we need a more reliable and more efficient
     * way of framing than just waiting between transmissions.
     *
     * This code uses "Consistent Overhead Byte Stuffing" (COBS). For details, see its Wikipedia page[0] or the proper
     * scientific paper[1] published on it. Roughly, it works like this:
     *
     * * A frame is at most 254 bytes in length.
     * * The null byte 0x00 acts as a frame delimiter. There is no null bytes inside frames.
     * * Every frame starts with an "overhead" byte indicating the number of non-null payload bytes until the next null
     *   byte in the payload, **plus one**. This means this byte can never be zero.
     * * Every null byte in the payload is replaced by *its* distance to *its* next null byte as above.
     *
     * This means, at any point the receiver can efficiently be synchronized on the next frame boundary by simply
     * waiting for a null byte. After that, only a simple state machine is necessary to strip the overhead byte and a
     * counter to then count skip intervals.
     *
     * Here is Wikipedia's table of example values:
     *
     *    Unencoded data          Encoded with COBS
     *    00                      01 01 00
     *    00 00                   01 01 01 00
     *    11 22 00 33             03 11 22 02 33 00
     *    11 22 33 44             05 11 22 33 44 00
     *    11 00 00 00             02 11 01 01 01 00
     *    01 02 ...FE             FF 01 02 ...FE 00
     *
     * [0] https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
     * [1] Cheshire, Stuart; Baker, Mary (1999). "Consistent Overhead Byte Stuffing"
     *     IEEE/ACM Transactions on Networking. doi:10.1109/90.769765
     *     http://www.stuartcheshire.org/papers/COBSforToN.pdf
     */

    /* This pointer stores where we write data. The higher-level protocol logic decides on a frame-by-frame-basis where
     * the next frame's data will be stored. */
    static volatile uint8_t *writep = rx_buf.byte_data;
    /* Index inside the current frame payload */
    static int rxpos = 0;
    /* COBS state machine. This implementation might be a little too complicated, but it works well enough and I find it
     * reasonably easy to understand. */
    static enum {
        COBS_WAIT_SYNC = 0,  /* Synchronize with frame */
        COBS_WAIT_START = 1, /* Await overhead byte */
        COBS_RUNNING = 2     /* Process payload */
    } cobs_state = 0;
    /* COBS skip counter. During payload processing this contains the remaining non-null payload bytes */
    static int cobs_count = 0;

    if (USART1->ISR & USART_ISR_ORE) { /* Overrun handling */
        overruns++;
        /* Reset and re-synchronize. Retry next frame. */
        rxpos = 0;
        cobs_state = COBS_WAIT_SYNC;
        /* Clear interrupt flag */
        USART1->ICR = USART_ICR_ORECF;

    } else { /* Data received */
        uint8_t data = USART1->RDR; /* This automatically acknowledges the IRQ */

        if (data == 0x00) { /* End-of-packet */
            /* Process higher protocol layers on this packet. */
            writep = packet_received(rxpos);

            /* Reset for next packet. */
            cobs_state = COBS_WAIT_START;
            rxpos = 0;

        } else { /* non-null byte */
            if (cobs_state == COBS_WAIT_SYNC) { /* Wait for null byte */
                /* ignore data */

            } else if (cobs_state == COBS_WAIT_START) { /* Overhead byte */
                cobs_count = data;
                cobs_state = COBS_RUNNING;

            } else { /* Payload byte */
                if (--cobs_count == 0) { /* Skip byte */
                    cobs_count = data;
                    data = 0;
                }

                /* Write processed payload byte to current receive buffer */
                writep[rxpos++] = data;
            }
        }
    }
}

#define ADC_OVERSAMPLING 8
uint32_t vsense;
void DMA1_Channel1_IRQHandler(void) {
    /* This interrupt takes either 1.2us or 13us. It can be pre-empted by the more timing-critical UART and LED timer
     * interrupts. */
    GPIOA->BSRR = GPIO_BSRR_BS_4; // Debug
    static int count = 0; /* oversampling accumulator sample count */
    static uint32_t adc_aggregate[2] = {0, 0}; /* oversampling accumulator */

    /* Clear the interrupt flag */
    DMA1->IFCR |= DMA_IFCR_CGIF1;

    adc_aggregate[0] += adc_buf[0];
    adc_aggregate[1] += adc_buf[1];

    if (++count == (1<<ADC_OVERSAMPLING)) {
        /* This has been copied from the code examples to section 12.9 ADC>"Temperature sensor and internal reference
         * voltage" in the reference manual with the extension that we actually measure the supply voltage instead of
         * hardcoding it. This is not strictly necessary since we're running off a bored little LDO but it's free and
         * the current supply voltage is a nice health value.
         */
        adc_vcc_mv = (3300 * VREFINT_CAL)/(adc_aggregate[0]>>ADC_OVERSAMPLING);
        int32_t temperature = (((uint32_t)TS_CAL1) - ((adc_aggregate[1]>>ADC_OVERSAMPLING) * adc_vcc_mv / 3300)) * 1000;
        temperature = (temperature/5336) + 30;
        adc_temp_celsius = temperature;

        count = 0;
        adc_aggregate[0] = 0;
        adc_aggregate[1] = 0;
    }
    GPIOA->BSRR = GPIO_BSRR_BR_4; // Debug
}

void adc_config(void) {
    /* The ADC is used for temperature measurement. To compute the temperature from an ADC reading of the internal
     * temperature sensor, the supply voltage must also be measured. Thus we are using two channels.
     *
     * The ADC is triggered by compare channel 4 of timer 1. The trigger is set to falling edge to trigger on compare
     * match, not overflow.
     */
    ADC1->CFGR1 = ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG | (2<<ADC_CFGR1_EXTEN_Pos) | (1<<ADC_CFGR1_EXTSEL_Pos);
    /* Clock from PCLK/4 instead of the internal exclusive high-speed RC oscillator. */
    ADC1->CFGR2 = (2<<ADC_CFGR2_CKMODE_Pos);
    /* Use the slowest available sample rate */
    ADC1->SMPR  = (7<<ADC_SMPR_SMP_Pos);
    /* Internal VCC and temperature sensor channels */
    ADC1->CHSELR = ADC_CHSELR_CHSEL16 | ADC_CHSELR_CHSEL17;
    /* Enable internal voltage reference and temperature sensor */
    ADC->CCR = ADC_CCR_TSEN | ADC_CCR_VREFEN;
    /* Perform ADC calibration */
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL)
        ;
    /* Enable ADC */
    ADC1->CR |= ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADSTART;

    /* Configure DMA 1 Channel 1 to get rid of all the data */
    DMA1_Channel1->CPAR = (unsigned int)&ADC1->DR;
    DMA1_Channel1->CMAR = (unsigned int)&adc_buf;
    DMA1_Channel1->CNDTR = sizeof(adc_buf)/sizeof(adc_buf[0]);
    DMA1_Channel1->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel1->CCR |=
          DMA_CCR_CIRC /* circular mode so we can leave it running indefinitely */
        | (1<<DMA_CCR_MSIZE_Pos) /* 16 bit */
        | (1<<DMA_CCR_PSIZE_Pos) /* 16 bit */
        | DMA_CCR_MINC
        | DMA_CCR_TCIE; /* Enable transfer complete interrupt. */
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* Enable channel */

    /* triggered on transfer completion. We use this to process the ADC data */
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
}

/*
    tx_buf.desc_reply.firmware_version = FIRMWARE_VERSION;
    tx_buf.desc_reply.hardware_version = HARDWARE_VERSION;
    tx_buf.desc_reply.digit_rows = NROWS;
    tx_buf.desc_reply.digit_cols = NCOLS;
    tx_buf.desc_reply.uptime = sys_time_seconds;
    tx_buf.desc_reply.vcc_mv = adc_vcc_mv;
    tx_buf.desc_reply.temp_tenth_celsius = adc_temp_tenth_celsius;
    tx_buf.desc_reply.nbits = nbits;
    tx_buf.desc_reply.millifps = frame_duration_us > 0 ? 1000000000 / frame_duration_us : 0;
*/

int main(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk & ~RCC_CFGR_SW_Msk & ~RCC_CFGR_PPRE_Msk & ~RCC_CFGR_HPRE_Msk;
    RCC->CFGR |= (2<<RCC_CFGR_PLLMUL_Pos) | RCC_CFGR_PLLSRC_HSE_PREDIV; /* PLL x4 -> 50.0MHz */
    RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_Msk;
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV2; /* prediv :2 -> 12.5MHz */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    SystemCoreClockUpdate();

    /* Turn on lots of neat things */
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_DMAEN | RCC_AHBENR_CRCEN | RCC_AHBENR_FLITFEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADCEN | RCC_APB2ENR_DBGMCUEN | RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOA->MODER |=
          (1<<GPIO_MODER_MODER0_Pos)  /* PA0  - Debug */
        | (2<<GPIO_MODER_MODER1_Pos)  /* PA1  - RS485 DE */
        | (2<<GPIO_MODER_MODER2_Pos)  /* PA2  - RS485 TX */
        | (2<<GPIO_MODER_MODER3_Pos)  /* PA3  - RS485 RX */
        | (1<<GPIO_MODER_MODER4_Pos)  /* PA4  - Debug */
        | (2<<GPIO_MODER_MODER5_Pos)  /* PA5  - SCLK */
        | (2<<GPIO_MODER_MODER6_Pos)  /* PA6  - LED !OE */
        | (2<<GPIO_MODER_MODER7_Pos)  /* PA7  - MOSI */
        | (2<<GPIO_MODER_MODER9_Pos)  /* PA9  - LED strobe */
        | (1<<GPIO_MODER_MODER10_Pos);/* PA10 - Auxiliary strobe */

    /* Set shift register IO GPIO output speed */
    GPIOA->OSPEEDR |=
          (2<<GPIO_OSPEEDR_OSPEEDR0_Pos)   /* Debug */
        | (2<<GPIO_OSPEEDR_OSPEEDR1_Pos)   /* RS485 DE */
        | (2<<GPIO_OSPEEDR_OSPEEDR2_Pos)   /* TX */
        | (2<<GPIO_OSPEEDR_OSPEEDR3_Pos)   /* RX */
        | (2<<GPIO_OSPEEDR_OSPEEDR4_Pos)   /* Debug */
        | (2<<GPIO_OSPEEDR_OSPEEDR5_Pos)   /* SCLK */
        | (2<<GPIO_OSPEEDR_OSPEEDR6_Pos)   /* LED !OE   */
        | (2<<GPIO_OSPEEDR_OSPEEDR7_Pos)   /* MOSI */
        | (2<<GPIO_OSPEEDR_OSPEEDR9_Pos)   /* LED strobe */
        | (2<<GPIO_OSPEEDR_OSPEEDR10_Pos); /* Auxiliary strobe */

    GPIOA->AFR[0] |=
          (1<<GPIO_AFRL_AFRL1_Pos)   /* USART1_RTS (DE) */
        | (1<<GPIO_AFRL_AFRL2_Pos)   /* USART1_TX */
        | (1<<GPIO_AFRL_AFRL3_Pos)   /* USART1_RX */
        | (0<<GPIO_AFRL_AFRL5_Pos)   /* SPI1_SCK */
        | (1<<GPIO_AFRL_AFRL6_Pos)   /* TIM3_CH1 */
        | (0<<GPIO_AFRL_AFRL7_Pos);  /* SPI1_MOSI */
    GPIOA->AFR[1] |=
          (2<<GPIO_AFRH_AFRH1_Pos);  /* TIM1_CH2 */

    GPIOA->PUPDR |=
          (2<<GPIO_PUPDR_PUPDR1_Pos)  /* RS485 DE: Pulldown */
        | (1<<GPIO_PUPDR_PUPDR2_Pos)  /* TX */
        | (1<<GPIO_PUPDR_PUPDR3_Pos); /* RX */

    cfg_spi1();

    /* Pre-compute aux register values for timer ISR */
    for (int i=0; i<NSEGMENTS; i++) {
        segment_map[i] = 0xff00 ^ (0x100<<segment_map[i]);
    }

    /* Clear frame buffer */
    read_fb->brightness = 1;
    for (int i=0; i<sizeof(read_fb->data)/sizeof(uint32_t); i++) {
        read_fb->data[i] = 0xffffffff; /* FIXME DEBUG 0x00000000; */
    }

    cfg_timers_led();
    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    uart_config();
    adc_config();

    int k=0;
    while (42) {
        aux_reg = (read_fb->brightness ? SR_ILED_HIGH : SR_ILED_LOW) | (led_state<<1);
        if (k++ == 1000000) {
            k = 0;
            led_state = (led_state+1)&7;
        }

        /* Process pending buffer transfer */
        if (fb_op == FB_FORMAT) {
            transpose_data(rx_buf.byte_data, write_fb);
            fb_op = FB_UPDATE;
        }
    }
}

void NMI_Handler(void) {
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler() {
    asm volatile ("bkpt");
}

void SVC_Handler(void) {
}


void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
    static int n = 0;
    sys_time++;
    if (n++ == 1000) {
        n = 0;
        sys_time_seconds++;
    }
}

