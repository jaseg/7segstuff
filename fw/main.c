

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#include <stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <stm32f0xx_ll_spi.h>
#pragma GCC diagnostic pop

#include <system_stm32f0xx.h>

#include <stdint.h>
#include <string.h>
#include <unistd.h>

/* 
 * Part number: STM32F030F4C6
 */

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

static volatile unsigned int sys_time = 0;

enum Segment { SegA, SegB, SegC, SegD, SegE, SegF, SegG, SegDP, nsegments };
enum {
    nrows = 4,
    ncols = 8,
    nbits = 10,
};
enum {
    frame_size_words = nrows*ncols*nsegments/32,
};
struct framebuf {
    /* Multiplexing order: first Digits, then Time/bits, last Segments */
    union {
        uint32_t data[nbits*frame_size_words];
        struct {
            struct {
                uint32_t data[frame_size_words];
            } frame[nbits];
        };
    };
    uint8_t brightness; /* 0 or 1; controls global brighntess control */
};

volatile struct framebuf fb[2] = {0};
volatile struct framebuf *read_fb=fb+0, *write_fb=fb+1;
volatile int led_state = 0;
volatile enum { FB_WRITE, FB_FORMAT, FB_UPDATE } fb_op;
volatile uint8_t rx_buf[sizeof(struct framebuf)];
volatile uint8_t this_addr = 0x05; /* FIXME */

#define LED_COMM     0x0001
#define LED_ERROR    0x0002
#define LED_ID       0x0004
#define SR_ILED_HIGH 0x0080
#define SR_ILED_LOW  0x0040

void transpose_data(volatile uint8_t *rx_buf, volatile struct framebuf *out_fb) {
    memset((uint8_t *)out_fb, 0, sizeof(*out_fb));
    struct data_format {
        union {
            uint8_t high[8];
            struct { uint8_t ah, bh, ch, dh, eh, fh, gh, dph; };
        };
        union {
            uint16_t low;
            struct { uint8_t dpl:2, gl:2, fl:2, el:2, dl:2, cl:2, bl:2, al:2; };
        };
    };
    struct data_format *rxp = (struct data_format *)rx_buf;
    for (int bit=0; bit<8; bit++) { /* bits */
        uint32_t bit_mask = 1U<<bit;
        volatile uint32_t *frame_data = out_fb->frame[bit+2].data;
        uint8_t *start_inp = rxp->high;
        for (volatile uint32_t *outp=frame_data; outp<frame_data+8; outp++) { /* segments */
            uint32_t acc = 0;
            uint8_t *inp = start_inp++;
            for (int digit=0; digit<32; digit++) {
                acc |= (*inp & bit_mask) >> bit << digit;
                inp += sizeof(struct data_format);
            }
            *outp = acc;
        }
    }
    for (int bit=0; bit<2; bit++) { /* bits */
        volatile uint32_t *frame_data = out_fb->frame[bit].data;
        uint16_t *inp = &rxp->low;
        for (int seg=0; seg<8; seg++) { /* segments */
            uint32_t mask = 1 << bit << seg;
            uint32_t acc = 0;
            for (int digit=0; digit<32; digit++) {
                acc |= (*inp & mask) >> bit >> seg << digit;
                inp += sizeof(struct data_format)/sizeof(uint16_t);
            }
            frame_data[seg] = acc;
        }
    }
}

void shift_aux(int global_current, int leds, int active_segment) {
    spi_send(
              (global_current ? SR_ILED_HIGH : SR_ILED_LOW)
            | (leds<<1)
            | (0xff00 ^ (0x100<<active_segment)));
    strobe_aux();
}

static volatile int frame_duration;
/* returns new bit time in cycles */
int shift_data() {
    static int active_segment = 0;
    static unsigned int active_bit = 0;
    static int last_frame_sys_time;

    /* Note: On boot, multiplexing will start with bit 1 due to the next few lines. This is perfectly ok. */
    active_bit++;
    if (active_bit == nbits) {
        active_bit = 0;

        active_segment++;
        if (active_segment == nsegments) {
            active_segment = 0;

            int time = sys_time;
            frame_duration = time - last_frame_sys_time;
            last_frame_sys_time = sys_time;
            if (fb_op == FB_UPDATE) {
                volatile struct framebuf *tmp = read_fb;
                read_fb = write_fb;
                write_fb = tmp;
                fb_op = FB_WRITE;
            }
        }

        shift_aux(read_fb->brightness, led_state, active_segment);
    }

    uint32_t current_word = read_fb->data[active_bit*frame_size_words + active_segment];
    spi_send(current_word&0xffff);
    spi_send(current_word>>16);
    strobe_leds();

    return 1<<active_bit;
}

void cfg_timer3() {
    TIM3->CCMR1 = 6<<TIM_CCMR1_OC1M_Pos; /* PWM Mode 1 */
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P; /* Inverting output */
    TIM3->DIER  = TIM_DIER_CC1IE;
    TIM3->CCR1  = 1000; /* Schedule first interrupt */
    TIM3->PSC   = SystemCoreClock/5000000 * 2; /* 0.40us/tick */
    TIM3->ARR   = 0xffff;
    TIM3->EGR  |= TIM_EGR_UG;
    TIM3->CR1   = TIM_CR1_ARPE;
    TIM3->CR1  |= TIM_CR1_CEN;
}

TIM_TypeDef *tim3 = TIM3;

void TIM3_IRQHandler() {
    //TIM3->CR1 &= ~TIM_CR1_CEN_Msk; FIXME

    /* This takes about 10us */
    int period = 0; /* FIXME DEBUG shift_data(); */
    TIM3->CCR1 = period;
    TIM3->CNT = 0xffff; /* To not enable OC1 right away */

    TIM3->SR &= ~TIM_SR_CC1IF_Msk;
    //TIM3->CR1 |= TIM_CR1_CEN;
}

enum Command {
    CMD_PING,
    CMD_SET_ADDR,
    CMD_SET_FB,
    CMD_GET_DESC,
};

void USART1_IRQHandler() {
    int addr = USART1->RDR;
    int cmd = addr>>5;
    addr &= 0x1F;
    /* Are we addressed? */
    if (addr != this_addr) {
        /* We are not. Mute USART until next idle condition */
        USART1->RQR |= USART_RQR_MMRQ;
    } else {
        /* We are. Switch by command. */
        switch (cmd) {
        case CMD_SET_FB:
            /* Are we ready to process new frame data? */
            if (fb_op != FB_WRITE)
                goto errout; /* Error: Not yet ready to receive new packet */
            /* Disable this RX interrupt for duration of DMA transfer */
            USART1->CR1 &= ~USART_CR1_RXNEIE_Msk;
            /* Enable DMA transfer to write buffer */
            DMA1_Channel3->CCR |= DMA_CCR_EN;
            /* Kick off formatting code in main loop outside interrupt context */
            fb_op = FB_FORMAT;
            break;
        }
    }
errout:
    /* FIXME */
    return;
}

void DMA1_Channel2_3_IRQHandler() {
    /* DMA Transfer complete, re-enable receive interrupt */
    USART1->CR1 |= USART_CR1_RXNEIE;
}

int main(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk & ~RCC_CFGR_SW_Msk & ~RCC_CFGR_PPRE_Msk & ~RCC_CFGR_HPRE_Msk;
    RCC->CFGR |= (1<<RCC_CFGR_PLLMUL_Pos) | RCC_CFGR_PLLSRC_HSE_PREDIV; /* PLL x4 -> 50.0MHz */
    RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_Msk;
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV2; /* prediv :2 -> 12.5MHz */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    SystemCoreClockUpdate();

    //SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    //NVIC_DisableIRQ(SysTick_IRQn);
    SysTick->VAL = 0U;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (42) {
        static unsigned int cvr __attribute__((used));
        cvr = SysTick->VAL;
        //if (fb_op == FB_FORMAT) {
            transpose_data(rx_buf, write_fb);
            write_fb->brightness = rx_buf[offsetof(struct framebuf, brightness)];
        //    fb_op = FB_UPDATE;
        //    while (fb_op == FB_UPDATE)
        //        ;
        //}
        cvr = cvr -  SysTick->VAL;
        asm volatile ("bkpt");
    }
    //LL_Init1msTick(SystemCoreClock);

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_DMAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOA->MODER |=
          (2<<GPIO_MODER_MODER1_Pos)  /* PA1  - RS485 DE */
        | (2<<GPIO_MODER_MODER2_Pos)  /* PA2  - RS485 TX */
        | (2<<GPIO_MODER_MODER3_Pos)  /* PA3  - RS485 RX */
        | (2<<GPIO_MODER_MODER5_Pos)  /* PA5  - SCLK */
        | (2<<GPIO_MODER_MODER6_Pos)  /* PA6  - LED !OE */
        | (2<<GPIO_MODER_MODER7_Pos)  /* PA7  - MOSI */
        | (1<<GPIO_MODER_MODER9_Pos)  /* PA9  - LED strobe */
        | (1<<GPIO_MODER_MODER10_Pos);/* PA10 - Auxiliary strobe */

    /* Set shift register IO GPIO output speed */
    GPIOA->OSPEEDR |=
          (2<<GPIO_OSPEEDR_OSPEEDR1_Pos)   /* RS485 DE */
        | (2<<GPIO_OSPEEDR_OSPEEDR2_Pos)   /* TX */
        | (2<<GPIO_OSPEEDR_OSPEEDR3_Pos)   /* RX */
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

    GPIOA->PUPDR |=
          (2<<GPIO_PUPDR_PUPDR1_Pos)  /* RS485 DE: Pulldown */
        | (1<<GPIO_PUPDR_PUPDR2_Pos)  /* TX */
        | (1<<GPIO_PUPDR_PUPDR3_Pos); /* RX */

    /* Configure SPI controller */
    SPI1->I2SCFGR = 0;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 |= LL_SPI_DATAWIDTH_16BIT;

    /* Configure USART1 */
    USART1->CR1 = /* 8-bit -> M1, M0 clear */
        /* RTOIE clear */
          (8 << USART_CR1_DEAT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        | (8 << USART_CR1_DEDT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        | USART_CR1_OVER8
        /* CMIF clear */
        | USART_CR1_MME
        /* WAKE clear */
        /* PCE, PS clear */
        | USART_CR1_RXNEIE
        /* other interrupts clear */
        | USART_CR1_TE
        | USART_CR1_RE;
    //USART1->CR2 = USART_CR2_RTOEN; /* Timeout enable */
    USART1->CR3 = USART_CR3_DEM; /* RS485 DE enable (output on RTS) */
    int usartdiv = 25;
    USART1->BRR = (usartdiv&0xFFF0) | ((usartdiv>>1) & 0x7);
    USART1->CR1 |= USART_CR1_UE;

    /* Configure DMA for USART frame data reception */
    USART1->CR3 |= USART_CR3_DMAR;
    DMA1_Channel3->CPAR = (unsigned int)&USART1->RDR;
    DMA1_Channel3->CMAR = (unsigned int)rx_buf;
    DMA1_Channel3->CNDTR = sizeof(rx_buf);
    DMA1_Channel3->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel3->CCR |=
          (0<<DMA_CCR_MSIZE_Pos)
        | (0<<DMA_CCR_PSIZE_Pos)
        | DMA_CCR_MINC
        | DMA_CCR_TCIE;

    /* Baud rate PCLK/2 -> 25MHz */
    SPI1->CR1 =
          SPI_CR1_BIDIMODE
        | SPI_CR1_BIDIOE
        | SPI_CR1_SSM
        | SPI_CR1_SSI
        | SPI_CR1_SPE
        | (0<<SPI_CR1_BR_Pos)
        | SPI_CR1_MSTR
        | SPI_CR1_CPOL
        | SPI_CR1_CPHA;
    /* FIXME maybe try w/o BIDI */

    read_fb->brightness = 1;
    for (int i=0; i<sizeof(read_fb->data)/sizeof(uint32_t); i++) {
        read_fb->data[i] = 0xffffffff;
    }
    cfg_timer3();

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 2);
 
    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */

    while (42) {
        led_state = (sys_time>>8)&7;
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
    sys_time++;
}

