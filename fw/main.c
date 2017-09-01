

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

#include "transpose.h"

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

volatile unsigned int sys_time = 0;

volatile struct framebuf fb[2] = {0};
volatile struct framebuf *read_fb=fb+0, *write_fb=fb+1;
volatile int led_state = 0;
volatile enum { FB_WRITE, FB_FORMAT, FB_UPDATE } fb_op;
volatile uint8_t rx_buf[sizeof(struct framebuf) + 4 /* crc */];
volatile union {
    struct {
        uint32_t millifps;
    } fps_reply;
} tx_buf;
volatile uint8_t this_addr = 0x05; /* FIXME */

union {
    struct {
        uint8_t device_id __attribute__((aligned(2)));
    };
    uint16_t flash_data[0];
} flash_data __attribute__((section ("configflash")));

void write_config(void *target, uint16_t value) {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->CR = FLASH_CR_PG;
    *(uint16_t *)target = value;
    while (FLASH->SR & FLASH_SR_BSY)
        ;
    FLASH->CR = FLASH_CR_LOCK;
}

void erase_config() {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = (uint32_t)&flash_data;
    FLASH->CR = FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY)
        ;
    FLASH->CR = FLASH_CR_LOCK;
}

#define LED_COMM     0x0001
#define LED_ERROR    0x0002
#define LED_ID       0x0004
#define SR_ILED_HIGH 0x0080
#define SR_ILED_LOW  0x0040

inline unsigned int stk_start() {
    return SysTick->VAL;
}

inline unsigned int stk_end(unsigned int start) {
    return (start - SysTick->VAL) & 0xffffff;
}

inline unsigned int stk_microseconds() {
    return sys_time*1000 + (1000 - (SysTick->VAL / (SystemCoreClock/1000000)));
}

enum {
    SPI_AUX,
    SPI_WORD0,
    SPI_WORD1,
    SPI_IDLE,
} spi_state;
static volatile uint32_t spi_word = 0;

void cfg_spi1() {
    /* Configure SPI controller */
    SPI1->I2SCFGR = 0;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 |= LL_SPI_DATAWIDTH_16BIT;

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

    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_SetPriority(SPI1_IRQn, 2);
}

void SPI1_IRQHandler() {
    switch (spi_state) {
        case SPI_AUX:
            strobe_aux();
            SPI1->DR = spi_word>>16;
            break;
        case SPI_WORD0:
            SPI1->DR = spi_word&0xFFFF;
            break;
        default:
            tick(); /* This one is important. Otherwise, weird stuff happens and parts of the aux register seem to leak
                       into the driver registers. */
            strobe_leds();
            SPI1->CR2 &= ~SPI_CR2_TXEIE;
            break;
    }
    spi_state ++;
}

uint8_t segment_map[8] = {5, 7, 6, 4, 1, 3, 0, 2};

static volatile int frame_duration_us;
/* returns new bit time in cycles */
int shift_data() {
    static int active_segment = 0;
    static unsigned int active_bit = 0;
    static unsigned int last_frame_time;

    /* Note: On boot, multiplexing will start with bit 1 due to the next few lines. This is perfectly ok. */
    active_bit++;
    if (active_bit == nbits) {
        active_bit = 0;

        active_segment++;
        if (active_segment == nsegments) {
            active_segment = 0;

            int time = stk_microseconds();
            frame_duration_us = time - last_frame_time;
            last_frame_time = time;
            if (fb_op == FB_UPDATE) {
                volatile struct framebuf *tmp = read_fb;
                read_fb = write_fb;
                write_fb = tmp;
                fb_op = FB_WRITE;
            }
        }

        spi_word = read_fb->data[active_bit*frame_size_words + active_segment];
        spi_state = SPI_AUX;
        SPI1->DR = (read_fb->brightness ? SR_ILED_HIGH : SR_ILED_LOW)
            | (led_state<<1)
            | (0xff00 ^ (0x100<<segment_map[active_segment]));
    } else {
        spi_word = read_fb->data[active_bit*frame_size_words + active_segment];
        spi_state = SPI_WORD0;
        SPI1->DR = spi_word>>16;
    }
    SPI1->CR2 |= SPI_CR2_TXEIE;

    return 1<<active_bit;
}

void cfg_timer3() {
    /* Capture/compare channel 1 is used to generate the LED driver !OE signal. Channel 2 is used to trigger the
     * interrupt to load the next bits in to the shift registers. Channel 2 triggers simultaneously with channel 1 at
     * long !OE periods but will be delayed slightly to a fixed 32 timer periods (12.8us) to allow for SPI1 to finish
     * shifting out all frame data before asserting !OE. */
    TIM3->CCMR1 = 6<<TIM_CCMR1_OC1M_Pos; /* PWM Mode 1 */
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P; /* Inverting output */
    TIM3->DIER  = TIM_DIER_CC2IE;
    TIM3->CCR2  = 1000; /* Schedule first interrupt */
    TIM3->PSC   = SystemCoreClock/5000000 * 2; /* 0.40us/tick */
    TIM3->ARR   = 0xffff;
    TIM3->EGR  |= TIM_EGR_UG;
    TIM3->CR1   = TIM_CR1_ARPE;
    TIM3->CR1  |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 2);
}

void TIM3_IRQHandler() {
    //TIM3->CR1 &= ~TIM_CR1_CEN_Msk; FIXME

    int period = shift_data();
    TIM3->CCR1 = period;
    if (period < 32) /* FIXME this constant */
        TIM3->CCR2 = 32;
    else
        TIM3->CCR2 = period;
    TIM3->CNT = 0xffff; /* To not enable OC1 riglt away */

    TIM3->SR &= ~TIM_SR_CC2IF_Msk;
    //TIM3->CR1 |= TIM_CR1_CEN;
}

enum Command {
    CMD_PING,
    CMD_SET_ADDR,
    CMD_SET_FB,
    CMD_GET_DESC,
    CMD_GET_FRAMERATE,
    CMD_ERASE_CONFIG,
};

enum {
    PROT_ADDR,
    PROT_CMD,
    PROT_
} protocol_state = PROT_ADDR;

int gaddr;
void USART1_IRQHandler() {
    gaddr = USART1->RDR;
    USART1->RQR |= USART_RQR_RXFRQ;
    int addr = gaddr; /* FIXME DEBUG */
    int cmd = addr>>5;
    addr &= 0x1F;
    /* Overrun detected? */
    if (USART1->ISR & USART_ISR_ORE) {
        USART1->ICR |= USART_ICR_ORECF;
        USART1->RQR |= USART_RQR_MMRQ;
    }
    /* Are we addressed? */
    if (addr != this_addr) {
        asm("bkpt");
        /* We are not. Mute USART until next idle condition */
        USART1->RQR |= USART_RQR_MMRQ;
    } else {
        /* We are. Switch by command. */
        switch (cmd) {
        case CMD_PING:
            USART1->TDR = 42;
            USART1->RQR |= USART_RQR_MMRQ;
            break;
        case CMD_SET_FB:
            /* Are we ready to process new frame data? */
            if (fb_op != FB_WRITE) {
                //asm("bkpt"); /* FIXME DEBUG */
                goto errout; /* Error: Not yet ready to receive new packet */
            }
            /* Disable this RX interrupt for duration of DMA transfer */
            USART1->CR1 &= ~USART_CR1_RXNEIE_Msk;
            /* Enable DMA transfer to write buffer */
            DMA1->IFCR |= DMA_IFCR_CGIF3;
            DMA1_Channel3->CCR |= DMA_CCR_EN;
            USART1->CR3 |= USART_CR3_DMAR;
            break;
        case CMD_GET_FRAMERATE:
            tx_buf.fps_reply.millifps = frame_duration_us > 0 ? 1000000000 / frame_duration_us : 0;
            DMA1_Channel4->CNDTR = sizeof(tx_buf.fps_reply);
            USART1->ICR |= USART_ICR_TCCF; /* FIXME: (1) is this necessary? (2) where should this be done? */
            DMA1_Channel4->CCR |= DMA_CCR_EN;
            USART1->RQR |= USART_RQR_MMRQ;
            break;
        }
    }
errout:
    /* FIXME */
return;
}

void DMA1_Channel2_3_IRQHandler() {
    /* DMA Transfer complete */
    /* ...and disable this DMA channel */
    USART1->CR3 &= ~USART_CR3_DMAR_Msk;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN_Msk;
    /* Kick off formatting code in main loop outside interrupt context */
    fb_op = FB_FORMAT;
    DMA1->IFCR |= DMA_IFCR_CGIF3;
    /* re-enable receive interrupt */
    USART1->RQR |= USART_RQR_RXFRQ;
    USART1->CR1 |= USART_CR1_RXNEIE;
}

void DMA1_Channel4_5_IRQHandler() {
    DMA1->IFCR |= DMA_IFCR_CGIF4;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN_Msk;
}

void uart_config(void) {
    USART1->CR1 = /* 8-bit -> M1, M0 clear */
        /* RTOIE clear */
          (8 << USART_CR1_DEAT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        | (8 << USART_CR1_DEDT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        //| USART_CR1_OVER8 FIXME debug?
        /* CMIF clear */
        | USART_CR1_MME
        /* WAKE clear */
        /* PCE, PS clear */
        | USART_CR1_RXNEIE
        /* other interrupts clear */
        | USART_CR1_TE
        | USART_CR1_RE;
    //USART1->CR2 = USART_CR2_RTOEN; /* Timeout enable */
    USART1->CR3 = USART_CR3_DEM /* RS485 DE enable (output on RTS) */
            | USART_CR3_DMAT;
    int usartdiv = 25;
    USART1->BRR = usartdiv;
    USART1->CR1 |= USART_CR1_UE;

    /* Configure DMA 1 / Channel 3 for USART frame data reception: USART1->RDR -> rx_buf */
    DMA1_Channel3->CPAR = (unsigned int)&USART1->RDR;
    DMA1_Channel3->CMAR = (unsigned int)rx_buf;
    DMA1_Channel3->CNDTR = sizeof(rx_buf);
    DMA1_Channel3->CCR = (1<<DMA_CCR_PL_Pos);
    DMA1_Channel3->CCR |=
          (0<<DMA_CCR_MSIZE_Pos) /* 8 bit */
        | (0<<DMA_CCR_PSIZE_Pos) /* 8 bit */
        | DMA_CCR_MINC
        | DMA_CCR_TCIE
        | DMA_CCR_CIRC;

    /* Configure DMA 1 / Channel 5 for USART frame data CRC check: rx_buf -> CRC->DR */
    DMA1_Channel5->CPAR = (unsigned int)&CRC->DR;
    DMA1_Channel5->CMAR = (unsigned int)rx_buf;
    DMA1_Channel5->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel5->CCR |=
          DMA_CCR_MEM2MEM /* Software trigger (precludes CIRC) */
        | DMA_CCR_DIR /* Read from memory */
        | (0<<DMA_CCR_MSIZE_Pos) /* 8 bit */
        | (0<<DMA_CCR_PSIZE_Pos) /* 8 bit */
        | DMA_CCR_MINC;

    /* Configure DMA 1 / Channel 4 for USART reply transmission */
    DMA1_Channel4->CPAR = (unsigned int)&USART1->TDR;
    DMA1_Channel4->CMAR = (unsigned int)&tx_buf;
    DMA1_Channel4->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel4->CCR |=
          DMA_CCR_DIR /* Read from memory */
        | (0<<DMA_CCR_MSIZE_Pos) /* 8 bit */
        | (0<<DMA_CCR_PSIZE_Pos) /* 8 bit */
        | DMA_CCR_MINC
        | DMA_CCR_TCIE;
    
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 4);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
    NVIC_SetPriority(DMA1_Channel4_5_IRQn, 5);
}

int errcnt = 0;
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

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_DMAEN | RCC_AHBENR_CRCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;
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

    cfg_spi1();

    /* Clear frame buffer */
    read_fb->brightness = 1;
    for (int i=0; i<sizeof(read_fb->data)/sizeof(uint32_t); i++) {
        read_fb->data[i] = 0xffffffff; /* FIXME DEBUG 0x00000000; */
    }

    cfg_timer3();
    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    uart_config();

    while (42) {
        if (fb_op == FB_FORMAT) {
            CRC->CR |= CRC_CR_RESET;
            DMA1_Channel5->CNDTR = sizeof(struct framebuf);
            DMA1_Channel5->CCR |= DMA_CCR_EN;

            transpose_data(rx_buf, write_fb);

            while (!(DMA1->ISR & DMA_ISR_TCIF4))
                ;
            DMA1->IFCR |= DMA_IFCR_CGIF4;
            DMA1_Channel5->CCR &= ~DMA_CCR_EN_Msk;

            if (CRC->DR != *(uint32_t *)(rx_buf+sizeof(struct framebuf))) {
                fb_op = FB_WRITE;
                errcnt++;
            } else {
                fb_op = FB_UPDATE;
                while (fb_op == FB_UPDATE)
                    ;
            }
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
    sys_time++;
}

