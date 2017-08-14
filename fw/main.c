

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
    uint32_t data[nbits*frame_size_words];
    int brightness; /* 0 or 1; controls global brighntess control */
};

struct framebuf fb[2] = {0};
struct framebuf *read_fb=fb+0, *write_fb=fb+1;
volatile int led_state = 0;
volatile enum { FB_WRITE, FB_UPDATE } fb_op;

#define LED_COMM     0x0001
#define LED_ERROR    0x0002
#define LED_ID       0x0004
#define SR_ILED_HIGH 0x0080
#define SR_ILED_LOW  0x0040

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
                struct framebuf *tmp = read_fb;
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
    //TIM3->CR1 &= ~TIM_CR1_CEN_Msk;

    static int last_ivl;
    last_ivl = TIM3->CNT;
    /* This takes about 10us */
    int period = shift_data();
    static int ivl;
    ivl = TIM3->CNT - last_ivl;
    TIM3->CCR1 = period;
    TIM3->CNT = 0xffff; /* To not enable OC1 right away */

    TIM3->SR &= ~TIM_SR_CC1IF_Msk;
    //TIM3->CR1 |= TIM_CR1_CEN;
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

    LL_Init1msTick(SystemCoreClock);

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOA->MODER |=
          (2<<GPIO_MODER_MODER5_Pos)  /* PA5  - SCLK */
        | (2<<GPIO_MODER_MODER6_Pos)  /* PA6  - LED !OE */
        | (2<<GPIO_MODER_MODER7_Pos)  /* PA7  - MOSI */
        | (1<<GPIO_MODER_MODER9_Pos)  /* PA9  - LED strobe */
        | (1<<GPIO_MODER_MODER10_Pos);/* PA10 - Auxiliary strobe */

    /* Set shift register IO GPIO output speed */
    GPIOA->OSPEEDR |=
          (2<<GPIO_OSPEEDR_OSPEEDR5_Pos)   /* SCLK   FIXME maybe try 0x2 here? */
        | (2<<GPIO_OSPEEDR_OSPEEDR6_Pos)   /* LED !OE   */
        | (2<<GPIO_OSPEEDR_OSPEEDR7_Pos)   /* MOSI */
        | (2<<GPIO_OSPEEDR_OSPEEDR9_Pos)   /* LED strobe */
        | (2<<GPIO_OSPEEDR_OSPEEDR10_Pos); /* Auxiliary strobe */

    GPIOA->AFR[0] |=
          (0<<GPIO_AFRL_AFRL5_Pos)   /* SPI1_SCK  */
        | (1<<GPIO_AFRL_AFRL6_Pos)   /* TIM3_CH1  */
        | (0<<GPIO_AFRL_AFRL7_Pos);  /* SPI1_MOSI */

    /* Configure SPI controller */
    SPI1->I2SCFGR = 0;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 |= LL_SPI_DATAWIDTH_16BIT;
    /* FIXME maybe try w/o BIDI */
    /* Baud rate PCLK/2 -> 25MHz */
    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (0<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA;

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

        int ctr = sys_time>>2;
        for (int bit=0, bmask=1; bit<nbits; bit++, bmask<<=1) {

            int data = 0;
            for (uint32_t ibit = 1, j=0; ibit; ibit<<=1, j++) {
                int _100 = (1<<nbits);
                int _1ff = (2*_100-1);
                int val = (ctr + (j<<(nbits-5))) & _1ff;
                val = val&_100 ? _1ff-val : val;
                data |= val&bmask ? ibit : 0;
            }

            for (int seg=0; seg<frame_size_words; seg++)
                write_fb->data[bit*frame_size_words + seg] = data;
        }
        fb_op = FB_UPDATE;
        while (fb_op == FB_UPDATE)
            ;
    }
}

void NMI_Handler(void) {
}

void HardFault_Handler(void) {
    for(;;);
}

void SVC_Handler(void) {
}


void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
    sys_time++;
}

