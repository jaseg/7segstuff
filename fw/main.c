

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
volatile  int16_t adc_temp_tenth_celsius = 0;

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

uint8_t segment_map[8] = {5, 7, 6, 4, 1, 3, 0, 2};

static volatile uint32_t aux_reg = 0;
static volatile int frame_duration_us;
volatile int nbits = MAX_BITS;

static unsigned int active_bit = 0;
static int active_segment = 0;

/* Bit timing base value. This is the lowes bit interval used */
#define PERIOD_BASE 4

/* This value is a constant offset added to every bit period to allow for the timer IRQ handler to execute. This is set
 * empirically using a debugger and a logic analyzer. */
#define TIMER_CYCLES_FOR_SPI_TRANSMISSIONS 13

#define TIMER_CYCLES_BEFORE_LED_STROBE 12

#define AUX_SPI_PRETRIGGER 20

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

void cfg_timer3() {
    /* FIXME update comment */
    /* Capture/compare channel 1 is used to generate the LED driver !OE signal. Channel 2 is used to trigger the
     * interrupt to load the next bits in to the shift registers. Channel 2 triggers simultaneously with channel 1 at
     * long !OE periods but will be delayed slightly to a fixed 32 timer periods (12.8us) to allow for SPI1 to finish
     * shifting out all frame data before asserting !OE. */
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

    TIM1->SR    = 0;
    TIM1->BDTR  = TIM_BDTR_MOE;
    TIM1->SMCR  = (2<<TIM_SMCR_TS_Pos) | (4<<TIM_SMCR_SMS_Pos); /* Internal Trigger 2 (ITR2) -> TIM3; slave mode: reset */
    TIM1->CCMR1 = (6<<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; /* PWM Mode 1, enable CCR preload */
    TIM1->CCER  = TIM_CCER_CC2E | TIM_CCER_CC1E;
    TIM1->CCR1  = timer_period_lookup[nbits-1] - AUX_SPI_PRETRIGGER;
    TIM1->DIER  = TIM_DIER_CC1IE;
    TIM1->CCR2  = TIMER_CYCLES_BEFORE_LED_STROBE;
    TIM1->PSC   = TIM3->PSC; /* 0.20us/tick */
    TIM1->ARR   = 0xffff;
    TIM1->EGR  |= TIM_EGR_UG;
    TIM1->CR1   = TIM_CR1_ARPE;
    TIM1->CR1  |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM1_CC_IRQn);
    NVIC_SetPriority(TIM1_CC_IRQn, 3);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 2);
}

void TIM1_CC_IRQHandler() {
    GPIOA->BSRR = GPIO_BSRR_BS_0; // Debug

    active_bit = 0;
    active_segment++;
    if (active_segment == NSEGMENTS) {
        active_segment = 0;

        /* FIXME remove this?
        int time = stk_microseconds();
        frame_duration_us = time - last_frame_time;
        last_frame_time = time;
        */
        if (fb_op == FB_UPDATE) {
            volatile struct framebuf *tmp = read_fb;
            read_fb = write_fb;
            write_fb = tmp;
            fb_op = FB_WRITE;
        }
    }

    GPIOA->BSRR = GPIO_BSRR_BR_10;
    SPI1->DR = aux_reg | segment_map[active_segment];

    TIM1->SR &= ~TIM_SR_CC1IF_Msk;
    GPIOA->BSRR = GPIO_BSRR_BR_0; // Debug
}

void TIM3_IRQHandler() {
    GPIOA->BSRR = GPIO_BSRR_BS_4; // Debug
    //TIM3->CR1 &= ~TIM_CR1_CEN_Msk; FIXME

    GPIOA->BSRR = GPIO_BSRR_BS_10;

    /* Note: On boot, multiplexing will start with bit 1 due to the next few lines. This is perfectly ok. */
    uint32_t spi_word = read_fb->data[active_bit*FRAME_SIZE_WORDS + active_segment];
    SPI1->DR = spi_word>>16;
    spi_word &= 0xFFFF;
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = spi_word;
    while (!(SPI1->SR & SPI_SR_TXE));

    active_bit++;
    TIM3->ARR = timer_period_lookup[active_bit];

    TIM3->SR &= ~TIM_SR_UIF_Msk;
    GPIOA->BSRR = GPIO_BSRR_BR_4; // Debug
}

enum Command {
    CMD_PING,
    CMD_SET_FB,
    CMD_SET_NBITS,
    CMD_GET_DESC,
    N_CMDS
};

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
}

#define ADC_OVERSAMPLING 12
uint32_t vsense;
void DMA1_Channel1_IRQHandler(void) {
    static int count = 0;
    static uint32_t adc_aggregate[2] = {0, 0};

    DMA1->IFCR |= DMA_IFCR_CGIF1;

    adc_aggregate[0] += adc_buf[0];
    adc_aggregate[1] += adc_buf[1];

    if (count++ == (1<<ADC_OVERSAMPLING)) {
        adc_vcc_mv = (3300 * VREFINT_CAL)/(adc_aggregate[0]>>ADC_OVERSAMPLING);
        vsense = ((adc_aggregate[1]>>ADC_OVERSAMPLING) * adc_vcc_mv)/4095 ;
        adc_temp_tenth_celsius = 300 - (((TS_CAL1*adc_vcc_mv/4095) - vsense)*100)/43;
        count = 0;
        adc_aggregate[0] = 0;
        adc_aggregate[1] = 0;
    }
}

void adc_config(void) {
    ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
    ADC1->CFGR2 = 2<<ADC_CFGR2_CKMODE_Pos;
    ADC1->SMPR = 7<<ADC_SMPR_SMP_Pos;
    ADC1->CHSELR = ADC_CHSELR_CHSEL16 | ADC_CHSELR_CHSEL17;
    ADC->CCR = ADC_CCR_TSEN | ADC_CCR_VREFEN;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL)
        ;
    ADC1->CR |= ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADSTART;
    /* FIXME handle adc overrun */

    DMA1_Channel1->CPAR = (unsigned int)&ADC1->DR;
    DMA1_Channel1->CMAR = (unsigned int)&adc_buf;
    DMA1_Channel1->CNDTR = sizeof(adc_buf)/sizeof(adc_buf[0]);
    DMA1_Channel1->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel1->CCR |=
          DMA_CCR_CIRC
        | (1<<DMA_CCR_MSIZE_Pos) /* 16 bit */
        | (1<<DMA_CCR_PSIZE_Pos) /* 16 bit */
        | DMA_CCR_MINC
        | DMA_CCR_TCIE;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 6);
}

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
    for (int i=0; i<sizeof(segment_map)/sizeof(segment_map[0]); i++) {
        segment_map[i] = 0xff00 ^ (0x100<<segment_map[i]);
    }

    /* Clear frame buffer */
    read_fb->brightness = 1;
    for (int i=0; i<sizeof(read_fb->data)/sizeof(uint32_t); i++) {
        read_fb->data[i] = 0xffffffff; /* FIXME DEBUG 0x00000000; */
    }

    cfg_timer3();
    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    uart_config();
    adc_config();

    volatile uint8_t *rxd = rx_buf.byte_data;
    int k=0;
    while (42) {
        aux_reg = (read_fb->brightness ? SR_ILED_HIGH : SR_ILED_LOW) | (led_state<<1);
        if (k++ == 1000000) {
            k = 0;
            led_state = (led_state+1)&7;
        }

        if (USART1->ISR & USART_ISR_RXNE) {
            *rxd++ = USART1->RDR;
            if (rxd >= rx_buf.set_fb_rq.end) {
                rxd = rx_buf.byte_data;
                if (fb_op == FB_FORMAT) {
                    transpose_data(rx_buf.byte_data, write_fb);
                    fb_op = FB_UPDATE;
                }
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
    static int n = 0;
    sys_time++;
    if (n++ == 1000) {
        n = 0;
        sys_time_seconds++;
    }
}

