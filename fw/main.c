
#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <stm32f0xx_ll_spi.h>
#include <string.h>
#include <unistd.h>
/* 
 * Part number: STM32F030F4C6
 */

void tick(void) {
    for(int i=0; i<50; i++)
        __asm__("nop");
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

#define SR_COMM      0x0002
#define SR_ERROR     0x0004
#define SR_ID        0x0008
#define SR_ILED_HIGH 0x0080
#define SR_ILED_LOW  0x0040
int main(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk & ~RCC_CFGR_SW_Msk;
    RCC->CFGR |= (2<<RCC_CFGR_PLLMUL_Pos) | RCC_CFGR_PLLSRC_HSE_PREDIV; /* PLL x4 */
    RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_Msk;
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    SystemCoreClockUpdate();

    LL_Init1msTick(SystemCoreClock);

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIOA->MODER |=
          (2<<GPIO_MODER_MODER5_Pos)  /* PA5  - SCLK */
        | (1<<GPIO_MODER_MODER6_Pos)  /* PA6  - LED !OE */
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
        | (0<<GPIO_AFRL_AFRL7_Pos);  /* SPI1_MOSI */

    /* Configure SPI controller */
    SPI1->I2SCFGR = 0;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 &= ~SPI_CR2_DS_Msk;
    SPI1->CR2 |= LL_SPI_DATAWIDTH_16BIT;
    /* FIXME maybe try w/o BIDI */
    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (0<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA;

    int val = 0xffff;
    int aval = 0x0000;
    GPIOA->BSRR = GPIO_BSRR_BR_6; /* OE */
    int j = 0;
    int ctr = 0;
    int q = 0;
    int bval = 0x400;
    while (42) {
        for (int i=0; i<8; i++) {
            spi_send(1<<(ctr&3));
            spi_send(1<<((ctr>>1)&3));
            strobe_leds();
            //spi_send(0x0200 | bval | (0xff^(1<<i)));
            //spi_send((0xff^(1<<i))<<8);
            //spi_send(SR_COMM | SR_ILED_HIGH | 0xff);
            //spi_send(0x00ff ^ (1<<ctr) | (0x100<<ctr));
            spi_send(bval | (0xff00 ^ (0x100<<i)));
            strobe_aux();
            for(int i=0; i<10; i++)
                tick();
            j++;
            if (j == 1000) {
                j = 0;
                ctr++;
                if (ctr == 8) {
                    ctr = 0;
                    q++;
                    if (q == 6)
                        q = 0;
                }
                switch (q) {
                    case 0:
                        bval = SR_COMM  | SR_ILED_LOW;
                        break;
                    case 1:
                        bval = SR_ID    | SR_ILED_LOW;
                        break;
                    case 2:
                        bval = SR_ERROR | SR_ILED_LOW;
                        break;
                    case 3:
                        bval = SR_COMM  | SR_ILED_HIGH;
                        break;
                    case 4:
                        bval = SR_ID    | SR_ILED_HIGH;
                        break;
                    case 5:
                        bval = SR_ERROR | SR_ILED_HIGH;
                        break;
                }
            }
        }
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
}

