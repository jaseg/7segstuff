
#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <string.h>
#include <unistd.h>
/* 
 * Part number: STM32F030F4C6
 */

void tick(void) {
    for(int i=0; i<50; i++)
        __asm__("nop");
}

int main(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk & ~RCC_CFGR_SW_Msk;
    RCC->CFGR |= (2<<RCC_CFGR_PLLMUL_Pos) | RCC_CFGR_PLLSRC_HSE_PREDIV; /* PLL x4 */
    RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_Msk; /* PREDIV=0 */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    SystemCoreClockUpdate();

    LL_Init1msTick(SystemCoreClock);

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIOA->MODER |=
          (2<<GPIO_MODER_MODER5_Pos)  /* PA5  - SCLK */
        | (2<<GPIO_MODER_MODER7_Pos)  /* PA7  - MOSI */
        | (1<<GPIO_MODER_MODER9_Pos)  /* PA9  - LED strobe */
        | (1<<GPIO_MODER_MODER10_Pos);/* PA10 - Auxiliary strobe */

    /* Set shift register IO GPIO output speed */
    GPIOA->OSPEEDR |=
          (3<<GPIO_OSPEEDR_OSPEEDR5_Pos)   /* SCLK   */
        | (3<<GPIO_OSPEEDR_OSPEEDR7_Pos)   /* MOSI   */
        | (3<<GPIO_OSPEEDR_OSPEEDR9_Pos)   /* LED strobe   */
        | (3<<GPIO_OSPEEDR_OSPEEDR10_Pos); /* Auxiliary strobe   */

    GPIOA->AFR[0] |=
          (0<<GPIO_AFRL_AFRL5_Pos)   /* SPI1_SCK  */
        | (0<<GPIO_AFRL_AFRL7_Pos);  /* SPI1_MOSI */

    /* Configure SPI controller */
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (7<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI1->CR2 = (7<<SPI_CR2_DS_Pos);

    int pos1 = 0;
    int pos2 = 0;

    while (42) {
        for (int i=0; i<6; i++) {
            if (pos1 == i) {
                SPI1->DR = 1<<pos2;
            } else {
                SPI1->DR = 0;
            }
            while (SPI1->SR & SPI_SR_BSY);
            tick();
        }
        pos2 += 1;
        if (pos2 == 8) {
            pos2 = 0;
            pos1 += 1;
            if (pos1 == 6)
                pos1 = 0;
        }
        /* Strobe both LED drivers and aux regs */
        GPIOA->BSRR = GPIO_BSRR_BS_9 | GPIO_BSRR_BS_10;
        tick();
        GPIOA->BSRR = GPIO_BSRR_BR_9 | GPIO_BSRR_BR_10;
        LL_mDelay(1);
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

