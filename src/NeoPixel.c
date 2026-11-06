/**********************************************************************
* CPEG222 Project 7 - NeoPixel helper (source)
*
* A NeoPixel (WS2812) bit is a pulse of fixed period (1.25 us = 800 kHz):
*     a "0" bit = short HIGH (0.4 us), a "1" bit = long HIGH (0.8 us).
* TIM2_CH4 makes the PWM on PB11 with ARR = 111 (112 counts @ 90 MHz).
*     "0" -> CCR4 = 36 counts (0.4 us)     "1" -> CCR4 = 72 counts (0.8 us)
* DMA1 Stream7 (channel 3) copies one CCR4 value per PWM period (on the TIM2
* update event), so the CPU never has to bit-bang the 96 bits.
* Each pixel needs 24 bits in GRB order, MSB first.  After the data the line
* is held LOW for > 50 us (the buffer ends with zero entries) = "latch".
**********************************************************************/
#include "NeoPixel.h"

#define NUM_PIXELS   4U
#define BITS_PER_PX  24U
#define RESET_SLOTS  48U     // 48 * 1.25 us = 60 us low time
#define BUF_LEN      (NUM_PIXELS * BITS_PER_PX + RESET_SLOTS)

#define T_PERIOD     112U    // 90 MHz / 112 = 803 kHz (~1.25 us)
#define T_ZERO       36U     // 0.4 us
#define T_ONE        72U     // 0.8 us

static uint32_t npBuf[BUF_LEN];   // one CCR4 value per PWM period

void NeoPixel_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // PB11 -> alternate function (10), AF1 = TIM2_CH4, fast speed
    GPIOB->MODER   &= ~(3U << (11 * 2));
    GPIOB->MODER   |=  (2U << (11 * 2));
    GPIOB->OSPEEDR |=  (3U << (11 * 2));
    GPIOB->AFR[1]  &= ~(0xFU << ((11 - 8) * 4));
    GPIOB->AFR[1]  |=  (1U   << ((11 - 8) * 4));

    // TIM2: 90 MHz timer clock, period 112 counts, CH4 in PWM mode 1 with preload
    TIM2->CR1  = 0;
    TIM2->PSC  = 0;
    TIM2->ARR  = T_PERIOD - 1;
    TIM2->CCMR2 &= ~(0xFFU << 8);
    TIM2->CCMR2 |= (6U << 12) | TIM_CCMR2_OC4PE;   // OC4M = 110 (PWM1), preload on
    TIM2->CCER |= TIM_CCER_CC4E;                    // enable CH4 output
    TIM2->CCR4 = 0;
    TIM2->DIER = 0;

    // DMA1 Stream7, channel 3 = TIM2 (update / CH4 request)
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream7->CR & DMA_SxCR_EN) { }
}

void NeoPixel_ShowAll(uint8_t r, uint8_t g, uint8_t b)
{
    // ---- 1. Build the buffer: 24 bits per pixel, GRB order, MSB first
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    uint32_t i = 0;
    for (uint32_t px = 0; px < NUM_PIXELS; px++) {
        for (int bit = 23; bit >= 0; bit--) {
            npBuf[i++] = ((grb >> bit) & 1U) ? T_ONE : T_ZERO;
        }
    }
    while (i < BUF_LEN) npBuf[i++] = 0;            // low "latch" time at the end

    // ---- 2. Make sure timer and DMA are stopped, line low, counter reset
    TIM2->CR1  &= ~TIM_CR1_CEN;
    TIM2->DIER &= ~TIM_DIER_UDE;
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream7->CR & DMA_SxCR_EN) { }
    DMA1->HIFCR = (0x3DU << 22);                    // clear all Stream7 flags

    TIM2->CCR4 = 0;
    TIM2->CNT  = 0;
    TIM2->EGR  = TIM_EGR_UG;                        // load preload (CCR4 = 0)
    TIM2->SR   = 0;

    // ---- 3. Configure and start DMA: memory -> CCR4, 32-bit, one shot
    DMA1_Stream7->PAR  = (uint32_t)&TIM2->CCR4;
    DMA1_Stream7->M0AR = (uint32_t)npBuf;
    DMA1_Stream7->NDTR = BUF_LEN;
    DMA1_Stream7->CR = (3U << 25)                   // CHSEL = 3 (TIM2)
                     | DMA_SxCR_PL_1                // high priority
                     | DMA_SxCR_MSIZE_1             // 32-bit memory
                     | DMA_SxCR_PSIZE_1             // 32-bit peripheral
                     | DMA_SxCR_MINC                // next buffer entry each time
                     | DMA_SxCR_DIR_0;              // memory -> peripheral
    DMA1_Stream7->CR |= DMA_SxCR_EN;

    // ---- 4. Start the PWM; every update event asks DMA for the next value
    TIM2->DIER |= TIM_DIER_UDE;
    TIM2->CR1  |= TIM_CR1_CEN;

    // ---- 5. Wait for DMA to finish (~180 us), then stop the timer (line stays low)
    uint32_t guard = 200000;
    while (!(DMA1->HISR & (1U << 27)) && --guard) { }   // TCIF7
    for (volatile uint32_t k = 0; k < 2000; k++) { __NOP(); }  // let last period finish (~5 us+)
    TIM2->CR1  &= ~TIM_CR1_CEN;
    TIM2->DIER &= ~TIM_DIER_UDE;
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
    DMA1->HIFCR = (0x3DU << 22);
}
