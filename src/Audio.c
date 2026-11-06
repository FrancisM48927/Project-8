/**********************************************************************
* CPEG222 Project 7 - Audio helper (source)
*
* How the tone is made (no CPU involved once started):
*   Lookup table (RAM/flash) holds ONE sine cycle of 64 samples.
*   TIM6 overflows at (frequency x 64) and sends a TRGO trigger to the DAC.
*   Each trigger makes the DAC ask DMA for the next sample.
*   DMA1 Stream5 (circular) copies the next table entry into DAC1->DHR12R1.
*   DAC1 turns that number into a voltage on PA4, the amplifier drives the speaker.
* To change the note we ONLY change TIM6->ARR. DAC, DMA and table stay the same.
**********************************************************************/
#include "Audio.h"

#define SAMPLES      64U
#define TIM6_CLK_X10 900000000UL   // TIM6 clock = 90 MHz, times 10 (we use freq in 0.1 Hz)
#define AMP_PIN      6U            // PB6
#define SW_PIN       8U            // PC8

// One cycle of a sine wave, 12-bit (0..4095), centered at 2048
static const uint16_t sineTable[SAMPLES] = {
    2048, 2224, 2399, 2571, 2737, 2897, 3048, 3190,
    3321, 3439, 3545, 3635, 3711, 3770, 3813, 3839,
    3848, 3839, 3813, 3770, 3711, 3635, 3545, 3439,
    3321, 3190, 3048, 2897, 2737, 2571, 2399, 2224,
    2048, 1872, 1697, 1525, 1359, 1199, 1048, 906,
    775, 657, 551, 461, 385, 326, 283, 257,
    248, 257, 283, 326, 385, 461, 551, 657,
    775, 906, 1048, 1199, 1359, 1525, 1697, 1872,
};

static uint8_t audioOn = 0xFF;   // remembers state so we only touch hardware on a change

static void Amp_Set(uint8_t enable)
{
    uint8_t level = enable ? AMP_ENABLE_LEVEL : !AMP_ENABLE_LEVEL;
    GPIOB->BSRR = level ? (1U << AMP_PIN) : (1U << (AMP_PIN + 16));
}

void Audio_Init(void)
{
    // ---- Clocks: GPIOA (PA4), GPIOB (PB6), GPIOC (PC8), DAC, TIM6, DMA1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN;

    // ---- PA4 analog mode (11) for the DAC output
    GPIOA->MODER |= (3U << (4 * 2));

    // ---- PB6 amplifier shutdown pin: output, start with amp OFF
    Amp_Set(0);
    GPIOB->MODER &= ~(3U << (AMP_PIN * 2));
    GPIOB->MODER |=  (1U << (AMP_PIN * 2));

    // ---- PC8 slide switch: input
    GPIOC->MODER &= ~(3U << (SW_PIN * 2));
    GPIOC->PUPDR &= ~(3U << (SW_PIN * 2));

    // ---- DMA1 Stream5, channel 7 = DAC1.  Memory -> peripheral, circular, 16-bit
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) { }
    DMA1->HIFCR = (0x3DU << 6);                         // clear all Stream5 flags
    DMA1_Stream5->PAR  = (uint32_t)&DAC->DHR12R1;
    DMA1_Stream5->M0AR = (uint32_t)sineTable;
    DMA1_Stream5->NDTR = SAMPLES;
    DMA1_Stream5->CR = (7U << 25)            // CHSEL = 7 (DAC1)
                     | DMA_SxCR_PL_1         // high priority
                     | DMA_SxCR_MSIZE_0      // memory size 16 bit
                     | DMA_SxCR_PSIZE_0      // peripheral size 16 bit
                     | DMA_SxCR_MINC         // step through the table
                     | DMA_SxCR_CIRC         // wrap to start of table forever
                     | DMA_SxCR_DIR_0;       // memory -> peripheral
    DMA1_Stream5->CR |= DMA_SxCR_EN;

    // ---- TIM6: update event -> TRGO -> DAC trigger
    TIM6->PSC = 0;
    TIM6->ARR = 0xFFFF;                      // real value set by Audio_SetFrequency
    TIM6->CR2 = TIM_CR2_MMS_1;               // MMS = 010: update event is TRGO

    // ---- DAC1: trigger = TIM6 TRGO (TSEL = 000), DMA request enabled
    DAC->CR = DAC_CR_TEN1 | DAC_CR_DMAEN1;
    DAC->CR |= DAC_CR_EN1;

    Audio_SetFrequency(4400);
    Audio_Output(0);
}

void Audio_SetFrequency(uint16_t freqX10)
{
    if (freqX10 == 0) return;
    // TIM6 rate = freq * SAMPLES.  ARR = 90 MHz / (freq * SAMPLES) - 1  (rounded)
    uint32_t denom = (uint32_t)freqX10 * SAMPLES;
    TIM6->ARR = (uint16_t)((TIM6_CLK_X10 + denom / 2) / denom - 1);
    TIM6->EGR = TIM_EGR_UG;                  // load the new ARR right away
}

void Audio_Output(uint8_t on)
{
    if (on == audioOn) return;               // nothing changed
    audioOn = on;
    if (on) {
        TIM6->CR1 |= TIM_CR1_CEN;            // start sending triggers
        Amp_Set(1);
    } else {
        Amp_Set(0);                          // shut the amplifier down
        TIM6->CR1 &= ~TIM_CR1_CEN;           // stop triggers
    }
}

uint8_t Audio_SwitchOn(void)
{
    return ((GPIOC->IDR >> SW_PIN) & 1U) == SWITCH_ON_LEVEL;
}
