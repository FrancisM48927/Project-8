/**********************************************************************
* CPEG222 Project 7 - Audio helper (header)
* DAC1 (PA4) + DMA1 Stream5 + TIM6 sine-table player (from Project 4)
* Audio amplifier shutdown pin = PB6, Slide switch S1 = PC8
**********************************************************************/
#ifndef AUDIO_H
#define AUDIO_H

#include "stm32f4xx.h"
#include <stdint.h>

// ---- Polarity settings: set these to match what you used in Project 4 ----
#define AMP_ENABLE_LEVEL   1   // PB6 level that turns the amplifier ON (1 = HIGH)
#define SWITCH_ON_LEVEL    1   // PC8 level when S1 is in the "audio enabled" position

void    Audio_Init(void);                      // DAC, DMA, TIM6, PB6 amp pin, PC8 switch
void    Audio_SetFrequency(uint16_t freqX10);  // frequency in 0.1 Hz units (4400 = 440.0 Hz)
void    Audio_Output(uint8_t on);              // 1 = tone + amp on, 0 = silent (amp shut down)
uint8_t Audio_SwitchOn(void);                  // 1 when S1 enables audio

#endif
