/**********************************************************************
* CPEG222 Project 7 - NeoPixel helper (header)
* 4 chained NeoPixels (D9..D12) on PB11 = TIM2_CH4 (AF1), PWM + DMA
**********************************************************************/
#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include "stm32f4xx.h"
#include <stdint.h>

void NeoPixel_Init(void);
// Set all four NeoPixels to the same RGB color and send it (blocks ~0.2 ms)
void NeoPixel_ShowAll(uint8_t r, uint8_t g, uint8_t b);

#endif
