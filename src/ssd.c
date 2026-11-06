/**********************************************************************
* Author: F. MAILOM
* CPEG222 Project 3, 10/1/26
* NucleoF466ZE CMSIS SSD array helper (source)
**********************************************************************/
#include "ssd.h"

// Number of digits in the SSD array
#define SSD_DIGITS 4

// Refresh rate of each digit (Hz). TIM5 fires SSD_DIGITS times faster
// so that one digit is refreshed per interrupt
#define SSD_REFRESH_HZ 100

// Segment bit positions in a pattern (1 = segment ON)
#define SEG_A  (1 << 0)
#define SEG_B  (1 << 1)
#define SEG_C  (1 << 2)
#define SEG_D  (1 << 3)
#define SEG_E  (1 << 4)
#define SEG_F  (1 << 5)
#define SEG_G  (1 << 6)
#define SEG_DP (1 << 7)

// Index of the blank entry in the digitSegments array
#define SSD_BLANK 10

// Segment patterns for 0-9 and blank (1 = segment ON, bit0 = A ... bit6 = G)
const unsigned char digitSegments[] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111, // 9
    0b0000000  // blank/off
};

// Segment pattern for each digit (including DP), updated by SSD_DisplayValue
volatile uint8_t SSD_BUFFER[SSD_DIGITS] = {0, 0, 0, 0};
// Digit that will be refreshed on the next TIM5 interrupt
volatile uint8_t SSD_DIGIT = 0;

// Build a BSRR value for one pin: segment ON -> pin LOW, OFF -> pin HIGH
// (the SSD array is common anode so a segment needs to be LOW to be ON)
static uint32_t Segment_Bsrr(uint8_t on, uint8_t pin)
{
    return on ? (1U << (pin + 16)) : (1U << pin);
}

// Write a segment pattern (1 = ON) to the segment pins
static void Write_Segments(uint8_t pattern)
{
    // A = PG9, D = PG14
    GPIOG->BSRR = Segment_Bsrr(pattern & SEG_A, 9) |
                  Segment_Bsrr(pattern & SEG_D, 14);
    // B = PF12, C = PF13, DP = PF14, F = PF15
    GPIOF->BSRR = Segment_Bsrr(pattern & SEG_B, 12) |
                  Segment_Bsrr(pattern & SEG_C, 13) |
                  Segment_Bsrr(pattern & SEG_DP, 14) |
                  Segment_Bsrr(pattern & SEG_F, 15);
    // E = PE8
    GPIOE->BSRR = Segment_Bsrr(pattern & SEG_E, 8);
    // G = PB4
    GPIOB->BSRR = Segment_Bsrr(pattern & SEG_G, 4);
}

// Turn off all four digit select transistors (base HIGH = digit OFF)
static void Digits_Off(void)
{
    GPIOE->BSRR = (1U << 10) | (1U << 7); // CA1 = PE10, CA2 = PE7
    GPIOB->BSRR = (1U << 5) | (1U << 3);  // CA3 = PB5,  CA4 = PB3
}

// Turn on one digit select transistor (base LOW = digit ON)
static void Digit_On(uint8_t digit)
{
    switch (digit)
    {
        case 0: GPIOE->BSRR = (1U << (10 + 16)); break; // CA1
        case 1: GPIOE->BSRR = (1U << (7 + 16));  break; // CA2
        case 2: GPIOB->BSRR = (1U << (5 + 16));  break; // CA3
        case 3: GPIOB->BSRR = (1U << (3 + 16));  break; // CA4
    }
}

void SSD_Init(void)
{
    // Enable clock for GPIOB, GPIOE, GPIOF, and GPIOG
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOEEN |
                    RCC_AHB1ENR_GPIOFEN |
                    RCC_AHB1ENR_GPIOGEN;
    // Enable TIM5 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    // Start with every digit and segment OFF (all pins HIGH)
    Digits_Off();
    Write_Segments(0x00);

    // Configure digit select pins PE10, PE7, PB5, PB3 as output
    GPIOE->MODER &= ~((3 << (10 * 2)) | (3 << (7 * 2)));
    GPIOE->MODER |=  ((1 << (10 * 2)) | (1 << (7 * 2)));
    GPIOB->MODER &= ~((3 << (5 * 2)) | (3 << (3 * 2)));
    GPIOB->MODER |=  ((1 << (5 * 2)) | (1 << (3 * 2)));

    // Configure segment pins as output
    // A = PG9, D = PG14
    GPIOG->MODER &= ~((3 << (9 * 2)) | (3 << (14 * 2)));
    GPIOG->MODER |=  ((1 << (9 * 2)) | (1 << (14 * 2)));
    // B = PF12, C = PF13, DP = PF14, F = PF15
    GPIOF->MODER &= ~((3 << (12 * 2)) | (3 << (13 * 2)) |
                      (3 << (14 * 2)) | (3 << (15 * 2)));
    GPIOF->MODER |=  ((1 << (12 * 2)) | (1 << (13 * 2)) |
                      (1 << (14 * 2)) | (1 << (15 * 2)));
    // E = PE8
    GPIOE->MODER &= ~(3 << (8 * 2));
    GPIOE->MODER |=  (1 << (8 * 2));
    // G = PB4
    GPIOB->MODER &= ~(3 << (4 * 2));
    GPIOB->MODER |=  (1 << (4 * 2));

    // Configure TIM5 to interrupt SSD_DIGITS * SSD_REFRESH_HZ times a second
    // (one digit per interrupt, each digit refreshed every 10 ms)
    TIM5->PSC = 0;
    // TIM5 is on APB1: with the 180 MHz clock its timer clock is 90 MHz (SystemCoreClock / 2)
    TIM5->ARR = ((SystemCoreClock / 2) / (SSD_DIGITS * SSD_REFRESH_HZ)) - 1;
    TIM5->EGR = (1 << 0);   // Force an update event to load the registers
    TIM5->SR = 0;           // Clear the flag set by the update event above
    TIM5->DIER |= (1 << 0); // Enable update interrupt

    // Enable TIM5 interrupt in NVIC (display refresh gets highest priority)
    NVIC_SetPriority(TIM5_IRQn, 1);
    NVIC_EnableIRQ(TIM5_IRQn);

    // Start TIM5
    TIM5->CR1 |= (1 << 0);
}

void SSD_DisplayValue(uint16_t hundredths)
{
    if (hundredths > 9999)
    {
        hundredths = 9999;
    }

    // Split the value into digits (tens, ones, tenths, hundredths of a second)
    uint8_t tens       = (hundredths / 1000) % 10;
    uint8_t ones       = (hundredths / 100) % 10;
    uint8_t tenths     = (hundredths / 10) % 10;
    uint8_t hundredth  = hundredths % 10;

    // Suppress the leading zero for values under 10.00
    SSD_BUFFER[0] = digitSegments[(tens == 0) ? SSD_BLANK : tens];
    // Decimal point goes after the second digit
    SSD_BUFFER[1] = digitSegments[ones] | SEG_DP;
    SSD_BUFFER[2] = digitSegments[tenths];
    SSD_BUFFER[3] = digitSegments[hundredth];
}

// Show a frequency in xxx.x format. Input is in tenths of Hz (4400 = 440.0 Hz).
// Decimal point is after the 3rd digit; leading zeros are blanked (0 shows "  0.0").
void SSD_DisplayFrequency(uint16_t tenthsHz)
{
    if (tenthsHz > 9999) tenthsHz = 9999;

    uint8_t hundreds = (tenthsHz / 1000) % 10;
    uint8_t tens     = (tenthsHz / 100) % 10;
    uint8_t ones     = (tenthsHz / 10) % 10;
    uint8_t tenth    = tenthsHz % 10;

    SSD_BUFFER[0] = digitSegments[(tenthsHz < 1000) ? SSD_BLANK : hundreds];
    SSD_BUFFER[1] = digitSegments[(tenthsHz < 100)  ? SSD_BLANK : tens];
    SSD_BUFFER[2] = digitSegments[ones] | SEG_DP;   // DP after the third digit
    SSD_BUFFER[3] = digitSegments[tenth];
}

void SSD_Refresh(void)
{
    // Turn everything off first to prevent ghosting between digits
    Digits_Off();
    // Output this digit's segments, then turn the digit on
    Write_Segments(SSD_BUFFER[SSD_DIGIT]);
    Digit_On(SSD_DIGIT);

    // Move on to the next digit
    SSD_DIGIT = (SSD_DIGIT + 1) % SSD_DIGITS;
}