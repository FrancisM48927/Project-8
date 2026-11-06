/**********************************************************************
Author: F. MAILOM
CPEG222 Project 8, 11/5/26
NucleoF466ZE CMSIS Read Microphone and Display Amplitude 
**********************************************************************/
#include "stm32f4xx.h"
#include "SysClock180MHz.h"
#include "ssd.h"
#include "OLED.h"
#include "I2C.h"
#include "BME280.h"
#include "NeoPixel.h"
#include "Audio.h"

#define READ_PERIOD_MS  1000U      // one reading per second

// ------------------------- Shared with interrupt handlers -------------------------
static volatile uint32_t sysMs    = 0;   // free-running millisecond clock
static volatile uint32_t msCount  = 0;   // counts 1 ms ticks up to 1000
static volatile uint8_t  readFlag = 0;   // set once per second, cleared by main()

// SysTick fires every 1 ms. A single SysTick can only count ~93 ms at 180 MHz,
// so we count 1000 ticks in software to get 1 second.
void SysTick_Handler(void)
{
    sysMs++;
    if (++msCount >= READ_PERIOD_MS) {
        msCount  = 0;
        readFlag = 1;
    }
}

// TIM5: refresh one SSD digit per interrupt
void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;
        SSD_Refresh();
    }
}

// ------------------------- State -------------------------
static BME280_Data meas;
static uint8_t     haveData  = 0;
static uint8_t     alarmOn   = 0;

// ------------------------- Small helpers -------------------------
static void Switch_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3U << (8 * 2));            // PC8 input
}

static uint8_t Switch_IsCelsius(void)
{
    return (GPIOC->IDR >> 8) & 1U;               // S1 = 1 -> Celsius
}

static void LEDs_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER = (GPIOD->MODER & 0xFFFF0000U) | 0x00005555U;   // PD0..PD7 outputs
    GPIOD->ODR  &= ~0xFFU;
}

// One LED per 12.5 % RH:  count = floor(RH / 12.5) = floor(RH x 8 / 100)
static void LEDs_ShowHumidity(uint32_t hum100)
{
    uint32_t count = (hum100 * 8U) / 10000U;     // 38.00 % -> 3
    if (count > 8) count = 8;
    uint32_t mask = (1U << count) - 1U;          // 3 -> 0b00000111
    GPIOD->ODR = (GPIOD->ODR & ~0xFFU) | mask;
}

// Print "label" + number with 2 decimals + "unit", padded to 20 characters
static void PrintLine(uint8_t page, const char *label, int32_t v100, const char *unit)
{
    char buf[26];
    char *p = buf;
    while (*label) *p++ = *label++;
    if (v100 < 0) { *p++ = '-'; v100 = -v100; }
    uint32_t ip = (uint32_t)v100 / 100U, fp = (uint32_t)v100 % 100U;
    char tmp[10];
    int n = 0;
    do { tmp[n++] = (char)('0' + ip % 10U); ip /= 10U; } while (ip);
    while (n) *p++ = tmp[--n];
    *p++ = '.';
    *p++ = (char)('0' + fp / 10U);
    *p++ = (char)('0' + fp % 10U);
    while (*unit) *p++ = *unit++;
    while (p < buf + 20) *p++ = ' ';
    *p = '\0';
    OLED_Print(page, 0, buf);
}

static int32_t TempF100(void)                    // F = C x 9/5 + 32
{
    return (meas.tempC100 * 9) / 5 + 3200;
}

// Temperature on the OLED and SSD, in the unit chosen by S1
static void ShowTemperature(void)
{
    int32_t shown;
    if (Switch_IsCelsius()) {
        shown = meas.tempC100;
        PrintLine(2, "Temp: ", shown, " C");
    } else {
        shown = TempF100();
        PrintLine(2, "Temp: ", shown, " F");
    }
    if (shown < 0) shown = 0;                    // SSD has no minus sign
    if (shown > 9999) shown = 9999;
    SSD_DisplayValue((uint16_t)shown);           // xx.xx
}

static void ShowTempColor(void)
{
    int32_t f = TempF100();
    if (f < 7000)       NeoPixel_ShowAll(0, 0, 255);   // below 70 F: blue
    else if (f <= 8000) NeoPixel_ShowAll(0, 255, 0);   // 70..80 F: green
    else                NeoPixel_ShowAll(255, 0, 0);   // above 80 F: red
}

// ------------------------- Once per second -------------------------
static void UpdateReading(void)
{
    static uint8_t sensorOk = 0;

    if (!sensorOk) {
        sensorOk = BME280_Init();
        if (!sensorOk) {
            OLED_Print(2, 0, "BME280 not found    ");
            return;
        }
    }
    if (!BME280_Read(&meas)) {
        sensorOk = 0;                            // try a full re-init next second
        OLED_Print(2, 0, "BME280 read error   ");
        return;
    }
    haveData = 1;

    ShowTemperature();
    PrintLine(4, "Press: ", (int32_t)meas.pressHpa100, " hPa");
    PrintLine(6, "Hum: ",   (int32_t)meas.hum100, " %RH");
    LEDs_ShowHumidity(meas.hum100);

    // Bonus condition: hotter than 80 F or more humid than 50 %
    uint8_t nowAlarm = (TempF100() > 8000) || (meas.hum100 > 5000);
    if (nowAlarm && !alarmOn) {
        Audio_Output(1);                         // start siren
    } else if (!nowAlarm && alarmOn) {
        Audio_Output(0);                         // condition ended: siren off
    }
    alarmOn = nowAlarm;

    if (!alarmOn) ShowTempColor();
}

// Runs in the main loop while the alarm is on: flash blue/red, sweep the siren
static void AlarmService(void)
{
    static uint32_t lastFlash = 0, lastSweep = 0;
    static uint8_t  red = 0;
    static int32_t  freqX10 = 6000;              // 600.0 Hz
    static int32_t  dir = 100;                   // +10 Hz per step

    if ((sysMs - lastFlash) >= 250U) {           // flash 4 times a second
        lastFlash = sysMs;
        red ^= 1U;
        if (red) NeoPixel_ShowAll(255, 0, 0);
        else     NeoPixel_ShowAll(0, 0, 255);
    }
    if ((sysMs - lastSweep) >= 10U) {            // siren: sweep 600..1200 Hz
        lastSweep = sysMs;
        freqX10 += dir;
        if (freqX10 >= 12000) dir = -100;
        if (freqX10 <= 6000)  dir = 100;
        Audio_SetFrequency((uint16_t)freqX10);
    }
}

// ------------------------- main -------------------------
int main(void)
{
    SystemClock_Config();                        // 16 MHz -> 180 MHz
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);      // 1 ms tick

    SSD_Init();
    OLED_Init();
    I2C_Init();
    NeoPixel_Init();
    Audio_Init();                                // siren hardware (bonus), starts silent
    Switch_Init();
    LEDs_Init();

    OLED_Print(0, 0, "CPEG222-F26 Project8");
    SSD_DisplayValue(0);
    NeoPixel_ShowAll(0, 0, 0);

    uint8_t lastUnit = Switch_IsCelsius();
    readFlag = 1;                                // first reading right away

    while (1)
    {
        if (readFlag) {                          // set by SysTick once per second
            readFlag = 0;
            UpdateReading();
        }

        // Flipping S1 changes the units immediately (no need to wait a second)
        uint8_t unit = Switch_IsCelsius();
        if (unit != lastUnit) {
            lastUnit = unit;
            if (haveData) ShowTemperature();
        }

        if (alarmOn) AlarmService();
    }
}
