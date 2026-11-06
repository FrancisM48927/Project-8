/**********************************************************************
* CPEG222 Project 8 - BME280 sensor helper (source)
*
* How it works:
*  1. Find the sensor: ACK at 0x76 (or 0x77) and chip-ID register 0xD0 = 0x60.
*  2. Read the factory CALIBRATION constants stored inside the sensor
*     (registers 0x88..0xA1 and 0xE1..0xE7). Every chip has its own values.
*  3. To measure: write the control registers (forced mode = measure once),
*     wait until the sensor is done, then read the raw ADC numbers
*     (registers 0xF7..0xFE: pressure, temperature, humidity).
*  4. The raw numbers mean nothing alone. Bosch's compensation formulas
*     combine each raw number with the calibration constants to give real units.
*     (Pressure and humidity also use "t_fine" from the temperature step.)
**********************************************************************/
#include "BME280.h"
#include "I2C.h"

#define REG_ID        0xD0
#define REG_RESET     0xE0
#define REG_CTRL_HUM  0xF2
#define REG_STATUS    0xF3
#define REG_CTRL_MEAS 0xF4
#define REG_DATA      0xF7

#define CHIP_ID_BME280 0x60

static uint8_t devAddr = 0x76;

// Calibration constants (read once from the sensor)
static uint16_t T1;  static int16_t T2, T3;
static uint16_t P1;  static int16_t P2, P3, P4, P5, P6, P7, P8, P9;
static uint8_t  H1, H3;  static int16_t H2, H4, H5;  static int8_t H6;
static int32_t  t_fine;

static void delay_rough(uint32_t n) { for (volatile uint32_t i = 0; i < n; i++) { } }

static uint8_t ReadBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        if (!I2C_ReadReg(devAddr, (uint8_t)(reg + i), &buf[i])) return 0;
    }
    return 1;
}

uint8_t BME280_Init(void)
{
    // ---- Is the sensor there? Try both possible addresses.
    if (I2C_Probe(0x76))      devAddr = 0x76;
    else if (I2C_Probe(0x77)) devAddr = 0x77;
    else return 0;

    uint8_t id = 0;
    if (!I2C_ReadReg(devAddr, REG_ID, &id) || id != CHIP_ID_BME280) return 0;

    // ---- Soft reset, then wait for it to finish copying its calibration data
    I2C_WriteReg(devAddr, REG_RESET, 0xB6);
    delay_rough(400000);                           // ~10 ms
    uint8_t st = 1;
    for (uint8_t tries = 0; tries < 100; tries++) {
        if (I2C_ReadReg(devAddr, REG_STATUS, &st) && !(st & 0x01)) break;
    }

    // ---- Read calibration constants
    uint8_t c1[26], c2[7];
    if (!ReadBytes(0x88, c1, 26)) return 0;
    if (!ReadBytes(0xE1, c2, 7))  return 0;

    T1 = (uint16_t)(c1[0] | (c1[1] << 8));
    T2 = (int16_t)(c1[2] | (c1[3] << 8));
    T3 = (int16_t)(c1[4] | (c1[5] << 8));
    P1 = (uint16_t)(c1[6] | (c1[7] << 8));
    P2 = (int16_t)(c1[8]  | (c1[9] << 8));
    P3 = (int16_t)(c1[10] | (c1[11] << 8));
    P4 = (int16_t)(c1[12] | (c1[13] << 8));
    P5 = (int16_t)(c1[14] | (c1[15] << 8));
    P6 = (int16_t)(c1[16] | (c1[17] << 8));
    P7 = (int16_t)(c1[18] | (c1[19] << 8));
    P8 = (int16_t)(c1[20] | (c1[21] << 8));
    P9 = (int16_t)(c1[22] | (c1[23] << 8));
    H1 = c1[25];                                           // register 0xA1
    H2 = (int16_t)(c2[0] | (c2[1] << 8));                  // 0xE1, 0xE2
    H3 = c2[2];                                            // 0xE3
    H4 = (int16_t)(((int8_t)c2[3] * 16) | (c2[4] & 0x0F)); // 0xE4, 0xE5 low nibble
    H5 = (int16_t)(((int8_t)c2[5] * 16) | (c2[4] >> 4));   // 0xE6, 0xE5 high nibble
    H6 = (int8_t)c2[6];                                    // 0xE7
    return 1;
}

// ---- Bosch compensation formulas (integer versions from the datasheet) ----

static int32_t comp_temp(int32_t adc_T)               // returns hundredths of deg C
{
    int32_t v1 = ((((adc_T >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
    int32_t v2 = (((((adc_T >> 4) - ((int32_t)T1)) *
                    ((adc_T >> 4) - ((int32_t)T1))) >> 12) * ((int32_t)T3)) >> 14;
    t_fine = v1 + v2;
    return (t_fine * 5 + 128) >> 8;
}

static uint32_t comp_press(int32_t adc_P)             // returns Pa x 256
{
    int64_t v1, v2, p;
    v1 = ((int64_t)t_fine) - 128000;
    v2 = v1 * v1 * (int64_t)P6;
    v2 = v2 + ((v1 * (int64_t)P5) << 17);
    v2 = v2 + (((int64_t)P4) << 35);
    v1 = ((v1 * v1 * (int64_t)P3) >> 8) + ((v1 * (int64_t)P2) << 12);
    v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)P1) >> 33;
    if (v1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = (((int64_t)P9) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (((int64_t)P8) * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)P7) << 4);
    return (uint32_t)p;
}

static uint32_t comp_hum(int32_t adc_H)               // returns %RH x 1024
{
    int32_t v = t_fine - 76800;
    v = (((((adc_H << 14) - (((int32_t)H4) << 20) - (((int32_t)H5) * v)) +
           ((int32_t)16384)) >> 15) *
         (((((((v * ((int32_t)H6)) >> 10) *
              (((v * ((int32_t)H3)) >> 11) + ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)H2) + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)H1)) >> 4);
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12);
}

uint8_t BME280_Read(BME280_Data *d)
{
    // Humidity setting must be written BEFORE ctrl_meas to take effect.
    if (!I2C_WriteReg(devAddr, REG_CTRL_HUM, 0x01)) return 0;   // humidity x1
    if (!I2C_WriteReg(devAddr, REG_CTRL_MEAS, 0x25)) return 0;  // temp x1, press x1, FORCED mode

    // Wait until the "measuring" bit (bit 3 of status) clears
    uint8_t st = 0x08;
    for (uint8_t tries = 0; tries < 200; tries++) {
        if (!I2C_ReadReg(devAddr, REG_STATUS, &st)) return 0;
        if (!(st & 0x08)) break;
    }

    uint8_t b[8];
    if (!ReadBytes(REG_DATA, b, 8)) return 0;

    int32_t adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    int32_t adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
    int32_t adc_H = ((int32_t)b[6] << 8)  |  b[7];

    d->tempC100    = comp_temp(adc_T);                  // must run first: sets t_fine
    d->pressHpa100 = comp_press(adc_P) / 256;           // Pa = hundredths of a hPa
    d->hum100      = (uint32_t)(((uint64_t)comp_hum(adc_H) * 100U) / 1024U);
    return 1;
}
