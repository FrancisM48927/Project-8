/**********************************************************************
* CPEG222 Project 8 - BME280 sensor helper (header)
* Temperature, pressure and humidity over I2C. Address 0x76 or 0x77.
**********************************************************************/
#ifndef BME280_H
#define BME280_H

#include <stdint.h>

typedef struct {
    int32_t  tempC100;       // temperature in hundredths of a degree C (2534 = 25.34 C)
    uint32_t pressHpa100;    // pressure in hundredths of a hPa        (101325 = 1013.25 hPa)
    uint32_t hum100;         // relative humidity in hundredths of %   (4997 = 49.97 %RH)
} BME280_Data;

uint8_t BME280_Init(void);                 // 1 = sensor found and calibration loaded
uint8_t BME280_Read(BME280_Data *data);    // 1 = new reading stored in *data

#endif
