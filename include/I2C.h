/**********************************************************************
* CPEG222 Project 8 - I2C1 helper (header)
* I2C1 master, 100 kHz, SCL = PB8, SDA = PB9 (AF4)
* All functions return 1 on success, 0 on failure (NACK / timeout).
**********************************************************************/
#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"
#include <stdint.h>

void    I2C_Init(void);
uint8_t I2C_Probe(uint8_t addr);                              // 1 if a device ACKs this 7-bit address
uint8_t I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val); // write one register
uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *val); // read one register

#endif
