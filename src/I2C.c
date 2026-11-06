/**********************************************************************
* CPEG222 Project 8 - I2C1 helper (source)
*
* I2C is a 2-wire bus:  SCL = clock (master drives), SDA = data (both drive).
* Both lines are open-drain with pull-ups, so a device only pulls them LOW.
* A transfer = START, 7-bit address + R/W bit, the slave ACKs (pulls SDA low),
* data bytes (each ACKed), then STOP.
*
* Reading a register uses a "repeated START":
*   START, addr+W, register number, START again, addr+R, read byte, STOP.
* We read ONE byte per transaction (simple and safe; the BME280 is in forced
* mode so its data does not change while we read it).
**********************************************************************/
#include "I2C.h"

#define SCL_PIN      8U            // PB8
#define SDA_PIN      9U            // PB9
#define I2C_TIMEOUT  100000U

static void I2C_Config(void)
{
    I2C1->CR1  = 0;                       // peripheral off while configuring
    I2C1->CR2  = 45;                      // APB1 clock = 45 MHz
    I2C1->CCR  = 225;                     // standard mode 100 kHz: 45 MHz / (2 x 100 kHz)
    I2C1->TRISE = 46;                     // max rise time 1000 ns -> 45 + 1
    I2C1->CR1  = I2C_CR1_PE;              // turn the peripheral on
}

static void I2C_Recover(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 |= I2C_CR1_SWRST;           // software reset clears a stuck BUSY flag
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    I2C_Config();
}

// Wait until the SR1 flag(s) in mask are set. Fails on NACK (AF) or timeout.
static uint8_t wait_sr1(uint32_t mask)
{
    uint32_t t = I2C_TIMEOUT;
    while (!(I2C1->SR1 & mask)) {
        if (I2C1->SR1 & I2C_SR1_AF) return 0;     // slave did not acknowledge
        if (--t == 0) return 0;
    }
    return 1;
}

static uint8_t wait_bus_free(void)
{
    uint32_t t = I2C_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) || (I2C1->CR1 & I2C_CR1_STOP)) {
        if (--t == 0) return 0;
    }
    return 1;
}

static void clear_addr(void)
{
    (void)I2C1->SR1;                      // reading SR1 then SR2 clears ADDR
    (void)I2C1->SR2;
}

void I2C_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB8, PB9: alternate function, open-drain, pull-up, AF4 = I2C1
    GPIOB->MODER   &= ~((3U << (SCL_PIN * 2)) | (3U << (SDA_PIN * 2)));
    GPIOB->MODER   |=  ((2U << (SCL_PIN * 2)) | (2U << (SDA_PIN * 2)));
    GPIOB->OTYPER  |=  (1U << SCL_PIN) | (1U << SDA_PIN);
    GPIOB->OSPEEDR |=  ((3U << (SCL_PIN * 2)) | (3U << (SDA_PIN * 2)));
    GPIOB->PUPDR   &= ~((3U << (SCL_PIN * 2)) | (3U << (SDA_PIN * 2)));
    GPIOB->PUPDR   |=  ((1U << (SCL_PIN * 2)) | (1U << (SDA_PIN * 2)));
    GPIOB->AFR[1]  &= ~((0xFU << ((SCL_PIN - 8) * 4)) | (0xFU << ((SDA_PIN - 8) * 4)));
    GPIOB->AFR[1]  |=  ((4U   << ((SCL_PIN - 8) * 4)) | (4U   << ((SDA_PIN - 8) * 4)));

    I2C1->CR1 = I2C_CR1_SWRST;            // reset, then release
    I2C1->CR1 = 0;
    I2C_Config();
}

// "Is anything there?"  Send START + address (write). A present device ACKs.
uint8_t I2C_Probe(uint8_t addr)
{
    uint8_t ok = 0;
    if (!wait_bus_free()) { I2C_Recover(); }

    I2C1->CR1 |= I2C_CR1_START;
    if (wait_sr1(I2C_SR1_SB)) {
        I2C1->DR = (uint8_t)(addr << 1);
        if (wait_sr1(I2C_SR1_ADDR)) {     // ADDR set = the device ACKed
            ok = 1;
            clear_addr();
        }
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->SR1 &= ~I2C_SR1_AF;             // clear a NACK flag if there was one
    wait_bus_free();
    return ok;
}

uint8_t I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val)
{
    if (!wait_bus_free()) { I2C_Recover(); }

    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB)) goto fail;
    I2C1->DR = (uint8_t)(addr << 1);                  // address + WRITE
    if (!wait_sr1(I2C_SR1_ADDR)) goto fail;
    clear_addr();
    if (!wait_sr1(I2C_SR1_TXE)) goto fail;
    I2C1->DR = reg;                                   // which register
    if (!wait_sr1(I2C_SR1_TXE)) goto fail;
    I2C1->DR = val;                                   // the new value
    if (!wait_sr1(I2C_SR1_BTF)) goto fail;
    I2C1->CR1 |= I2C_CR1_STOP;
    wait_bus_free();
    return 1;

fail:
    I2C_Recover();
    return 0;
}

uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    if (!wait_bus_free()) { I2C_Recover(); }

    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB)) goto fail;
    I2C1->DR = (uint8_t)(addr << 1);                  // address + WRITE
    if (!wait_sr1(I2C_SR1_ADDR)) goto fail;
    clear_addr();
    if (!wait_sr1(I2C_SR1_TXE)) goto fail;
    I2C1->DR = reg;                                   // which register to read
    if (!wait_sr1(I2C_SR1_BTF)) goto fail;

    I2C1->CR1 |= I2C_CR1_START;                       // repeated START
    if (!wait_sr1(I2C_SR1_SB)) goto fail;
    I2C1->DR = (uint8_t)((addr << 1) | 1U);           // address + READ
    if (!wait_sr1(I2C_SR1_ADDR)) goto fail;

    I2C1->CR1 &= ~I2C_CR1_ACK;                        // one byte only: NACK it
    clear_addr();
    I2C1->CR1 |= I2C_CR1_STOP;                        // STOP after that byte
    if (!wait_sr1(I2C_SR1_RXNE)) goto fail;
    *val = (uint8_t)I2C1->DR;
    wait_bus_free();
    return 1;

fail:
    I2C_Recover();
    return 0;
}
