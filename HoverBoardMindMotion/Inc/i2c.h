#ifndef I2C_H
#define I2C_H

#include <stdint.h>

//I2C master for the IMU (pin indices of hardware.h). Main firmware: the I2C1 peripheral by DMA (i2chw.c),
//the pins must be I2C1's. PinFinder: bit-banged on any two pins (i2c.c, detect imu searches them). Not
//reentrant: main loop only.
uint8_t I2cPins(uint8_t sda, uint8_t scl, uint8_t slow);    //0: not I2C1 pins (i2chw.c); slow: ~50 kHz (i2c.c)
void I2cRelease(void);                    //both pins back to floating inputs (i2c.c)
uint8_t I2cHigh(void);                    //both lines high (pulled up, not held by a slave) (i2c.c)
void I2cUnstick(void);                    //9 clocks and a stop: frees a slave stuck in a transfer
uint8_t I2cProbe(uint8_t addr);           //1 = acknowledged (i2c.c)
int16_t I2cReadReg(uint8_t addr, uint8_t reg);                     //-1: no acknowledge
uint8_t I2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n);    //1 = done
uint8_t I2cWriteReg(uint8_t addr, uint8_t reg, uint8_t v);       //1 = acknowledged
//a burst read without blocking: started, then I2cPoll() moves one byte per call
enum{ I2C_IDLE, I2C_BUSY, I2C_DONE, I2C_FAIL };
void I2cReadAsync(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n);
uint8_t I2cPoll(void);

#endif
