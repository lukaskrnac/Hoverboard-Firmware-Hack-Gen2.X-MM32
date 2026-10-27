#ifndef I2C_H
#define I2C_H

#include <stdint.h>

//Bit-banged I2C master on any two pins (pin indices of hardware.h), for the IMU: the main firmware
//(imu.c) and PinFinder (detect imu). Not reentrant: main loop only.
void I2cPins(uint8_t sda, uint8_t scl, uint8_t slow);    //open drain, released; slow: ~50 kHz instead of ~150
void I2cRelease(void);                    //both pins back to floating inputs
uint8_t I2cHigh(void);                    //both lines high (pulled up, not held by a slave)
void I2cUnstick(void);                    //9 clocks and a stop: frees a slave stuck in a transfer
uint8_t I2cProbe(uint8_t addr);           //1 = acknowledged
int16_t I2cReadReg(uint8_t addr, uint8_t reg);                     //-1: no acknowledge
uint8_t I2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n);    //1 = done
uint8_t I2cWriteReg(uint8_t addr, uint8_t reg, uint8_t v);       //1 = acknowledged
//a burst read without blocking: started, then I2cPoll() moves one byte per call
enum{ I2C_IDLE, I2C_BUSY, I2C_DONE, I2C_FAIL };
void I2cReadAsync(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n);
uint8_t I2cPoll(void);

#endif
