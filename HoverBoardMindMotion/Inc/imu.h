#ifndef IMU_H
#define IMU_H

#include <stdint.h>

//IMU with the InvenSense register map (MPU-6050 / 6500 / 9250, ICM-206xx and clones; I2C 0x68 / 0x69) on
//the configured pins (imusda, imuscl, imuaddr: PinFinder detect imu). imu.c
enum{ IMU_OFF = 0, IMU_RUNNING = 1, IMU_NO_ANSWER = 2 };
extern volatile uint8_t imuState;
extern volatile int16_t imuRaw[7];    //accel x y z, temperature, gyro x y z (chip units, ranges below)

void ImuInit(void);     //at boot, after the pins: wakes and sets up the chip
void ImuPoll(void);     //main loop: a reading every 10 ms
//k 0-2 accel m/s^2, 3-5 gyro deg/s (board axes after calibrate imu, X forward Y left Z up), 6 temperature C,
//7 pitch, 8 roll in degrees (front down / right side down positive); NaN unless running
float ImuValue(uint8_t k);

#endif
