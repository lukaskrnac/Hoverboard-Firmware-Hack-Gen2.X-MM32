#ifndef IMUMATH_H
#define IMUMATH_H

#include <stdint.h>

//IMU mounting (config word imumount, PinFinder calibrate imu) and the angle math, shared by the main
//firmware (imu.c) and PinFinder. Board axes as ROS: X forward (positive wheel speed), Y left (the axle),
//Z up. Chip axis codes 0..5 = +x +y +z -x -y -z; imumount = 1 + up*6 + forward, 0/65535 = not calibrated.
uint8_t ImuAxes(uint16_t mount, int8_t* ax);    //ax[0..2]: the chip axis code of board X, Y, Z; 0 = not set
void ImuToBoard(const int8_t* ax, const int16_t* chip, float* board);    //a chip vector in board axes
float Atan2Deg(float y, float x);    //degrees, -180..180, within ~0.1 deg

#endif
