#ifndef IMUMATH_H
#define IMUMATH_H

#include <stdint.h>

//IMU mounting (config word imumount, PinFinder calibrate imu) and the angle math, shared by the main
//firmware (imu.c) and PinFinder. Board axes as ROS: X forward (positive wheel speed), Y left (the axle),
//Z up. Chip axis codes 0..5 = +x +y +z -x -y -z; imumount = 1 + up*6 + forward, 0/65535 = not calibrated.
uint8_t ImuAxes(uint16_t mount, int8_t* ax);    //ax[0..2]: the chip axis code of board X, Y, Z; 0 = not set
void ImuToBoard(const int8_t* ax, const int16_t* chip, float* board);    //a chip vector in board axes
float Atan2Deg(float y, float x);    //degrees, -180..180, within ~0.1 deg

//register maps of the IMUs that are read (imumath.c), chosen by PinFinder's imuaddr / imuid. All set to
//+-4 g (8192 LSB per g) and ~1000 deg/s; raw[7] = accel x y z, temperature, gyro x y z (chip axes;
//ImuParse scales the gyro to 32.8 LSB per deg/s).
typedef struct{
	uint8_t ids[3];       //id register values (0: the InvenSense default at 0x68 / 0x69)
	uint8_t data, n;      //burst: start register, bytes
	uint8_t oa, og, ot;   //byte offsets of accel x, gyro x, temperature in it (ot 0xFF: none)
	uint8_t le;           //1: little endian
	uint8_t chk, run;     //register read back once a second, its value while running
	uint8_t glsb4;        //gyro LSB per deg/s x 4
	uint16_t tdiv;        //temperature C = raw x 100 / tdiv + t0 / 100
	int16_t t0;
	uint8_t init[14];     //register, value pairs, 0 = the end; register 0xFF: wait value ms
} ImuMap;
const ImuMap* ImuMapFor(uint16_t addr, uint16_t id);    //0: not supported (BMI270: needs an 8 KB config upload)
uint8_t ImuSetup(const ImuMap* m, uint8_t addr);        //1 = every write acknowledged
void ImuParse(const ImuMap* m, const uint8_t* b, int16_t* raw);

#endif
