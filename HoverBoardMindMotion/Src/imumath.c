//IMU mounting and angle math (imumath.h)
#include "imumath.h"
#include "i2c.h"

extern volatile uint32_t millis;

//InvenSense MPU-6050/6500/9250, ICM-206xx and clones (the default at 0x68 / 0x69), ICM-42605/42688,
//BMI160, LSM6DS3/DS3TR-C/DSO (0x6A / 0x6B), QMI8658 (0x6A / 0x6B). Datasheet register maps; only the
//InvenSense one tried on hardware (the bench board's MPU-6050 clone).
static const ImuMap maps[] = {
	{ { 0 }, 0x3B, 14, 0, 8, 6, 0, 0x6B, 0x01, 131, 34000, 3653,    //PWR_MGMT_1 run, 44 Hz low pass, 1000 dps, 4 g
	  { 0x6B, 0x01, 0x6C, 0x00, 0x1A, 0x03, 0x1B, 0x10, 0x1C, 0x08 } },
	{ { 0x42, 0x47 }, 0x1D, 14, 2, 8, 0, 0, 0x4E, 0x0F, 131, 13248, 2500,    //PWR_MGMT0 low noise, 1 kHz
	  { 0x4E, 0x0F, 0xFF, 1, 0x4F, 0x26, 0x50, 0x46 } },
	{ { 0xD1 }, 0x0C, 12, 6, 0, 0xFF, 1, 0x03, 0x14, 131, 1, 0,    //CMD acc / gyro normal mode, PMU_STATUS
	  { 0x7E, 0x11, 0xFF, 5, 0x7E, 0x15, 0xFF, 81, 0x41, 0x05, 0x43, 0x01 } },
	{ { 0x69, 0x6A, 0x6C }, 0x20, 14, 8, 2, 0, 1, 0x10, 0x48, 114, 25600, 2500,    //BDU + auto increment, 104 Hz
	  { 0x12, 0x44, 0x10, 0x48, 0x11, 0x48 } },
	{ { 0x05 }, 0x33, 14, 2, 8, 0, 1, 0x08, 0x03, 128, 25600, 0,    //auto increment little endian, 1024 dps
	  { 0x02, 0x40, 0x03, 0x13, 0x04, 0x63, 0x08, 0x03 } },
};

const ImuMap* ImuMapFor(uint16_t addr, uint16_t id){
	uint8_t a = (uint8_t)(addr&0xFE);
	if(a!=0x68&&a!=0x6A) return 0;
	for(uint8_t i=1;i<sizeof(maps)/sizeof(maps[0]);i++)
		for(uint8_t k=0;k<3;k++) if(maps[i].ids[k]&&maps[i].ids[k]==id) return (i>=3)==(a==0x6A) ? &maps[i] : 0;    //entries 3, 4 at 0x6A
	return a==0x68&&id!=0x24 ? &maps[0] : 0;
}

uint8_t ImuSetup(const ImuMap* m, uint8_t addr){
	for(const uint8_t* p=m->init;p<m->init+sizeof(m->init)&&*p;p+=2){
		if(*p==0xFF){ uint32_t t = millis; while(millis-t<p[1]); }
		else if(!I2cWriteReg(addr, p[0], p[1])) return 0;
	}
	return 1;
}

void ImuParse(const ImuMap* m, const uint8_t* b, int16_t* raw){
	for(uint8_t k=0;k<7;k++){
		int16_t o = k<3 ? m->oa+2*k : (k==3 ? (m->ot==0xFF ? -1 : m->ot) : m->og+2*(k-4));
		int32_t v = o<0 ? 0 : (int16_t)(m->le ? b[o]|b[o+1]<<8 : b[o]<<8|b[o+1]);
		raw[k] = (int16_t)(k>3&&m->glsb4!=131 ? v*131/m->glsb4 : v);    //gyro in 32.8 LSB per deg/s
	}
}

uint8_t ImuAxes(uint16_t mount, int8_t* ax){
	if(mount==0||mount>36) return 0;
	int8_t up = (int8_t)((mount-1)/6), fwd = (int8_t)((mount-1)%6);
	int8_t i = (int8_t)(up%3), j = (int8_t)(fwd%3);
	if(i==j) return 0;
	//Y = Z x X: e_i x e_j = +-e_k, + for the cyclic order (x y z); the signs of the two multiply in
	int8_t k = (int8_t)(3-i-j), neg = (int8_t)((up>=3)^(fwd>=3)^((j-i+3)%3!=1));
	ax[0] = fwd;
	ax[1] = (int8_t)(k+(neg ? 3 : 0));
	ax[2] = up;
	return 1;
}

void ImuToBoard(const int8_t* ax, const int16_t* chip, float* board){
	for(uint8_t n=0;n<3;n++){
		float v = (float)chip[ax[n]%3];
		board[n] = ax[n]>=3 ? -v : v;
	}
}

//atan on 0..1 as z * (45 + (1 - z) * (14.02 + 3.80 z)) degrees (pi/4 z - z (z - 1)(0.2447 + 0.0663 z),
//max error 0.0015 rad), then the octant
float Atan2Deg(float y, float x){
	float ax = x<0 ? -x : x, ay = y<0 ? -y : y;
	if(ax==0.0f&&ay==0.0f) return 0.0f;
	float z = ax>ay ? ay/ax : ax/ay;
	float a = z*(45.0f+(1.0f-z)*(14.02f+3.80f*z));
	if(ay>ax) a = 90.0f-a;
	if(x<0) a = 180.0f-a;
	return y<0 ? -a : a;
}
