//IMU (imu.h). The chip is read in the main loop every 10 ms: accel, temperature and gyro in one burst from
//0x3B (14 bytes, big endian) over the bit-banged I2C (i2c.c), one byte per main loop pass. Ranges
//+-4 g and +-1000 deg/s, 44 Hz low pass. Once a second PWR_MGMT_1 is checked: a chip that was reset
//(supply dip) is set up again. Bench board: a clone at 0x69 (WHO_AM_I 0x2E instead of 0x68) with the
//MPU-6050 register map (accel offsets at 0x06, self test at 0x0D-0x10).
#include "imu.h"
#include "i2c.h"
#include "hardware.h"
#include "pinout.h"
#include "main.h"

#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_DATA 0x3B
#define REG_PWR_MGMT_1 0x6B
#define PWR_RUN 0x01               //awake, clock from the X gyro's PLL
#define ACC_LSB_G 8192.0f          //+-4 g
#define GYRO_LSB_DPS 32.8f         //+-1000 deg/s

volatile uint8_t imuState;
volatile int16_t imuRaw[7];
static uint32_t imuT, checkT;
static uint8_t fails;

static uint8_t setup(void){
	uint8_t a = (uint8_t)IMU_ADDR;
	return I2cWriteReg(a, REG_PWR_MGMT_1, PWR_RUN)&&I2cWriteReg(a, REG_CONFIG, 0x03)
	       &&I2cWriteReg(a, REG_GYRO_CONFIG, 0x10)&&I2cWriteReg(a, REG_ACCEL_CONFIG, 0x08);
}

void ImuInit(void){
	imuState = IMU_OFF;
	if(IMU_SDA>=PINCOUNT||IMU_SCL>=PINCOUNT||(IMU_ADDR&0xFE)!=0x68) return;    //not configured / another map
	//at 0x68 / 0x69 too, with other register maps (imuid from PinFinder): ICM-42605 / 42688, BMI160 / 270
	if(IMU_ID==0x42||IMU_ID==0x47||IMU_ID==0xD1||IMU_ID==0x24) return;
	I2cPins((uint8_t)IMU_SDA, (uint8_t)IMU_SCL, 0);
	I2cUnstick();
	imuState = setup() ? IMU_RUNNING : IMU_NO_ANSWER;
}

void ImuPoll(void){
	static uint8_t b[14];
	if(imuState==IMU_OFF) return;
	uint8_t s = I2cPoll();    //one byte of the running read per main loop pass
	if(s==I2C_BUSY) return;
	if(s==I2C_DONE){
		for(uint8_t k=0;k<7;k++) imuRaw[k] = (int16_t)(b[2*k]<<8|b[2*k+1]);
		imuState = IMU_RUNNING;
		fails = 0;
	}else if(s==I2C_FAIL&&++fails>=10){    //100 ms without an answer
		imuState = IMU_NO_ANSWER;
		I2cUnstick();
	}
	if(millis-imuT<10) return;
	imuT = millis;
	if(millis-checkT>=1000){    //reset (it sleeps again) or never answered: set up again (~0.3 ms)
		checkT = millis;
		if(I2cReadReg((uint8_t)IMU_ADDR, REG_PWR_MGMT_1)!=PWR_RUN){
			I2cUnstick();
			if(!setup()){ imuState = IMU_NO_ANSWER; return; }
		}
	}
	I2cReadAsync((uint8_t)IMU_ADDR, REG_DATA, b, 14);
}

float ImuValue(uint8_t k){
	if(imuState!=IMU_RUNNING||k>6) return __builtin_nanf("");
	if(k<3) return (float)imuRaw[k]*(9.80665f/ACC_LSB_G);
	if(k<6) return (float)imuRaw[k+1]/GYRO_LSB_DPS;
	return (float)imuRaw[3]/340.0f+36.53f;    //MPU-6050 temperature scale
}
