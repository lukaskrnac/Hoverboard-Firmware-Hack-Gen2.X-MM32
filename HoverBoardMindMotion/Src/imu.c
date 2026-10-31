//IMU (imu.h). The chip is read in the main loop every 10 ms: accel, temperature and gyro in one burst from
//0x3B (14 bytes, big endian) over I2C1 by DMA (i2chw.c), started and polled from the main loop. Ranges
//+-4 g and +-1000 deg/s, 44 Hz low pass. Once a second PWR_MGMT_1 is checked: a chip that was reset
//(supply dip) is set up again. Bench board: a clone at 0x69 (WHO_AM_I 0x2E instead of 0x68) with the
//MPU-6050 register map (accel offsets at 0x06, self test at 0x0D-0x10).
#include "imu.h"
#include "i2c.h"
#include "hardware.h"
#include "pinout.h"
#include "main.h"
#include "imumath.h"

#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_DATA 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C            //standby bits per axis: all 0
#define PWR_RUN 0x01               //awake, clock from the X gyro's PLL
#define ACC_LSB_G 8192.0f          //+-4 g
#define GYRO_LSB_DPS 32.8f         //+-1000 deg/s

volatile uint8_t imuState;
volatile int16_t imuRaw[7];
static uint32_t imuT, checkT, angT;
static uint8_t fails, nBias, angles;
static int8_t ax[3] = { 0, 1, 2 };    //board X, Y, Z as chip axis codes (calibrate imu; else the chip's)
static float acc[3], rate[3], bsum[3], bias[3], pitch0, roll0, pitch, roll;    //board axes, chip units / deg

static uint8_t setup(void){
	uint8_t a = (uint8_t)IMU_ADDR;
	return I2cWriteReg(a, REG_PWR_MGMT_1, PWR_RUN)&&I2cWriteReg(a, REG_PWR_MGMT_2, 0)&&I2cWriteReg(a, REG_CONFIG, 0x03)
	       &&I2cWriteReg(a, REG_GYRO_CONFIG, 0x10)&&I2cWriteReg(a, REG_ACCEL_CONFIG, 0x08);
}

void ImuInit(void){
	imuState = IMU_OFF;
#ifdef FOC_TRACE
	return;    //make TRACE=1 (a debug build): no IMU, for the room the trace needs
#endif
	if(IMU_SDA>=PINCOUNT||IMU_SCL>=PINCOUNT||(IMU_ADDR&0xFE)!=0x68) return;    //not configured / another map
	//at 0x68 / 0x69 too, with other register maps (imuid from PinFinder): ICM-42605 / 42688, BMI160 / 270
	if(IMU_ID==0x42||IMU_ID==0x47||IMU_ID==0xD1||IMU_ID==0x24) return;
	//the level: gravity at rest (calibrate imu) is zero pitch and roll; not calibrated (unset words, read as
	//-1 each: ~135 deg) the chip's own level
	if(ImuAxes(IMU_MOUNT, ax)&&!(IMU_REST(0)==0xFFFF&&IMU_REST(1)==0xFFFF&&IMU_REST(2)==0xFFFF)){
		int16_t r[3] = { (int16_t)IMU_REST(0), (int16_t)IMU_REST(1), (int16_t)IMU_REST(2) };
		float b[3];
		ImuToBoard(ax, r, b);
		pitch0 = Atan2Deg(-b[0], b[2]);
		roll0 = Atan2Deg(b[1], b[2]);
	}
	if(!I2cPins((uint8_t)IMU_SDA, (uint8_t)IMU_SCL, 0)){ imuState = IMU_NOT_I2C1; return; }
	I2cUnstick();
	imuState = setup() ? IMU_RUNNING : IMU_NO_ANSWER;
}

//a new reading: board axes, gyro bias, and pitch / roll by a complementary filter (the gyro integrated,
//pulled to the accelerometer's angle with a ~0.5 s time constant: 2 % per 10 ms). ROS axes and signs:
//pitch about Y (left), positive with the front down; roll about X, positive with the right side down.
static void update(void){
	ImuToBoard(ax, (const int16_t*)imuRaw, acc);
	ImuToBoard(ax, (const int16_t*)imuRaw+4, rate);
	//gyro bias: the mean of the first 0.5 s that stands still (all rates below 5 deg/s)
	if(nBias<50){
		uint8_t still = 1;
		for(uint8_t k=0;k<3;k++) if(rate[k]>5.0f*GYRO_LSB_DPS||rate[k]<-5.0f*GYRO_LSB_DPS) still = 0;
		nBias = still ? (uint8_t)(nBias+1) : 0;
		for(uint8_t k=0;k<3;k++){
			bsum[k] = still ? bsum[k]+rate[k] : 0.0f;
			if(nBias==50) bias[k] = bsum[k]/50.0f;
		}
	}
	for(uint8_t k=0;k<3;k++) rate[k] -= bias[k];
	float ap = Atan2Deg(-acc[0], acc[2])-pitch0, ar = Atan2Deg(acc[1], acc[2])-roll0;
	float dt = (float)(millis-angT)*(0.001f/GYRO_LSB_DPS);
	angT = millis;
	if(!angles||dt>0.1f/GYRO_LSB_DPS){ pitch = ap; roll = ar; angles = 1; }
	else{
		pitch = 0.98f*(pitch+rate[1]*dt)+0.02f*ap;
		roll = 0.98f*(roll+rate[0]*dt)+0.02f*ar;
	}
}

void ImuPoll(void){
	static uint8_t b[14];
	if(imuState==IMU_OFF||imuState==IMU_NOT_I2C1) return;
	uint8_t s = I2cPoll();    //the running read (DMA)
	if(s==I2C_BUSY) return;
	if(s==I2C_DONE){
		for(uint8_t k=0;k<7;k++) imuRaw[k] = (int16_t)(b[2*k]<<8|b[2*k+1]);
		update();
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
	if(imuState!=IMU_RUNNING||k>8) return __builtin_nanf("");
	if(k<3) return acc[k]*(9.80665f/ACC_LSB_G);
	if(k<6) return rate[k-3]/GYRO_LSB_DPS;
	if(k==6) return (float)imuRaw[3]/340.0f+36.53f;    //MPU-6050 temperature scale
	return k==7 ? pitch : roll;
}
