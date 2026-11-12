//IMU (imu.h). The chip is read in the main loop every 10 ms: accel, temperature and gyro in one burst
//over I2C1 by DMA (i2chw.c), started and polled from the main loop; register maps in imumath.c (InvenSense,
//ICM-426xx, BMI160, LSM6DS*, QMI8658). Ranges +-4 g and ~1000 deg/s. Once a second a register is checked:
//a chip that was reset (supply dip) is set up again. Bench board: a clone at 0x69 (WHO_AM_I 0x2E instead of 0x68) with the
//MPU-6050 register map (accel offsets at 0x06, self test at 0x0D-0x10).
#include "imu.h"
#include "i2c.h"
#include "hardware.h"
#include "pinout.h"
#include "main.h"
#include "imumath.h"

#define ACC_LSB_G 8192.0f          //+-4 g
#define GYRO_LSB_DPS 32.8f         //+-1000 deg/s (ImuParse scales every map to it)

volatile uint8_t imuState;
volatile int16_t imuRaw[7];
static uint32_t imuT, checkT, angT;
static uint8_t fails, nBias, angles;
static const ImuMap* map;
static int8_t ax[3] = { 0, 1, 2 };    //board X, Y, Z as chip axis codes (calibrate imu; else the chip's)
static float acc[3], rate[3], bsum[3], bias[3], pitch0, roll0, pitch, roll;    //board axes, chip units / deg

static uint8_t setup(void){ return ImuSetup(map, (uint8_t)IMU_ADDR); }

void ImuInit(void){
	imuState = IMU_OFF;
#ifdef FOC_TRACE
	return;    //make TRACE=1 (a debug build): no IMU, for the room the trace needs
#endif
	map = ImuMapFor(IMU_ADDR, IMU_ID);
	if(IMU_SDA>=PINCOUNT||IMU_SCL>=PINCOUNT||!map) return;    //not configured / not supported (BMI270)
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
		ImuParse(map, b, (int16_t*)imuRaw);
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
		if(I2cReadReg((uint8_t)IMU_ADDR, map->chk)!=map->run){
			I2cUnstick();
			if(!setup()){ imuState = IMU_NO_ANSWER; return; }
		}
	}
	I2cReadAsync((uint8_t)IMU_ADDR, map->data, b, map->n);
}

float ImuValue(uint8_t k){
	if(imuState!=IMU_RUNNING||k>8) return __builtin_nanf("");
	if(k<3) return acc[k]*(9.80665f/ACC_LSB_G);
	if(k<6) return rate[k-3]/GYRO_LSB_DPS;
	if(k==6) return map->ot==0xFF ? __builtin_nanf("") : (float)imuRaw[3]*100.0f/(float)map->tdiv+(float)map->t0*0.01f;
	return k==7 ? pitch : roll;
}
