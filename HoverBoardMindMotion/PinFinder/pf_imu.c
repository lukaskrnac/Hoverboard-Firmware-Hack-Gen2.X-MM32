//IMU search: I2C lines have external pull-ups, so the candidates are the free pins that stay high with the
//internal pull-down on. Every pair of them is tried as SDA/SCL (bit-banged, open drain, ~50 kHz): the
//addresses that acknowledge, and the id register of the usual IMUs / accelerometers (i2c.c).
#include "pf.h"
#include "i2c.h"
#include "imumath.h"
#include <string.h>

//address, id register, value; the names in the same order, '\0' separated
static const struct{ uint8_t addr, reg, val; } chips[] = {
	{ 0x68, 0x75, 0x68 }, { 0x68, 0x75, 0x2E }, { 0x68, 0x75, 0x70 }, { 0x68, 0x75, 0x71 },
	{ 0x68, 0x75, 0x12 }, { 0x68, 0x75, 0xAF }, { 0x68, 0x75, 0x98 }, { 0x68, 0x75, 0x42 },
	{ 0x68, 0x75, 0x47 }, { 0x68, 0x00, 0xD1 }, { 0x68, 0x00, 0x24 }, { 0x6A, 0x0F, 0x69 },
	{ 0x6A, 0x0F, 0x6A }, { 0x6A, 0x0F, 0x6C }, { 0x6A, 0x00, 0x05 }, { 0x18, 0x0F, 0x33 },
	{ 0x18, 0x0F, 0x11 }, { 0x12, 0x00, 0xE7 }, { 0x12, 0x00, 0xB0 },
};
static const char chipNames[] = T(
	"MPU-6050\0" "MPU-6050 clone\0" "MPU-6500\0" "MPU-9250\0"
	"ICM-20602\0" "ICM-20608\0" "ICM-20689\0" "ICM-42605\0"
	"ICM-42688\0" "BMI160\0" "BMI270\0" "LSM6DS3\0"
	"LSM6DSL/DS3TR-C\0" "LSM6DSO\0" "QMI8658\0" "LIS3DH\0"
	"SC7A20\0" "QMA7981\0" "QMA6981\0");

static void pairRelease(uint16_t sda, uint16_t scl){    //I2cRelease floats them: back to the rest state
	I2cRelease();
	PinRest(sda);
	PinRest(scl);
}

void DetectImu(void){
	uint8_t cand[8], n = 0;
	uint16_t keep[2] = { IMU_SDA, IMU_SCL };
	IMU_SDA = IMU_SCL = PIN_NONE;
	for(uint16_t p=0;p<PINCOUNT&&n<8;p++){    //external pull-up: high against the internal pull-down
		if(!PinFree(p)) continue;
		pinMode((uint8_t)p, INPUT_PULLDOWN);
		WaitUs(50);
		if(digitalRead((uint8_t)p)) cand[n++] = (uint8_t)p;
		PinRest(p);    //pulled up again while the latch is unknown: floating it may switch the board off
	}
	for(uint8_t i=0;i<n;i++) kvp("pullup", cand[i]);
	uint8_t found = 0;
	for(uint8_t i=0;i<n&&!found;i++) for(uint8_t j=0;j<n&&!found;j++){
		if(i==j) continue;
		progress(i*n+j+1, n*n);    //every pair probes 112 addresses: seconds without output otherwise
		uint8_t sda = cand[i], scl = cand[j];
		I2cPins(sda, scl, 1);
		if(!I2cHigh()){ pairRelease(sda, scl); continue; }
		I2cUnstick();
		uint8_t acks = 0;
		for(uint8_t a=0x08;a<0x78&&acks<=8;a++) acks = (uint8_t)(acks+I2cProbe(a));
		if(!acks||acks>8){ pairRelease(sda, scl); continue; }    //nothing, or SDA does not follow (not an I2C pair)
		for(uint8_t a=0x08;a<0x78;a++){
			if(!I2cProbe(a)) continue;
			found = 1;
			kv("i2c"); out("sda "); outpin(sda); out(" scl "); outpin(scl); out(" addr "); outi(a);
			static const uint8_t idregs[3] = { 0x00, 0x0F, 0x75 };    //the usual id registers
			for(uint8_t r=0;r<3;r++){ out(" reg"); outi(idregs[r]); outc('='); outi(I2cReadReg(a, idregs[r])); }
			nl();
			uint8_t known = 0;
			const char* name = chipNames;
			for(uint8_t c=0;c<sizeof(chips)/sizeof(chips[0])&&!known;c++,name+=strlen(name)+1){
				if((chips[c].addr&0xFE)!=(a&0xFE)) continue;    //the chips' two addresses (SA0)
				int16_t v = I2cReadReg(a, chips[c].reg);
				if(v!=chips[c].val) continue;
				kvs("imu", name);
				IMU_SDA = sda; IMU_SCL = scl; IMU_ADDR = a; IMU_ID = (uint16_t)v;
				known = 1;
			}
			//an unknown chip at a usual IMU / accelerometer address (clones answer with other ids): keep it
			if(!known&&IMU_SDA==PIN_NONE&&((a&0xFE)==0x68||(a&0xFE)==0x6A||(a&0xFE)==0x18||(a&0xFE)==0x12)){
				int16_t v = I2cReadReg(a, (a&0xFE)==0x68 ? 0x75 : ((a&0xFE)==0x12 ? 0x00 : 0x0F));
				warn("unknown chip at an IMU address: kept");
				kvs("imu", "unknown");
				IMU_SDA = sda; IMU_SCL = scl; IMU_ADDR = a; IMU_ID = (uint16_t)(v<0 ? 0 : v);
			}
		}
		pairRelease(sda, scl);
	}
	progressEnd();
	if(IMU_SDA==PIN_NONE){
		IMU_SDA = keep[0]; IMU_SCL = keep[1];
		err(found ? "i2c devices found, no known IMU" : (n<2 ? "fewer than 2 free pins with a pull-up" : "no i2c device answers"));
		return;
	}
	kvp("imusda", IMU_SDA);
	kvp("imuscl", IMU_SCL);
	kvi("imuaddr", IMU_ADDR);
	kvi("imuid", IMU_ID);
	HwCheck();
	ok();
}

//calibrate imu: the mounting (imumount, imurestx/y/z; imumath.h). 1) The board stands still as when
//driving: gravity gives up, and is kept as the level (the main firmware's zero pitch and roll). 2) It is
//tilted forward (front down: the side the wheel moves to at positive speed) and held: gravity gains a
//component along -forward, so forward is the largest change across up. Units 1/8 of the chip's (1 g = 1024).
static uint8_t gravity(int16_t* g){    //accel averaged over 32 readings (~0.2 s)
	int32_t s[3] = { 0, 0, 0 };
	for(uint8_t n=0;n<32;n++){
		uint8_t b[6];
		if(!I2cRead((uint8_t)IMU_ADDR, 0x3B, b, 6)) return 0;
		for(uint8_t k=0;k<3;k++) s[k] += (int16_t)(b[2*k]<<8|b[2*k+1]);
		Wait(5);
	}
	for(uint8_t k=0;k<3;k++) g[k] = (int16_t)(s[k]/256);
	return 1;
}

static int8_t dominant(const int32_t* v){    //chip axis code of the largest component
	int8_t k = 0;
	for(int8_t i=1;i<3;i++) if(v[i]*v[i]>v[k]*v[k]) k = i;
	return (int8_t)(k+(v[k]<0 ? 3 : 0));
}

void CalImu(void){
	static const char axis[6][3] = { "+x", "+y", "+z", "-x", "-y", "-z" };
	if(IMU_SDA>=PINCOUNT||IMU_SCL>=PINCOUNT||(IMU_ADDR&0xFE)!=0x68){ err("no IMU at 0x68/0x69 (detect imu)"); return; }
	I2cPins((uint8_t)IMU_SDA, (uint8_t)IMU_SCL, 0);
	I2cUnstick();
	int16_t g0[3], g1[3], prev[3];
	int32_t d[3] = { 0, 0, 0 }, n0 = 0;
	uint8_t a = (uint8_t)IMU_ADDR, good = I2cWriteReg(a, 0x6B, 0x01)&&I2cWriteReg(a, 0x1C, 0x08), done = 0;
	note("keep the board still as when driving");
	Wait(1500);
	good = good&&gravity(g0);
	for(uint8_t k=0;k<3;k++){ prev[k] = g0[k]; n0 += g0[k]*g0[k]; d[k] = g0[k]; }
	int8_t up = dominant(d);
	note("tilt the front down 20-60 deg, hold");
	for(uint8_t t=0;good&&!done&&t<150&&!aborted();t++){    //~30 s
		good = gravity(g1);
		//steady (within ~2 deg of the last reading) and turned more than ~15 deg across up
		int32_t m = 0, h = 0;
		for(uint8_t k=0;k<3;k++){
			int32_t e = g1[k]-prev[k];
			m += e*e;
			d[k] = k==up%3 ? 0 : g0[k]-g1[k];
			h += d[k]*d[k];
			prev[k] = g1[k];
		}
		done = good&&m<n0/800&&h>n0/15;
	}
	I2cRelease();
	if(!done){ err(!good ? "the IMU does not answer" : "no steady forward tilt"); return; }
	int8_t fwd = dominant(d), ax[3];
	ImuAxes((uint16_t)(1+up*6+fwd), ax);
	IMU_MOUNT = (uint16_t)(1+up*6+fwd);
	IMU_REST(0) = (uint16_t)g0[0]; IMU_REST(1) = (uint16_t)g0[1]; IMU_REST(2) = (uint16_t)g0[2];
	kvs("forward", axis[fwd]); kvs("left", axis[ax[1]]); kvs("up", axis[up]);
	kvi("imumount", IMU_MOUNT);
	ok();
}
