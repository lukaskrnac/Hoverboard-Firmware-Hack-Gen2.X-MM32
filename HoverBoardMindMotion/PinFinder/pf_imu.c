//IMU search: I2C lines have external pull-ups, so the candidates are the free pins that stay high with the
//internal pull-down on. Every pair of them is tried as SDA/SCL (bit-banged, open drain, ~50 kHz): the
//addresses that acknowledge, and the id register of the usual IMUs / accelerometers.
#include "pf.h"

static uint8_t sda, scl;

static void lo(uint8_t p){ digitalWrite(p, 0); pinMode(p, OUTPUT_OD); }
static void hi(uint8_t p){ pinMode(p, INPUT); }    //released: the external pull-up
static void dly(void){ WaitUs(10); }

static uint8_t sclHigh(void){    //with clock stretching (bounded)
	hi(scl);
	for(uint16_t i=0;i<200&&!digitalRead(scl);i++) WaitUs(5);
	dly();
	return digitalRead(scl);
}

static void start(void){ hi(sda); sclHigh(); lo(sda); dly(); lo(scl); dly(); }
static void stop(void){ lo(sda); dly(); sclHigh(); hi(sda); dly(); }

static uint8_t writeByte(uint8_t b){    //1 = acknowledged
	for(uint8_t i=0;i<8;i++){
		if(b&0x80) hi(sda); else lo(sda);
		b = (uint8_t)(b<<1);
		dly();
		sclHigh();
		lo(scl);
	}
	hi(sda);
	dly();
	sclHigh();
	uint8_t ack = !digitalRead(sda);
	lo(scl);
	return ack;
}

static uint8_t readByte(uint8_t ack){
	uint8_t b = 0;
	hi(sda);
	for(uint8_t i=0;i<8;i++){
		dly();
		sclHigh();
		b = (uint8_t)(b<<1|digitalRead(sda));
		lo(scl);
	}
	if(ack) lo(sda); else hi(sda);
	dly();
	sclHigh();
	lo(scl);
	hi(sda);
	return b;
}

static uint8_t probe(uint8_t addr){
	start();
	uint8_t a = writeByte((uint8_t)(addr<<1));
	stop();
	return a;
}

static int16_t readReg(uint8_t addr, uint8_t reg){    //-1: no acknowledge
	start();
	if(!writeByte((uint8_t)(addr<<1))||!writeByte(reg)){ stop(); return -1; }
	start();
	if(!writeByte((uint8_t)(addr<<1|1))){ stop(); return -1; }
	uint8_t v = readByte(0);
	stop();
	return v;
}

//address, id register, value, name
static const struct{ uint8_t addr, reg, val; const char* name; } chips[] = {
	{ 0x68, 0x75, 0x68, "MPU-6050" }, { 0x68, 0x75, 0x70, "MPU-6500" }, { 0x68, 0x75, 0x71, "MPU-9250" },
	{ 0x68, 0x75, 0x12, "ICM-20602" }, { 0x68, 0x75, 0xAF, "ICM-20608" }, { 0x68, 0x75, 0x98, "ICM-20689" },
	{ 0x68, 0x75, 0x42, "ICM-42605" }, { 0x68, 0x75, 0x47, "ICM-42688" }, { 0x68, 0x00, 0xD1, "BMI160" },
	{ 0x68, 0x00, 0x24, "BMI270" }, { 0x6A, 0x0F, 0x69, "LSM6DS3" }, { 0x6A, 0x0F, 0x6A, "LSM6DSL/DS3TR-C" },
	{ 0x6A, 0x0F, 0x6C, "LSM6DSO" }, { 0x6A, 0x00, 0x05, "QMI8658" }, { 0x18, 0x0F, 0x33, "LIS3DH" },
	{ 0x18, 0x0F, 0x11, "SC7A20" }, { 0x12, 0x00, 0xE7, "QMA7981" }, { 0x12, 0x00, 0xB0, "QMA6981" },
};

void DetectImu(void){
	uint8_t cand[8], n = 0;
	uint16_t keep[2] = { IMU_SDA, IMU_SCL };
	IMU_SDA = IMU_SCL = PIN_NONE;
	for(uint16_t p=0;p<PINCOUNT&&n<8;p++){    //external pull-up: high against the internal pull-down
		if(!PinFree(p)) continue;
		pinMode((uint8_t)p, INPUT_PULLDOWN);
		WaitUs(50);
		if(digitalRead((uint8_t)p)) cand[n++] = (uint8_t)p;
		pinMode((uint8_t)p, INPUT);
	}
	for(uint8_t i=0;i<n;i++) kvp("pullup", cand[i]);
	uint8_t found = 0;
	for(uint8_t i=0;i<n&&!found;i++) for(uint8_t j=0;j<n&&!found;j++){
		if(i==j) continue;
		sda = cand[i];
		scl = cand[j];
		hi(sda); hi(scl);
		dly();
		if(!digitalRead(sda)||!digitalRead(scl)) continue;
		for(uint8_t k=0;k<9;k++){ lo(scl); dly(); sclHigh(); }    //free a slave stuck in a transfer
		stop();
		uint8_t acks = 0;
		for(uint8_t a=0x08;a<0x78&&acks<=8;a++) acks = (uint8_t)(acks+probe(a));
		if(!acks||acks>8){ hi(sda); hi(scl); continue; }    //nothing, or SDA does not follow (not an I2C pair)
		for(uint8_t a=0x08;a<0x78;a++){
			if(!probe(a)) continue;
			found = 1;
			kv("i2c"); out("sda "); outpin(sda); out(" scl "); outpin(scl); out(" addr "); outi(a);
			static const uint8_t idregs[3] = { 0x00, 0x0F, 0x75 };    //the usual id registers
			for(uint8_t r=0;r<3;r++){ out(" reg"); outi(idregs[r]); outc('='); outi(readReg(a, idregs[r])); }
			nl();
			uint8_t known = 0;
			for(uint8_t c=0;c<sizeof(chips)/sizeof(chips[0])&&!known;c++){
				if((chips[c].addr&0xFE)!=(a&0xFE)) continue;    //the chips' two addresses (SA0)
				int16_t v = readReg(a, chips[c].reg);
				if(v!=chips[c].val) continue;
				kvs("imu", chips[c].name);
				IMU_SDA = sda; IMU_SCL = scl; IMU_ADDR = a; IMU_ID = (uint16_t)v;
				known = 1;
			}
			//an unknown chip at a usual IMU / accelerometer address (clones answer with other ids): keep it
			if(!known&&IMU_SDA==PIN_NONE&&((a&0xFE)==0x68||(a&0xFE)==0x6A||(a&0xFE)==0x18||(a&0xFE)==0x12)){
				int16_t v = readReg(a, (a&0xFE)==0x68 ? 0x75 : ((a&0xFE)==0x12 ? 0x00 : 0x0F));
				warn("unknown chip at an IMU address: kept as the IMU (imuid = its WHO_AM_I)");
				kvs("imu", "unknown");
				IMU_SDA = sda; IMU_SCL = scl; IMU_ADDR = a; IMU_ID = (uint16_t)(v<0 ? 0 : v);
			}
		}
		hi(sda); hi(scl);
	}
	if(IMU_SDA==PIN_NONE){
		IMU_SDA = keep[0]; IMU_SCL = keep[1];
		err(found ? "i2c devices found, none of them a known IMU" : (n<2 ? "fewer than 2 free pins with a pull-up" : "no i2c device answers"));
		return;
	}
	kvp("imusda", IMU_SDA);
	kvp("imuscl", IMU_SCL);
	kvi("imuaddr", IMU_ADDR);
	kvi("imuid", IMU_ID);
	ok();
}
