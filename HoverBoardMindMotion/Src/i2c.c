//Bit-banged I2C master (i2c.h). Both pins are general purpose open drain outputs for the whole transfer:
//a line is released (the board's pull-up) or pulled low by BSRR / BRR, which are atomic, and read back
//through IDR, so nothing here read-modify-writes a port's mode register while the interrupts run. SCL is
//checked for clock stretching (bounded). An interrupt between two edges only makes a bit longer.
#include "i2c.h"
#include "hardware.h"

static GPIO_TypeDef* sdaPort;
static GPIO_TypeDef* sclPort;
static uint16_t sdaBit, sclBit;
static uint8_t sdaIo, sclIo, half;

static GPIO_TypeDef* portOf(uint8_t io, uint16_t* bit){
	uint8_t pb = PinPortBit(io);
	*bit = (uint16_t)(1u<<(pb&15));
	return (GPIO_TypeDef*)(GPIOA_BASE+0x400u*(pb>>4));
}

static void dly(void){ for(volatile uint8_t i=half;i;i--); }    //~0.15 us per count at 72 MHz
static void sdaLo(void){ sdaPort->BRR = sdaBit; }
static void sdaHi(void){ sdaPort->BSRR = sdaBit; }
static void sclLo(void){ sclPort->BRR = sclBit; }
static uint8_t sda(void){ return (sdaPort->IDR&sdaBit)!=0; }

static void sclHi(void){    //released; waits while a slave stretches the clock (~1 ms at most)
	sclPort->BSRR = sclBit;
	for(uint16_t i=0;i<2000&&!(sclPort->IDR&sclBit);i++) dly();
	dly();
}

void I2cPins(uint8_t sdaPin, uint8_t sclPin, uint8_t slow){
	sdaIo = sdaPin; sclIo = sclPin;
	half = slow ? 40 : 1;
	sdaPort = portOf(sdaIo, &sdaBit);
	sclPort = portOf(sclIo, &sclBit);
	sdaHi(); sclPort->BSRR = sclBit;
	pinMode(sdaIo, OUTPUT_OD);
	pinMode(sclIo, OUTPUT_OD);
	dly();
}

void I2cRelease(void){
	pinMode(sdaIo, INPUT);
	pinMode(sclIo, INPUT);
}

uint8_t I2cHigh(void){ return sda()&&(sclPort->IDR&sclBit)!=0; }

static void start(void){ sdaHi(); sclHi(); sdaLo(); dly(); sclLo(); dly(); }
static void stop(void){ sdaLo(); dly(); sclHi(); sdaHi(); dly(); }

void I2cUnstick(void){
	sdaHi();
	for(uint8_t k=0;k<9;k++){ sclLo(); dly(); sclHi(); }
	stop();
}

static uint8_t writeByte(uint8_t b){    //1 = acknowledged
	for(uint8_t i=0;i<8;i++){
		if(b&0x80) sdaHi(); else sdaLo();
		b = (uint8_t)(b<<1);
		dly();
		sclHi();
		sclLo();
	}
	sdaHi();
	dly();
	sclHi();
	uint8_t ack = !sda();
	sclLo();
	return ack;
}

static uint8_t readByte(uint8_t ack){
	uint8_t b = 0;
	sdaHi();
	for(uint8_t i=0;i<8;i++){
		dly();
		sclHi();
		b = (uint8_t)(b<<1|sda());
		sclLo();
	}
	if(ack) sdaLo();
	dly();
	sclHi();
	sclLo();
	sdaHi();
	return b;
}

uint8_t I2cProbe(uint8_t addr){
	start();
	uint8_t a = writeByte((uint8_t)(addr<<1));
	stop();
	return a;
}

uint8_t I2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n){
	start();
	uint8_t a = writeByte((uint8_t)(addr<<1))&&writeByte(reg);
	if(a){
		start();
		a = writeByte((uint8_t)(addr<<1|1));
		for(uint8_t i=0;a&&i<n;i++) buf[i] = readByte(i+1<n);
	}
	stop();
	return a;
}

int16_t I2cReadReg(uint8_t addr, uint8_t reg){
	uint8_t v;
	return I2cRead(addr, reg, &v, 1) ? v : -1;
}

//The burst read one byte per call: a 14 byte read blocked the main loop for ~1 ms in one piece (the bits
//are slow: flash wait states, the adc isr), byte by byte it is ~60 us per call. Between two calls the
//bus waits with SCL low, which I2C allows for any time.
static uint8_t aAddr, aReg, aN, aK, aStep;
static uint8_t* aBuf;

void I2cReadAsync(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n){
	aAddr = addr; aReg = reg; aBuf = buf; aN = n; aStep = 1;
}

uint8_t I2cPoll(void){
	uint8_t ok = 1;
	switch(aStep){
		case 0: return I2C_IDLE;
		case 1: start(); ok = writeByte((uint8_t)(aAddr<<1)); break;
		case 2: ok = writeByte(aReg); break;
		case 3: start(); ok = writeByte((uint8_t)(aAddr<<1|1)); aK = 0; break;
		default:
			aBuf[aK] = readByte(aK+1<aN);
			if(++aK<aN) return I2C_BUSY;
			stop(); aStep = 0;
			return I2C_DONE;
	}
	if(!ok){ stop(); aStep = 0; return I2C_FAIL; }
	aStep++;
	return I2C_BUSY;
}

uint8_t I2cWriteReg(uint8_t addr, uint8_t reg, uint8_t v){
	start();
	uint8_t a = writeByte((uint8_t)(addr<<1))&&writeByte(reg)&&writeByte(v);
	stop();
	return a;
}
