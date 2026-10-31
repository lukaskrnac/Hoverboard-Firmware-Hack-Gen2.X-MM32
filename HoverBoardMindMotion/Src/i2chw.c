//I2C master on the I2C1 peripheral (i2c.h), main firmware; PinFinder uses the bit-banged i2c.c on any pins.
//The pins must be I2C1 SDA / SCL (i2cs, hardware.c), else I2cPins() fails and the IMU stays off.
//A transfer runs by DMA (I2C1 TX on channel 2, RX on 3; the uart is on 4 / 5): the I2C FIFO is 2 entries
//deep, fed by the CPU the adc isr (~50 us) could empty it in a burst and the peripheral ends the transfer
//when it runs empty. The commands (data byte, or bit 8 = read one byte) go to DR; the direction change
//makes a repeated start (REPEN). An empty FIFO holds the bus (SCL low, measured: the master stayed active
//after the last byte); the stop is ENR.ABORT after the last byte, as the vendor's I2C_GenerateSTOP().
#include "i2c.h"
#include "hardware.h"
#include "main.h"

#define CR_MASTER   (1u<<0)
#define CR_FAST     (2u<<1)
#define CR_REPEN    (1u<<5)
#define CR_DISSLAVE (1u<<6)
#define RAW_TX_ABRT (1u<<6)
#define SR_TFE      (1u<<2)
#define ENR_ABORT   (1u<<1)
#define DR_READ     0x100u
#define DMA_EN      (1u<<0)
#define DMA_DIR     (1u<<4)    //memory to peripheral
#define DMA_MINC    (1u<<7)
#define DMA_P16     (1u<<8)
#define DMA_M16     (1u<<10)

static void dmaOff(void){ DMA1_Channel2->CCR = DMA1_Channel3->CCR = 0; }

static uint8_t sdaIo = 0xFF, sclIo = 0xFF, busy, nRead;
static uint16_t cmd[16];
static uint32_t startT;

static __attribute__((noinline)) int8_t entry(uint8_t io, uint8_t scl){    //(one copy: LTO inlined it twice)
	for(uint8_t i=0;i<I2CCOUNT;i++) if(i2cs[i].io==io&&i2cs[i].scl==scl) return (int8_t)i;
	return -1;
}

uint8_t I2cPins(uint8_t sda, uint8_t scl, uint8_t slow){
	(void)slow;
	int8_t a = entry(sda, 0), c = entry(scl, 1);
	if(a<0||c<0) return 0;
	sdaIo = sda; sclIo = scl;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1;
	RCC->AHBENR |= RCC_AHBENR_DMA1;
	I2C1->ENR = 0;
	I2C1->CR = CR_MASTER|CR_FAST|CR_REPEN|CR_DISSLAVE;
	I2C1->FSHR = 70;     //PCLK1 72 MHz: ~1 us high, ~1.7 us low, ~350 kHz
	I2C1->FSLR = 120;
	I2C1->IMR = 0;
	I2C1->DMA = 3;       //TX and RX DMA requests
	pinModeAF(sda, i2cs[a].af);
	pinModeAF(scl, i2cs[c].af);
	pinMode(sda, OUTPUT_AF_OD);
	pinMode(scl, OUTPUT_AF_OD);
	I2C1->ENR = 1;
	return 1;
}

//a slave left in a transfer (reset in the middle of a read) holds SDA low: 9 clocks on SCL as a gpio,
//then the pins back to I2C1 (bus recovery only, no transfer runs this way)
void I2cUnstick(void){
	if(sdaIo>=PINCOUNT) return;
	I2C1->ENR = 0;
	pinMode(sclIo, OUTPUT_OD);
	for(uint8_t k=18;k;k--){ digitalWrite(sclIo, k&1); for(volatile uint8_t d=40;d;d--); }    //ends high
	pinMode(sclIo, OUTPUT_AF_OD);
	busy = 0;
	I2C1->ENR = 1;
}

//queues n commands for addr (n_read of them reads into buf) by DMA
static void begin(uint8_t addr, uint8_t n, uint8_t* buf, uint8_t n_read){
	dmaOff();
	(void)I2C1->TX_ABRT;    //clears an old abort (and releases the TX FIFO)
	I2C1->ENR = 0;          //TAR is written while disabled; disabling also empties both FIFOs
	I2C1->TAR = addr;
	I2C1->ENR = 1;
	nRead = n_read;
	if(n_read){
		DMA1_Channel3->CPAR = (u32)&I2C1->DR;
		DMA1_Channel3->CMAR = (u32)buf;
		DMA1_Channel3->CNDTR = n_read;
		DMA1_Channel3->CCR = DMA_MINC|DMA_P16|DMA_EN;
	}
	DMA1_Channel2->CPAR = (u32)&I2C1->DR;
	DMA1_Channel2->CMAR = (u32)cmd;
	DMA1_Channel2->CNDTR = n;
	startT = millis;
	busy = 1;
	DMA1_Channel2->CCR = DMA_MINC|DMA_DIR|DMA_P16|DMA_M16|DMA_EN;
}

uint8_t I2cPoll(void){
	if(!busy) return I2C_IDLE;
	if(I2C1->RAWISR&RAW_TX_ABRT){    //no acknowledge (address or data): the peripheral sent the stop
		busy = 0;
		dmaOff();
		(void)I2C1->TX_ABRT;
		return I2C_FAIL;
	}
	uint8_t done = nRead ? DMA1_Channel3->CNDTR==0 : DMA1_Channel2->CNDTR==0&&(I2C1->SR&SR_TFE);
	if(done){
		busy = 0;
		dmaOff();
		I2C1->ENR |= ENR_ABORT;    //stop after the byte on the bus (the last one): ~25 us
		for(uint16_t i=0;i<1000&&(I2C1->ENR&ENR_ABORT);i++);
		(void)I2C1->TX_ABRT;       //the abort's flag
		return I2C_DONE;
	}
	if(millis-startT>3){    //a 15 byte read takes ~0.5 ms: bus held, or the peripheral stuck
		busy = 0;
		dmaOff();    //(the next begin() disables and enables the peripheral)
		return I2C_FAIL;
	}
	return I2C_BUSY;
}

static uint8_t wait(void){
	uint8_t s;
	while((s = I2cPoll())==I2C_BUSY);
	return s==I2C_DONE;
}

void I2cReadAsync(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n){
	if(sdaIo>=PINCOUNT||n>15) return;
	cmd[0] = reg;
	for(uint8_t i=1;i<=n;i++) cmd[i] = DR_READ;
	begin(addr, (uint8_t)(n+1), buf, n);
}

uint8_t I2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n){    //(imu.c calls these only after I2cPins() succeeded)
	I2cReadAsync(addr, reg, buf, n);
	return wait();
}

int16_t I2cReadReg(uint8_t addr, uint8_t reg){
	uint8_t v;
	return I2cRead(addr, reg, &v, 1) ? v : -1;
}

uint8_t I2cWriteReg(uint8_t addr, uint8_t reg, uint8_t v){
	cmd[0] = reg;
	cmd[1] = v;
	begin(addr, 2, 0, 0);
	return wait();
}
