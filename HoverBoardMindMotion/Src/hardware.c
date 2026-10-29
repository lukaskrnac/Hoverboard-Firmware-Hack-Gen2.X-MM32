#include "hardware.h"

//pins[]: port (IOA..IOD) in the high nibble, pin number in the low one
#define P(port, n) (((port)-IOA)<<4|(n))
static GPIO_TypeDef* const ports[4] = { GPIOA, GPIOB, GPIOC, GPIOD };




//Tables per MCU: from the vendor datasheets' pin and alternate function tables (AF0..AF7).
#if defined(MCU_SPIN25)
//MM32SPIN25 (DS_MM32SPIN2x_p 1.21). TIM1 PWM as the SPIN05: PA8-PA10 / PB13-PB15 on AF2.

static const uint8_t pins[PINCOUNT] = {
P(IOA,0),
P(IOA,1),
P(IOA,2),
P(IOA,3),
P(IOA,4),
P(IOA,5),
P(IOA,6),
P(IOA,7),
P(IOA,11),
P(IOA,12),
P(IOA,13),
P(IOA,14),
P(IOA,15),
P(IOB,0),
P(IOB,1),
P(IOB,2),
P(IOB,3),
P(IOB,4),
P(IOB,5),
P(IOB,6),
P(IOB,7),
P(IOB,8),
P(IOB,9),
P(IOB,10),
P(IOB,11),
P(IOB,12),
P(IOC,13),
P(IOC,14),
P(IOC,15),
P(IOD,0),
P(IOD,1),
P(IOD,2),
P(IOD,3),
P(IOD,7),
};

const MM32ADC adcs[ADCCOUNT] = {
{PA0,ADC_Channel_0},
{PA1,ADC_Channel_1},
{PA2,ADC_Channel_2},
{PA3,ADC_Channel_3},
{PA4,ADC_Channel_4},
{PA5,ADC_Channel_5},
{PA6,ADC_Channel_6},
{PA7,ADC_Channel_7},
{PB0,ADC_Channel_8},
{PB1,ADC_Channel_9},
{PB2,ADC_Channel_10},
{PB10,ADC_Channel_11},
};

const MM32TIM23 halltims[TIMCOUNT] = {
{PA0,GPIO_AF_2,HT(1,1)},    //DS_MM32SPIN2x_p 1.21 tables 6-9
{PA1,GPIO_AF_2,HT(1,2)},
{PA2,GPIO_AF_2,HT(1,3)},
{PA5,GPIO_AF_2,HT(1,1)},
{PA11,GPIO_AF_4,HT(1,1)},
{PA12,GPIO_AF_4,HT(1,2)},
{PA15,GPIO_AF_2,HT(1,1)},
{PB1,GPIO_AF_4,HT(1,3)},
{PB3,GPIO_AF_2,HT(1,2)},
{PB10,GPIO_AF_2,HT(1,3)},
{PC13,GPIO_AF_6,HT(1,1)},
{PC14,GPIO_AF_6,HT(1,2)},
{PC15,GPIO_AF_6,HT(1,3)},
{PA6,GPIO_AF_1,HT(0,1)},
{PA7,GPIO_AF_1,HT(0,2)},
{PB0,GPIO_AF_1,HT(0,3)},
{PB4,GPIO_AF_1,HT(0,1)},
{PB5,GPIO_AF_1,HT(0,2)},
{PB8,GPIO_AF_6,HT(0,2)},
{PB9,GPIO_AF_6,HT(0,3)},
{PD7,GPIO_AF_5,HT(0,1)},
};

//UART2 TX on PA14 (AF1) is SWCLK: not listed
const MM32UART uarts[UARTCOUNT] = {
{PB6,GPIO_AF_0,1,1},
{PB7,GPIO_AF_0,0,1},
{PB8,GPIO_AF_0,0,1},
{PB9,GPIO_AF_0,1,1},
{PD0,GPIO_AF_3,1,1},
{PD1,GPIO_AF_3,0,1},
{PA2,GPIO_AF_1,1,2},
{PA3,GPIO_AF_1,0,2},
{PA15,GPIO_AF_1,0,2},
};

const MM32TIMBK ocps[TIMBKCOUNT] = {
{PA6,GPIO_AF_2},
{PA11,GPIO_AF_6},
{PB12,GPIO_AF_2},
{PD1,GPIO_AF_0},
};

const MM32I2C i2cs[I2CCOUNT] = {    //tables 6-9 (PA9/PA10, PB13/PB14: bridge)
{PA11,GPIO_AF_5,1},
{PB6,GPIO_AF_1,1},
{PB8,GPIO_AF_1,1},
{PB10,GPIO_AF_1,1},
{PD1,GPIO_AF_1,1},
{PD2,GPIO_AF_1,1},
{PA12,GPIO_AF_5,0},
{PB7,GPIO_AF_1,0},
{PB9,GPIO_AF_1,0},
{PB11,GPIO_AF_1,0},
{PD0,GPIO_AF_1,0},
{PD3,GPIO_AF_1,0},
};

const MM32BUZ buzs[BUZCOUNT] = {
{PA0,GPIO_AF_6,BZ_TIM14},
{PA4,GPIO_AF_4,BZ_TIM14},
{PA7,GPIO_AF_4,BZ_TIM14},
{PB1,GPIO_AF_0,BZ_TIM14},
{PA6,GPIO_AF_5,BZ_TIM16},
{PB8,GPIO_AF_2,BZ_TIM16},
{PB6,GPIO_AF_2,BZ_TIM16+BZ_N},
{PA7,GPIO_AF_5,BZ_TIM17},
{PB9,GPIO_AF_2,BZ_TIM17},
{PD7,GPIO_AF_6,BZ_TIM17},
{PB7,GPIO_AF_2,BZ_TIM17+BZ_N},
};


#else
//MM32SPIN05/06 and MM32F031 (DS_MM32F031xx_q 1.25: the same alternate functions, but no UART2 pins)

static const uint8_t pins[PINCOUNT] = {
P(IOA,0),
P(IOA,1),
P(IOA,2),
P(IOA,3),
P(IOA,4),
P(IOA,5),
P(IOA,6),
P(IOA,7),
P(IOA,11),
P(IOA,12),
P(IOA,13),
P(IOA,14),
P(IOA,15),
P(IOB,0),
P(IOB,1),
P(IOB,2),
P(IOB,3),
P(IOB,4),
P(IOB,5),
P(IOB,6),
P(IOB,7),
P(IOB,8),
P(IOB,9),
P(IOB,10),
P(IOB,11),
P(IOB,12),
P(IOC,13),
P(IOC,14),
P(IOC,15),
P(IOD,0),
P(IOD,1),
P(IOD,2),
P(IOD,3),
};

const MM32ADC adcs[ADCCOUNT] = {
{PA0,ADC_Channel_0},
{PA1,ADC_Channel_1},
{PA2,ADC_Channel_2},
{PA3,ADC_Channel_3},
{PA4,ADC_Channel_4},
{PA5,ADC_Channel_5},
{PA6,ADC_Channel_6},
{PA7,ADC_Channel_7},
{PB0,ADC_Channel_8},
{PB1,ADC_Channel_9},
};

const MM32TIM23 halltims[TIMCOUNT] = {
{PA0,GPIO_AF_2,HT(1,1)},    //DS_MM32SPIN05 1.24 / DS_MM32F031xx_q 1.25 tables 6-8; a pin on several channels
{PA0,GPIO_AF_4,HT(1,3)},    //has one entry each (HallTimEntries() picks the combination)
{PA1,GPIO_AF_2,HT(1,2)},
{PA2,GPIO_AF_2,HT(1,3)},
{PA5,GPIO_AF_2,HT(1,1)},
{PA15,GPIO_AF_2,HT(1,1)},
{PB3,GPIO_AF_7,HT(1,1)},
{PB3,GPIO_AF_2,HT(1,2)},
{PB3,GPIO_AF_4,HT(1,3)},
{PB4,GPIO_AF_7,HT(1,2)},
{PB5,GPIO_AF_7,HT(1,3)},
{PB6,GPIO_AF_4,HT(1,1)},
{PB10,GPIO_AF_2,HT(1,3)},
{PC13,GPIO_AF_6,HT(1,1)},
{PC14,GPIO_AF_6,HT(1,2)},
{PC15,GPIO_AF_6,HT(1,3)},
{PA6,GPIO_AF_1,HT(0,1)},
{PA7,GPIO_AF_1,HT(0,2)},
{PB0,GPIO_AF_1,HT(0,3)},
{PB4,GPIO_AF_1,HT(0,1)},
{PB5,GPIO_AF_1,HT(0,2)},
};

const MM32UART uarts[UARTCOUNT] = {
{PB3,GPIO_AF_3,1,1},
{PB4,GPIO_AF_3,0,1},
{PB6,GPIO_AF_0,1,1},
{PB7,GPIO_AF_0,0,1},
#ifndef MCU_NO_UART2
{PA2,GPIO_AF_1,1,2},
{PA3,GPIO_AF_1,0,2},
{PA6,GPIO_AF_3,0,2},
{PA15,GPIO_AF_1,0,2},
{PB7,GPIO_AF_4,1,2},
{PB8,GPIO_AF_4,0,2},
#endif
};

const MM32TIMBK ocps[TIMBKCOUNT] = {
{PA4,GPIO_AF_3},
{PA6,GPIO_AF_2},
{PA13,GPIO_AF_7},
{PB12,GPIO_AF_2},
};

const MM32I2C i2cs[I2CCOUNT] = {    //tables 6-9 (PA9/PA10, PB13/PB14: bridge)
{PA5,GPIO_AF_5,1},
{PA11,GPIO_AF_5,1},
{PB6,GPIO_AF_1,1},
{PB8,GPIO_AF_1,1},
{PB10,GPIO_AF_1,1},
{PD1,GPIO_AF_1,1},
{PA4,GPIO_AF_5,0},
{PA12,GPIO_AF_5,0},
{PB7,GPIO_AF_1,0},
{PB9,GPIO_AF_1,0},
{PB11,GPIO_AF_1,0},
{PD0,GPIO_AF_1,0},
};

const MM32BUZ buzs[BUZCOUNT] = {
{PA4,GPIO_AF_4,BZ_TIM14},
{PA7,GPIO_AF_4,BZ_TIM14},
{PB1,GPIO_AF_0,BZ_TIM14},
{PA6,GPIO_AF_5,BZ_TIM16},
{PB8,GPIO_AF_2,BZ_TIM16},
{PB6,GPIO_AF_2,BZ_TIM16+BZ_N},
{PA7,GPIO_AF_5,BZ_TIM17},
{PB9,GPIO_AF_2,BZ_TIM17},
{PB7,GPIO_AF_2,BZ_TIM17+BZ_N},
};


#endif



//as the vendor GPIO_Init() (50 MHz outputs) for one pin; the pull-up / pull-down through BSRR / BRR
void gpioMode(GPIO_TypeDef* gpio, uint8_t pin, uint8_t mode){
	uint32_t dat = (mode&0x0Fu)|((mode&0x10u) ? GPIO_Speed_50MHz : 0);
	volatile u32* reg = pin<8 ? &gpio->CRL : &gpio->CRH;
	uint8_t sh = (uint8_t)((pin&7)*4);
	*reg = (*reg&~(0xFu<<sh))|(dat<<sh);
	if(mode==GPIO_Mode_IPD) gpio->BRR = 1u<<pin;
	else if(mode==GPIO_Mode_IPU) gpio->BSRR = 1u<<pin;
}

void gpioAF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af){    //as GPIO_PinAFConfig()
	volatile u32* reg = pin<8 ? &gpio->AFRL : &gpio->AFRH;
	uint8_t sh = (uint8_t)((pin&7)*4);
	*reg = (*reg&~(0xFu<<sh))|((uint32_t)af<<sh);
}

//io >= PINCOUNT (65535: not configured) is ignored / reads 0
void pinMode(uint8_t io,uint8_t state){
	if(io<PINCOUNT) gpioMode(ports[pins[io]>>4], pins[io]&15, state);
}

void pinModeAF(uint8_t io,uint8_t af){
	if(io<PINCOUNT) gpioAF(ports[pins[io]>>4], pins[io]&15, af);
}

uint8_t PinPortBit(uint8_t io){    //port (0 = A) << 4 | pin number
	return io<PINCOUNT ? pins[io] : 0xFF;
}

uint8_t digitalRead(uint8_t io){
	if(io>=PINCOUNT) return 0;
	return (uint8_t)((ports[pins[io]>>4]->IDR>>(pins[io]&15))&1);
}

void digitalWrite(uint8_t io, uint8_t state){
	if(io>=PINCOUNT) return;
	GPIO_TypeDef* g = ports[pins[io]>>4];
	if(state) g->BSRR = 1u<<(pins[io]&15); else g->BRR = 1u<<(pins[io]&15);
}

uint16_t analogRead(uint8_t io){    //the last conversion of the pin's channel; 0 without one (channels 0-9)
	for(uint8_t i=0;i<ADCCOUNT;i++){
		if(adcs[i].io==io){
			return adcs[i].channel<=ADC_Channel_9 ? (uint16_t)(&ADC1->ADDR0)[adcs[i].channel] : 0;
		}
	}
	return 0;
}
