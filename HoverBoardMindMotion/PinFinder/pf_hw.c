//PinFinder hardware: time, adc (software started scans), the bridge (block steps), pin helpers.
#include "pf.h"
#include "commutation.h"
#include "main.h"
#include "initialize.h"
#include "hal_adc.h"
#include "hal_tim.h"
#include <string.h>

uint8_t uartUsed = 1;

//SysTick (1 ms) also feeds the watchdog: PinFinder blocks in long waits, and after a soft start from the
//main firmware the watchdog is running (it cannot be stopped), a watchdog reset powers a latched board off
void SysTick_Handler(void);
void SysTick_Handler(void){
	millis++;
	IWDG->KR = 0xAAAA;
}

void Wait(uint32_t ms){
	uint32_t t = millis;
	while(millis-t<ms);
}

//single period scan of the selected channels, started by software; 239.5 cycles sample time (the battery
//divider and the 1.2 V reference are high impedance): ~21 us per channel
void AdcInit(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC1->ADCFG = ADC_CFGR_PRE_6|ADC_CFGR_SAMCTL_239_5;
	ADC1->ADCR = ADC_CR_SCAN;
	ADC_VrefintCmd(ENABLE);
	ADC1->ADCFG |= ADC_CFGR_ADEN;
}

void AdcScan(uint32_t chmask){
	ADC1->ADCHS = chmask;
	ADC1->ADSTA = ADC_SR_ADIF;
	ADC1->ADCR |= ADC_CR_ADST;
	uint32_t t = millis;
	while(!(ADC1->ADSTA&ADC_SR_ADIF)&&millis-t<5);
	ADC1->ADSTA = ADC_SR_ADIF;
}

//one scan triggered by TIM1 CC4 (counter at 1: the middle of the low side time, where the main firmware
//samples the shunt and the phase current op-amps), short sample time
void AdcScanTriggered(uint32_t chmask){
	AdcScanAt(chmask, 1);
}

//the same at another counter value (CCR4; AdcScanTriggered() sets the main firmware's 1 again)
void AdcScanAt(uint32_t chmask, uint16_t cnt){
	uint32_t cfg = ADC1->ADCFG;
	if(TIM1->CCR4!=cnt){    //preloaded: the new value counts from the next update event, the old one may fire first
		TIM1->CCR4 = cnt;
		Wait(2);
	}
	ADC1->ADCFG = (cfg&~ADC_CFGR_SAMCTL)|ADC_CFGR_SAMCTL_7_5;
	ADC1->ADCHS = chmask;
	ADC1->ADSTA = ADC_SR_ADIF;
	ADC1->ADCR = (ADC1->ADCR&~ADC_CR_TRGSEL)|ADC_ExternalTrigConv_T1_CC4|ADC_CR_TRGEN;
	uint32_t t = millis;
	while(!(ADC1->ADSTA&ADC_SR_ADIF)&&millis-t<3);
	ADC1->ADCR &= ~ADC_CR_TRGEN;
	ADC1->ADSTA = ADC_SR_ADIF;
	ADC1->ADCFG = cfg;
}

uint16_t AdcValue(uint8_t ch){
	return (uint16_t)((&ADC1->ADDR0)[ch]&0xFFF);
}

int8_t AdcChannel(uint16_t pin){
	for(uint8_t i=0;i<ADCCOUNT;i++){
		if(adcs[i].io==pin) return (int8_t)adcs[i].channel;
	}
	return -1;
}

int32_t VddMv(void){
	AdcScan(1u<<15);
	uint32_t v = AdcValue(15);
	return v ? (int32_t)(1200u*4096u/v) : 0;
}

int32_t PinMv(uint16_t pin){
	int8_t ch = AdcChannel(pin);
	if(ch<0) return -1;
	pinMode((uint8_t)pin, INPUT_ADC);
	AdcScan(1u<<15|1u<<ch);
	uint32_t ref = AdcValue(15);
	return ref ? (int32_t)(AdcValue((uint8_t)ch)*1200u/ref) : 0;
}

int32_t VbatMv(void){
	if(VBATPIN>=PINCOUNT) return -1;
	return PinMv(VBATPIN)*(int32_t)VBAT_DIVIDER;
}

//TIM1 as the main firmware sets it up (centre aligned, PWM_PERIOD, low side polarity from the config), gates off
void MotorInit(void){
	BLDC_init();
	TIM1_init(8192, 0);
	GatesOff();
}

void GatesOff(void){
	TIM1->BDTR &= ~(TIM_BDTR_AOEN|TIM_BDTR_MOEN);
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
}

//bldc.c's commutate() (commutation.h): CCxNE alone follows OCxREF, so the low side is on together with the
//high side for the duty
static const uint16_t blockCcer[7] = BLOCK_CCER_INIT;

static void outputs(uint32_t ccer, uint16_t duty){
	const uint32_t all = TIM_CCER_CC1EN|TIM_CCER_CC1NEN|TIM_CCER_CC2EN|TIM_CCER_CC2NEN|TIM_CCER_CC3EN|TIM_CCER_CC3NEN;
	TIM1->CCMR1 = (TIM1->CCMR1&~(TIM_CCMR1_OC1M|TIM_CCMR1_OC2M))|TIM_OCMode_PWM1|TIM_OCMode_PWM1<<8;
	TIM1->CCMR2 = (TIM1->CCMR2&~TIM_CCMR2_OC3M)|TIM_OCMode_PWM1;
	TIM1->CCER = (TIM1->CCER&~all)|ccer;
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = duty;
	TIM1->EGR = TIM_EventSource_COM;
	TIM1->BDTR |= TIM_BDTR_MOEN;
}

void BlockStep(uint8_t s, uint16_t duty){
	outputs(s<7 ? blockCcer[s] : 0, duty);
}

//Field direction k x 30 deg el with all six switches complementary (the main firmware's sine / FOC output,
//PWM2: the low sides conduct around the counter's zero, where the adc samples), so the phase current is
//continuous: phase X duty 1/2 + mv/vbat x cos(theta - 120 X), theta = 210 + 30 k deg. mv is the phase
//voltage amplitude; even k is the direction of block step k/2+1.
uint16_t fieldCcr[3];    //the last FieldVector()'s compare values (phases A, B, C)

void FieldVector(uint8_t k, int32_t mv){
	static const int16_t cos12[12] = { 1000, 866, 500, 0, -500, -866, -1000, -866, -500, 0, 500, 866 };
	int32_t vb = VbatMv(), arr = PWM_PERIOD;
	if(vb<10000||vb>60000) vb = 42000;
	if(mv>vb/4) mv = vb/4;
	volatile u32* ccr[3] = { &TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3 };
	for(uint8_t x=0;x<3;x++){
		int32_t c = cos12[(7+k+12-4*x)%12];
		fieldCcr[x] = (uint16_t)(arr/2-arr*mv/vb*c/1000);
		*ccr[x] = fieldCcr[x];
	}
	const uint32_t all = TIM_CCER_CC1EN|TIM_CCER_CC1NEN|TIM_CCER_CC2EN|TIM_CCER_CC2NEN|TIM_CCER_CC3EN|TIM_CCER_CC3NEN;
	TIM1->CCMR1 = (TIM1->CCMR1&~(TIM_CCMR1_OC1M|TIM_CCMR1_OC2M))|TIM_OCMode_PWM2|TIM_OCMode_PWM2<<8;
	TIM1->CCMR2 = (TIM1->CCMR2&~TIM_CCMR2_OC3M)|TIM_OCMode_PWM2;
	TIM1->CCER = (TIM1->CCER&~all)|all;
	TIM1->EGR = TIM_EventSource_COM;
	TIM1->BDTR |= TIM_BDTR_MOEN;
}

uint16_t DutyForMv(int32_t mv){
	int32_t vb = VbatMv();
	if(vb<10000||vb>60000) vb = 42000;    //no battery pin: assume a full 10S pack (the lowest duty)
	int32_t d = mv*(int32_t)PWM_PERIOD/vb;
	return (uint16_t)(d<1 ? 1 : (d>PWM_PERIOD/2 ? PWM_PERIOD/2 : d));
}

uint8_t PinFree(uint16_t pin){
	if(pin>=PINCOUNT||pin==PA13||pin==PA14) return 0;    //SWD
	for(uint8_t i=0;i<=17;i++){
		if(pinstorage[i]==pin) return 0;
	}
	return OCPPIN!=pin&&OCPREFPIN!=pin;
}

uint8_t HallState(void){
	return (uint8_t)(digitalRead(HALLCPIN)*4+digitalRead(HALLBPIN)*2+digitalRead(HALLAPIN));
}

const char* PinName(uint16_t pin){
	static char b[5];
	if(pin>=PINCOUNT) return "none";
	uint8_t pb = PinPortBit((uint8_t)pin);
	b[0] = 'P';
	b[1] = (char)('A'+(pb>>4));
	uint8_t n = pb&15;
	if(n>=10){ b[2] = '1'; b[3] = (char)('0'+n-10); b[4] = 0; }
	else{ b[2] = (char)('0'+n); b[3] = 0; }
	return b;
}

uint16_t PinByName(const char* s){
	char u[5];
	uint8_t i;
	for(i=0;i<4&&s[i];i++) u[i] = (char)(s[i]>='a'&&s[i]<='z' ? s[i]-32 : s[i]);
	u[i] = 0;
	if(s[i]) return PIN_NONE;
	for(uint16_t p=0;p<PINCOUNT;p++){
		if(!strcmp(u, PinName(p))) return p;
	}
	return PIN_NONE;
}
