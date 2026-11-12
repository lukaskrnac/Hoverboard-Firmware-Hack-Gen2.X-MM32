//PinFinder hardware: time, adc (software started scans), the bridge (block steps), pin helpers.
#include "pf.h"
#include "commutation.h"
#include "main.h"
#include "initialize.h"
#include "hal_adc.h"
#include "hal_tim.h"
#include <string.h>

uint8_t uartUsed = 1;
//the pins PinFinder runs on: SWD, its uart TX and RX (set at boot) and the latch it drives (LatchHold). Never
//free, whatever the words say after set / revert / defaults: a detection that took one cut the link or the power
uint16_t ownPins[OWN_PINS] = { PA13, PA14, PIN_NONE, PIN_NONE, PIN_NONE };

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

void WaitUs(uint32_t us){    //SysTick counts down from SystemCoreClock/1000 - 1
	uint32_t load = SysTick->LOAD+1, last = SysTick->VAL, n = us*(SystemCoreClock/1000000);
	while(n){
		uint32_t v = SysTick->VAL;
		uint32_t d = last>=v ? last-v : last+load-v;
		last = v;
		n = d>=n ? 0 : n-d;
	}
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

//only free pins and the analog inputs: the analog mode on a pin in use takes it from its function (uart AF,
//latch output, hall pull-up). A free pin returns to its rest state after the reading (the pull-up that may hold
//an unknown latch); it settles first: the rest pull-up charged a high impedance divider's capacitor
int32_t PinMv(uint16_t pin){
	int8_t ch = AdcChannel(pin);
	uint8_t isFree = PinFree(pin);
	if(ch<0||!(isFree||pin==VBATPIN||pin==ITOTALPIN||pin==IPHASEAPIN||pin==IPHASEBPIN)) return -1;
	pinMode((uint8_t)pin, INPUT_ADC);
	if(isFree) Wait(20);
	AdcScan(1u<<15|1u<<ch);
	if(isFree) PinRest(pin);
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

//bldc.c's commutate() (commutation.h): the high side chops, the low side stays on for the step
static const uint16_t blockCcer[7] = BLOCK_CCER_INIT;

static void outputs(uint32_t ccer, uint16_t duty){
	const uint32_t all = TIM_CCER_CC1EN|TIM_CCER_CC1NEN|TIM_CCER_CC2EN|TIM_CCER_CC2NEN|TIM_CCER_CC3EN|TIM_CCER_CC3NEN;
	TIM1->CCMR1 = (TIM1->CCMR1&~(TIM_CCMR1_OC1M|TIM_CCMR1_OC2M))|BlockOcMode(ccer, 0)|BlockOcMode(ccer, 1)<<8;
	TIM1->CCMR2 = (TIM1->CCMR2&~TIM_CCMR2_OC3M)|BlockOcMode(ccer, 2);
	TIM1->CCER = (TIM1->CCER&~all)|ccer|BLOCK_CCER_LOW;
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = duty;
	TIM1->EGR = TIM_EventSource_COM;
	TIM1->BDTR |= TIM_BDTR_MOEN;
}

void BlockStep(uint8_t s, uint16_t duty){
	outputs(s<7 ? blockCcer[s] : 0, duty);
}

//Static fields with all six switches complementary (the main firmware's sine / FOC output, PWM2: the low
//sides conduct around the counter's zero, where the adc samples), so the phase current is continuous. mv
//is the phase voltage amplitude.
uint16_t fieldCcr[3];    //the last FieldVector()'s compare values (phases A, B, C)

//sin(a) in Q15, a: 65536 per turn (quarter table, linear interpolation: < 0.01 % error)
int32_t Sin16(uint16_t a){
	static const int16_t q[65] = { 0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512, 10278,
		11039, 11793, 12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787,
		21403, 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510,
		28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412,
		32521, 32609, 32678, 32728, 32757, 32767 };
	uint16_t x = a&0x3FFF;
	if(a&0x4000) x = (uint16_t)(0x4000-x);
	int32_t i = x>>8, f = x&255;
	int32_t v = i>=64 ? q[64] : q[i]+((q[i+1]-q[i])*f>>8);
	return (a&0x8000) ? -v : v;
}

//field at angle th (65536 per electrical turn: the main firmware's frame) with all six switches
//complementary: phase X duty 1/2 + mv/vbat x cos(th - 120 X deg)
void FieldAngle(uint16_t th, int32_t mv){
	static int32_t vb;    //the battery voltage, measured every 100 ms (the kv drive calls this in a loop)
	static uint32_t vbT;
	if(!vb||millis-vbT>100){ vb = VbatMv(); vbT = millis; }
	int32_t arr = PWM_PERIOD;
	if(vb<10000||vb>60000) vb = 42000;
	if(mv>vb/4) mv = vb/4;
	volatile u32* ccr[3] = { &TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3 };
	for(uint8_t x=0;x<3;x++){
		int32_t c = Sin16((uint16_t)(th+16384-x*21845));
		fieldCcr[x] = (uint16_t)(arr/2-arr*mv/vb*c/32768);
		*ccr[x] = fieldCcr[x];
	}
	const uint32_t all = TIM_CCER_CC1EN|TIM_CCER_CC1NEN|TIM_CCER_CC2EN|TIM_CCER_CC2NEN|TIM_CCER_CC3EN|TIM_CCER_CC3NEN;
	TIM1->CCMR1 = (TIM1->CCMR1&~(TIM_CCMR1_OC1M|TIM_CCMR1_OC2M))|TIM_OCMode_PWM2|TIM_OCMode_PWM2<<8;
	TIM1->CCMR2 = (TIM1->CCMR2&~TIM_CCMR2_OC3M)|TIM_OCMode_PWM2;
	TIM1->CCER = (TIM1->CCER&~all)|all;
	TIM1->EGR = TIM_EventSource_COM;
	TIM1->BDTR |= TIM_BDTR_MOEN;
}

//direction k of 12: 210 + 30 k deg (even k: the direction of block step k/2+1)
void FieldVector(uint8_t k, int32_t mv){
	FieldAngle((uint16_t)((uint32_t)((7+k)%12)*65536/12), mv);
}

uint16_t DutyForMv(int32_t mv){
	int32_t vb = VbatMv();
	if(vb<10000||vb>60000) vb = 42000;    //no battery pin: assume a full 10S pack (the lowest duty)
	int32_t d = mv*(int32_t)PWM_PERIOD/vb;
	return (uint16_t)(d<1 ? 1 : (d>PWM_PERIOD/2 ? PWM_PERIOD/2 : d));
}

uint8_t PinFree(uint16_t pin){
	for(uint8_t i=0;i<OWN_PINS;i++) if(ownPins[i]==pin) return 0;
	return pin<PINCOUNT&&CfgPinWord(pin, 0xFF)<0;
}

uint8_t HallState(void){    //as the main firmware reads it (hallpos(1)): inverted with hallinvert
	uint8_t h = (uint8_t)(digitalRead(HALLCPIN)*4+digitalRead(HALLBPIN)*2+digitalRead(HALLAPIN));
	return cfgBool(HALL_INVERT, 0) ? (uint8_t)(7-h) : h;
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

//n / d with a 64 bit dividend, truncated towards zero (the quotient must fit in 32 bits). Shift and subtract:
//PinFinder has no floats (the soft float library was ~0.8 KB of its 31 KB) and libgcc's 64 bit division is large
int32_t Div64(int64_t n, int32_t d){
	uint8_t neg = (n<0)!=(d<0);
	uint64_t u = n<0 ? 0u-(uint64_t)n : (uint64_t)n, r = 0, q = 0;
	uint32_t v = d<0 ? 0u-(uint32_t)d : (uint32_t)d;
	if(!v) return 0;
	for(uint8_t i=0;i<64;i++){
		r = r<<1|u>>63;
		u <<= 1;
		q <<= 1;
		if(r>=v){ r -= v; q |= 1; }
	}
	return neg ? -(int32_t)q : (int32_t)q;
}

int32_t MulDiv(int32_t a, int32_t b, int32_t c){ return Div64((int64_t)a*b, c); }
