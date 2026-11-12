/*-------------------- Includes -----------------------*/
#include "mcu.h"
#include "pwm_gen.h"
#include "pinout.h"

/*------------------- Private variables ---------------*/
PWM_GEN_T pwm_gen;

/*------------------ Private functions ----------------*/
void PWM_GEN_init(PWM_GEN_T *u1);
void PWM_GEN_calc(PWM_GEN_T *u1);
void Update_PWM(PWM_GEN_T *u1);

/*************************************
	Function Name：PWM_GEN_init
	Description：It initializes SVPWM structure
	Input：*u1 --- Structure PWM_GEN_T
	Output：none
**************************************/
void PWM_GEN_init(PWM_GEN_T *u1)
{
	u1->Alpha = 0;
	u1->Beta	= 0;
	u1->CompA	= 0;
	u1->CompB	= 0;
	u1->CompC	= 0;
	u1->Mode	= SEVENMODE;
	u1->Sector	= 0;
	u1->N_halfPeriod	= PWM_PERIOD;
}

//Space vector PWM (seven segment): alpha / beta (Q15) -> compare values. Tables for the sector cases of the
//vendor version (and its unused five segment mode removed; 0.3 KB less); identical outputs, checked on the
//host for 50e6 alpha / beta / period combinations. Sector 0 / 7 (alpha = beta = 0): 50 %, not 0: in PWM2 mode
//a compare of 0 keeps all high sides on, no low side time for the current sense and the bootstrap capacitors.
static int32_t dutyOf(int32_t t, int32_t n){    //Q15 time -> compare value, 0 .. n-1
	t = t*n>>15;
	return t>=n ? n-1 : (t<0 ? 0 : t);
}
void PWM_GEN_calc(PWM_GEN_T *u1){
	//per sector: the two vector times as +-(1 + index) of X, Y, Z; which time goes to A, B, C (2 bits each)
	static const int8_t t1[6] = { 3, 2, -3, -1, 1, -2 }, t2[6] = { 2, -1, 1, 3, -2, -3 };
	static const uint8_t ord[6] = { 1|0<<2|2<<4, 0|2<<2|1<<4, 0|1<<2|2<<4, 2|1<<2|0<<4, 2|0<<2|1<<4, 1|2<<2|0<<4 };
	int32_t tA = (28378*u1->Alpha)>>15, tB = u1->Beta>>1, n = u1->N_halfPeriod;    //alpha*sqrt(3)/2, beta/2
	int32_t v[3] = { u1->Beta, tB+tA, tB-tA };    //X, Y, Z
	uint8_t sector = (uint8_t)((u1->Beta>=0)|(tA-tB>=0)<<1|(-tA-tB>=0)<<2);
	u1->Sector = sector;
	if(sector==0||sector==7){ u1->CompA = u1->CompB = u1->CompC = (int16_t)(n/2); return; }
	int32_t a = t1[sector-1], b = t2[sector-1];
	int32_t x = a>0 ? v[a-1] : -v[-a-1], y = b>0 ? v[b-1] : -v[-b-1];
	int32_t t[3];
	t[0] = (32768-x-y)>>1; t[1] = t[0]+x; t[2] = t[1]+y;    //(1 - t1 - t2)/2, + t1, + t2
	for(uint8_t k=0;k<3;k++) t[k] = dutyOf(t[k], n);
	uint8_t o = ord[sector-1];
	u1->CompA = (int16_t)t[o&3]; u1->CompB = (int16_t)t[o>>2&3]; u1->CompC = (int16_t)t[o>>4&3];
}

/****************************************************************
	Function Name：Update_PWM
	Description：Update duty cycle values
	Input: *u1 --- Structure PWM_GEN_T
	Output：none
****************************************************************/
void Update_PWM(PWM_GEN_T *u1)
{
	TIM1->CCR1 = u1->CompA;
	TIM1->CCR2 = u1->CompB;
	TIM1->CCR3 = u1->CompC;
}
