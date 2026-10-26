#include "../Inc/mcu.h"
#include "../Inc/main.h"
#include "../Inc/interrupt.h"
#include "hal_gpio.h"
#include "../Inc/pinout.h"
#include "../Inc/remoteUartBus.h"
#include "../Inc/bldc.h"
#include "../Inc/calculation.h"
#include "hal_tim.h"
#include "hal_conf.h"
#include "math.h"
#include "../Inc/hallhandle.h"

volatile uint16_t abspwm;
volatile uint32_t lastcommutate;
volatile uint8_t gatesoff=0;
uint32_t lasttestrotate;
int testrotatedir=1;
uint8_t hallpos(uint8_t dir);
bool lastdir=0;
uint8_t lowbatcount=0;
uint8_t lowbatperm=0;

const uint8_t hall_to_pos[8] =
{
	// annotation: for example SA=0 means hall sensor pulls SA down to Ground
  0, // hall position [-] - No function (access from 1-6) 
  3, // hall position [1] (SA=1, SB=0, SC=0) -> PWM-position 3
  5, // hall position [2] (SA=0, SB=1, SC=0) -> PWM-position 5
  4, // hall position [3] (SA=1, SB=1, SC=0) -> PWM-position 4
  1, // hall position [4] (SA=0, SB=0, SC=1) -> PWM-position 1
  2, // hall position [5] (SA=1, SB=0, SC=1) -> PWM-position 2
  6, // hall position [6] (SA=0, SB=1, SC=1) -> PWM-position 6
  0, // hall position [-] - No function (access from 1-6) 
};


uint8_t hallpos(uint8_t dir){
	uint8_t HallA;
	uint8_t HallB;
	uint8_t HallC;
	uint8_t HallValue;
	
	HallA = digitalRead(HALLAPIN);
	HallB = digitalRead(HALLBPIN);
	HallC = digitalRead(HALLCPIN);
	
	if(dir == 1)
	{
		HallValue = HallC*4 + HallB*2 + HallA;
	}
	else
	{
		HallValue = 7-(HallC*4 + HallB*2 + HallA);
	}
	
	//return hall_to_pos[HallValue];
	return HallValue;
}

void commutate(){//U,V,W(A,B,C)

	if(step == 1) {//-1,0,1
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);	
	}else if(step == 2) {//0,-1,1
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);		
	}else if(step == 3) {//1,-1,0
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); 			
	}else if(step == 4) {//1,0,-1
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	}else if(step == 5) {//0,1,-1
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);	
	}else if(step == 6) {//-1,1,0
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); 
	}else{//0,0,0
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	}

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);//apply changes
}




void speedupdate(){
	if(poweron>127){
		RemoteUpdate();	
		//speed=200;
		if(BAT_EMPTY>20000&&BAT_EMPTY<65000&&vbat*10<BAT_EMPTY){
			if(lowbatcount<10){
				lowbatcount++;
			}else if(lowbatcount>=10&&speed==0&&realspeed<100){
				lowbatperm=1;
			}
		}else{
			lowbatcount=0;
		}
		gatesoff = lowbatperm || (wState & STATE_DISABLE);
		if(gatesoff){
			pwm=0;
			PIDrst();    //no wind-up while the gates are off
			SpeedCtrlReset();
		}else{
			if(DRIVEMODE==COM_VOLT){
				pwm=speed*PWM_RES/1000;//1000~-1000 for all pwm resolution
			}else if(DRIVEMODE==COM_SPEED){	
				if(speed==0){    //coast
					pwm=0;
					SpeedCtrlReset();
				}else{
					pwm=SpeedCtrl(speed,realspeed);    //speed in rpm
				}
			}else if(DRIVEMODE==SINE_VOLT){	
				pwm= speed*40;// IQ=30000~-30000
			}else if(DRIVEMODE==SINE_SPEED){	
				pwm= PID2PWM((PID(speed,realspeed))/50);// command the required RPM
				if(speed==0){
					pwm=0;
					PIDrst();
				}
			}
		}
		if(pwm>0){
		dir=1;
		}else{
		dir=-1;
		}
		abspwm=pwm<0 ? -pwm : pwm;    // 4095~-4095
		if(DRIVEMODE==COM_VOLT||DRIVEMODE==COM_SPEED){
			TIM1->CCR1=abspwm;
			TIM1->CCR2=abspwm;
			TIM1->CCR3=abspwm;
		}
	}
}

//TIM1 main output enable for all six gates. AOEN goes with MOEN: with automatic output enable set,
//the timer sets MOEN again at the next update event and a software "off" lasts only one pwm period.
void gatesOutput(uint8_t on){
	if(on){
		TIM1->BDTR |= TIM_BDTR_AOEN|TIM_BDTR_MOEN;
	}else{
		TIM1->BDTR &= ~(TIM_BDTR_AOEN|TIM_BDTR_MOEN);
	}
}

void TIMOCInit(){
	if(DRIVEMODE==COM_VOLT||DRIVEMODE==COM_SPEED){
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		commutate();
	}else{
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM2);
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM2);
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM2);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	
	}
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}