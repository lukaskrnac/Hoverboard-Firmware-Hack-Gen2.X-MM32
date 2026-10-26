/*-------------------- Includes -----------------------*/
#include "../Inc/hallhandle.h"
#include "../Inc/main.h"
#include "../Inc/interrupt.h"
#include "../Inc/bldc.h"
#include "../Inc/mcu.h"

/*------------------- Private variables ---------------*/
HALLType HALL1;
int16_t CWShift = 0;
/*------------------ Private functions ----------------*/
void HALLModuleInit(HALLType *u);
void HALLModuleCalc(HALLType *u);


/****************************************************************
	Function Name��HALLModuleInit
	Description��Initialize HALLModule Parameter
	Input��none
	Output��none
****************************************************************/
void HALLModuleInit(HALLType *u)
{
	uint8_t i;
	
	u->RunHallValue = hallpos(dir);
	u->PreHallValue = u->RunHallValue;
	u->PreRaw = hallpos(1);
	u->CMDDIR = 1;
	u->IncAngle = 5;
	u->IncAngleMax = 10922;
	u->SpeedTemp = 0;
	u->Time100msCNT = 0;
	u->HallTimeSum = 60000;
	
	u->CWAngleTab[5] = 32767;
	u->CWAngleTab[1] = 43688;
	u->CWAngleTab[3] = 54610;
	u->CWAngleTab[2] = 0;
	u->CWAngleTab[6] = 10922;
	u->CWAngleTab[4] = 21844;



		u->Angle = u->CWAngleTab[HALL1.RunHallValue] + CWShift;
	
	for(i=0;i<8;i++)
	{
		u->HallTime[i] = 10000;
	}
}

/****************************************************************
	Function Name��HALLModuleCalc
	Description��Update the rotor electrical angle and motor speed
	Input��HALL--Structure HALLType
	Output��none
****************************************************************/
uint32_t ChangeTime = 0;

void HALLModuleCalc(HALLType *u){
	static uint8_t i = 0;
	uint8_t j = 0;
	uint8_t raw = hallpos(1);    //hall state C*4+B*2+A, independent of the commanded direction
	uint8_t com = DRIVEMODE==COM_VOLT||DRIVEMODE==COM_SPEED;

	//RunHallValue is the hall state as seen by the commutation: complemented (7-raw) for the other direction
	u->RunHallValue = u->CMDDIR==1 ? raw : 7-raw;
	
	//measurement: only real hall edges. A change of the commanded direction also changes RunHallValue;
	//counting that as an edge made a fake odometer step and a speed sample from a stale timer capture.
	if(raw!=u->PreRaw && hall_to_pos[raw] && hall_to_pos[u->PreRaw]){
		uint8_t d = (hall_to_pos[raw]+6-hall_to_pos[u->PreRaw])%6;    //1 or 5 for adjacent positions
		TIM_TypeDef* tim = halltimen ? TIM2 : TIM3;
		i++;
		if(i >= 6)
		{
			i = 0;
		}
		if(d==1){
			realdir=0;
		}else if(d==5){
			realdir=1;
		}
		u->HallTime[i] = tim->CCR1;    //us since the previous edge (the timer is reset by every edge)
		if(tim->SR&TIM_SR_UI){         //16 bit timer overflowed: more than 65 ms, the capture wrapped
			tim->SR = ~TIM_SR_UI;
			u->HallTime[i] = 250000;
		}
		ChangeTime = u->HallTime[i];
				
		//Calculate rotor electrical angle speed and mechanical speed
		u->HallTimeSum = 0;
		for(j=0;j<6;j++)
		{
			u->HallTimeSum += u->HallTime[j];
		}
		
		u->SpeedTemp = SPEEDX/u->HallTimeSum;
		if(realdir){
			realspeed = u->SpeedTemp;
		}else{
			realspeed = -u->SpeedTemp;
		}
		lastcommutate = millis;	
		if (realdir == 1){
			iOdom++;
		}else{
			iOdom--;
		}
	}
	
	//sine: re-anchor the electrical angle at a hall edge or a direction change. Unit of IncAngle is 62.5us.
	if(!com&&u->PreHallValue != u->RunHallValue){
		u->IncAngle = 4096000/u->HallTimeSum;
		u->IncAngleMax = 10922;
		u->Angle = u->CWAngleTab[u->RunHallValue] + CWShift;
	}
	//block commutation: evaluated every time, so a direction change or a mode switch takes effect at
	//once instead of at the next hall edge
	if(com&&hall_to_pos[u->RunHallValue]!=step){
		step=hall_to_pos[u->RunHallValue];
		commutate();
	}
	
	if(millis-lastcommutate>250){//zero out speed
		realspeed=0;
	}
	u->PreHallValue = u->RunHallValue;
	u->PreRaw = raw;

	//Incremental value limit
	if(((u->IncAngleMax - u->IncAngle)>=0)&&!(DRIVEMODE==COM_VOLT||DRIVEMODE==COM_SPEED)){
		u->IncAngleMax = u->IncAngleMax - u->IncAngle;
		u->Angle = u->Angle + u->CMDDIR * u->IncAngle;
	}

}

