/*-------------------- Includes -----------------------*/
#include "hallhandle.h"
#include "main.h"
#include "pinout.h"
#include "interrupt.h"
#include "bldc.h"
#include "mcu.h"

/*------------------- Private variables ---------------*/
HALLType HALL1;
volatile int32_t fastspeed;
volatile uint8_t hallAngleSure;    //the angle is known: a regular edge, its interpolation not run out
static uint32_t pllTh, pllBnd;     //angle tracker: angle, the boundary of the last edge (2^32 per electrical turn)
static int32_t pllW;               //its speed, angle per pwm period, signed
static uint8_t pllOn;
static uint32_t edgeMs;    //millis at the last hall edge: interval timing past the 65 ms of the us timer
static volatile int16_t CWShift = 0;    //extra sine angle advance in the direction of motion (65536 = 360 deg el), tuning over SWD
/*------------------ Private functions ----------------*/
void HALLModuleInit(HALLType *u);
void HALLModuleCalc(HALLType *u);


/****************************************************************
	Function Name：HALLModuleInit
	Description：Initialize HALLModule Parameter
	Input：none
	Output：none
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



	u->MoveDir = 1;
	u->Angle = (int16_t)u->CWAngleTab[u->PreRaw];    //middle of the current sector

	for(i=0;i<8;i++)
	{
		u->HallTime[i] = 10000;
	}
}

/****************************************************************
	Function Name：HALLModuleCalc
	Description：Update the rotor electrical angle and motor speed
	Input：HALL--Structure HALLType
	Output：none
****************************************************************/

void HALLModuleCalc(HALLType *u){
	static uint8_t i = 0;
	uint8_t j = 0;
	uint8_t raw = hallpos(1);    //hall state C*4+B*2+A, independent of the commanded direction
	uint8_t com = COMMUTATION(DRIVEMODE)==COMMUT_BLOCK;

	//RunHallValue is the hall state as seen by the commutation: complemented (7-raw) for the other direction
	u->RunHallValue = u->CMDDIR==1 ? raw : 7-raw;

	//measurement: only real hall edges. A change of the commanded direction also changes RunHallValue;
	//counting that as an edge made a fake odometer step and a speed sample from a stale timer capture.
	uint8_t edge = 0;
	uint32_t since = 0;    //us from the hall edge to its detection here
	if(raw!=u->PreRaw && hall_to_pos[raw] && hall_to_pos[u->PreRaw]){
		uint8_t d = (hall_to_pos[raw]+6-hall_to_pos[u->PreRaw])%6;    //1 or 5 for adjacent positions
		edge = 1;
		if(d==1){
			u->MoveDir = 1;     //electrical angle rising: hall states 2,6,4,5,1,3
		}else if(d==5){
			u->MoveDir = -1;
		}
		TIM_TypeDef* tim = halltimen ? TIM2 : TIM3;
		uint8_t reversal = (d==1) != (realdir==0);    //against the previous edge's direction
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
		since = tim->CNT;              //so this is the time since this edge
		if(since>456) since = 456;
		if(tim->SR&TIM_SR_UI){         //16 bit timer overflowed: more than 65 ms, the capture wrapped;
			tim->SR = ~TIM_SR_UI;      //ms timing instead (it was a fixed 250 ms: below ~10 rpm the speed
			uint32_t ms = millis-edgeMs;    //read 0 or far too low)
			u->HallTime[i] = ms<250 ? ms*1000 : 250000;
		}
		if(reversal){
			//a direction change is no speed sample: a rotor resting on a hall boundary (position hold)
			//flickers the bit, the short intervals read as up to 89 rpm in alternating directions and the
			//FOC speed loop answered with 6 A kicks. Start the averaging over from standstill.
			for(j=0;j<6;j++) u->HallTime[j] = 250000;
		}
		edgeMs = millis;

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

	//sine: rotor angle (65536 per electrical turn) from the hall sectors. Measured on the bench motor
	//(d-axis vector swept over two turns): the hall states are centred on CWAngleTab, their boundaries
	//at +-30 deg (5461), within 1.5 deg. At an edge the rotor is on the boundary just crossed; between
	//edges the angle is interpolated at the measured speed, at most one sector. The old code put the
	//angle on the sector centre at the edge (30 deg ahead) and interpolated as if this ran every 62.5 us
	//instead of every pwm period (2*PWM_RES/72 MHz = 227.6 us), so the angle lagged and jumped.
	if(!com){
		if(edge&&u->HallTimeSum>=6*65535){
			//no speed known (below ~10 rpm, after a reversal): the middle of the sector, error <= 30 deg.
			//Starting at the boundary with a crawling interpolation the error grew to 60 deg until the next
			//edge snapped it: in FOC a current transient that tripped (position hold near its target).
			u->Angle = (int16_t)u->CWAngleTab[raw];
			u->IncAngleMax = 0;
			hallAngleSure = 0;
			pllOn = 0;
		}else if(edge){
			//an alpha-beta tracker (a PLL on the hall edges): angle and speed run on every period; an
			//edge corrects the angle by 1/8 of the error to its boundary and the speed by 1/64 of it per
			//sector (1/4..1/32 per edge measured: 1/8 and 1/16 shook least, 1/8 follows faster). The edge is seen here up to a period late: its boundary plus the time since (the
			//timer count). Snapping the angle to every edge (as before) passed the edge jitter (+-20 %
			//of an interval with field weakening current) straight into the angle, and a d current turns
			//angle noise into torque (|id| * sin(error)): 20-60 Hz shaking with field weakening that grew
			//with the d current. The rotor (inertia) turns smoothly: the edges correct, they do not set.
			int32_t s = u->MoveDir;
			uint32_t ivl = u->HallTime[i];    //us
			uint32_t k = (uint32_t)PWM_RES*16384/54;    //65536*2*PWM_RES/72/6 / interval: per period, 65536 per turn
			uint32_t q = k/ivl, r = k%ivl;    //(k<<16)/ivl in 32 bits: 2^32 per turn
			int32_t wMeas = s*(int32_t)((q<<16)+(((r<<8)/ivl)<<8));
			uint32_t bnd = (uint32_t)(uint16_t)(u->CWAngleTab[raw]-s*5461)<<16;
			uint32_t meas = bnd+(uint32_t)(wMeas/228*(int32_t)since);
			int32_t e = (int32_t)(meas-pllTh);
			if(!pllOn||(pllW>0)!=(s>0)||e>(2731<<16)||e<-(2731<<16)){    //start, reversal, > 15 deg off
				pllTh = meas; pllW = wMeas; pllOn = 1;
			}else{
				int32_t n = (int32_t)(ivl/228);
				pllTh += (uint32_t)(e/8);
				pllW += e/64/(n<1 ? 1 : n);
			}
			pllBnd = bnd;
			hallAngleSure = 1;
		}else if(millis-lastcommutate>250){    //standing: middle of the sector
			u->Angle = (int16_t)u->CWAngleTab[raw];
			u->IncAngleMax = 0;
			hallAngleSure = 0;
			pllOn = 0;
		}else if(pllOn){
			//at most one sector (and a period) past the last edge: no edge where one was due means the
			//rotor is slower; the angle waits there and the d current is off until the next edge (a
			//negative d current in a frame ahead of the rotor brakes it, it stalled and rocked)
			uint32_t next = pllTh+(uint32_t)pllW;
			int32_t travel = (int32_t)(next-pllBnd), aw = pllW<0 ? -pllW : pllW;
			if(u->MoveDir<0) travel = -travel;
			if(travel<=(10922<<16)+aw) pllTh = next; else hallAngleSure = 0;
		}
		if(pllOn){
			//output lead 9/8 periods (the centre of the next pwm period): measured with FOC at 300 and
			//510 rpm, where the d voltage the loops settle at (no d current) is 0 (frame on the rotor)
			int32_t aw = pllW<0 ? -pllW : pllW;
			u->IncAngle = (int16_t)(aw>>16);
			u->Angle = (int16_t)((pllTh+(uint32_t)(pllW/8*9))>>16)+u->MoveDir*CWShift;
		}
	}
	//block commutation: evaluated every time, so a direction change or a mode switch takes effect at
	//once instead of at the next hall edge
	if(com&&!blockbrake&&hall_to_pos[u->RunHallValue]!=step){
		step=hall_to_pos[u->RunHallValue];
		commutate();
	}

	if(millis-lastcommutate>250){//zero out speed
		realspeed=0;
	}
	//speed from the last hall interval. realspeed averages six, which lags by ~3 intervals (100 ms at
	//20 rpm): too slow for a current driven speed loop (FOC), which, unlike the voltage modes, gets no
	//damping from the back emf. The hall sectors are equal within 1.5 deg on the bench motor. Between
	//edges it decays with the time since the last edge once that is longer than the last interval;
	//0 after 250 ms without an edge (~3 rpm), like realspeed.
	{
		TIM_TypeDef* tim = halltimen ? TIM2 : TIM3;
		uint32_t el = (tim->SR&TIM_SR_UI) ? (millis-edgeMs)*1000 : tim->CNT;
		uint32_t iv = u->HallTime[i]>el ? u->HallTime[i] : el;
		int32_t f = iv>=250000 ? 0 : (int32_t)(SPEEDX/6/iv);
		fastspeed = realdir ? f : -f;
	}
	u->PreHallValue = u->RunHallValue;
	u->PreRaw = raw;
}
