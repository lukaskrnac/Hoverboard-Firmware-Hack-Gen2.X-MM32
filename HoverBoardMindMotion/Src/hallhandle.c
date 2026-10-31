/*-------------------- Includes -----------------------*/
#include "hallhandle.h"
#include "main.h"
#include "pinout.h"
#include "interrupt.h"
#include "bldc.h"
#include "foc.h"
#include "mcu.h"

/*------------------- Private variables ---------------*/
HALLType HALL1;
volatile int32_t fastspeed;
volatile uint8_t hallAngleSure;    //the angle is known: a regular edge, its interpolation not run out
volatile uint16_t hallInvalid;    //invalid hall readings (diagnostic)
volatile int16_t hallAngleSample;  //rotor angle at this period's current sample (Angle has the output lead)
volatile int32_t hallPosFine;      //position, 1/POS_FINE hall step (odometer and the angle in the sector)
static uint16_t bndLo[8], bndHi[8];  //per hall state its edges (lower: entered with rising angle, upper)
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

	//sector centres of states 1-6 (CWAngleTab[1..6])
	static const uint16_t centre[6] = { 43688, 0, 54610, 21844, 32767, 10922 };
	//calibrated edges (PinFinder calibrate hall, all six set): the state from its edge to the next state's
	//edge, the centre in the middle. Unset: +-30 deg around the table above (the bench motor, within 1.5 deg)
	uint8_t cal = 1;
	for(i=1;i<=6;i++) if(HALL_ANG(i)==0||HALL_ANG(i)==0xFFFF) cal = 0;
	for(i=1;i<=6;i++){
		uint16_t c = centre[i-1], lo = (uint16_t)(c-5461), width = 10922;
		if(cal){
			lo = HALL_ANG(i);
			width = 0xFFFF;
			for(uint8_t j=1;j<=6;j++){    //the next edge above (circular)
				uint16_t d = (uint16_t)(HALL_ANG(j)-lo);
				if(j!=i&&d&&d<width) width = d;
			}
			c = (uint16_t)(lo+width/2);
		}
		u->CWAngleTab[i] = c;
		bndLo[i] = lo;
		bndHi[i] = (uint16_t)(lo+width);
	}



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
		//the timer captures the interval and restarts at every edge, ~14 us after the pin changes (its
		//input filter). An edge seen here before that still has the previous interval in CCR1 and the
		//count running since the previous edge: it was taken as an edge two periods old (the angle
		//tracker ~2 periods of travel off, at 300 rpm on ~10 % of the edges). A count above 1.5 periods
		//is that case: the interval is the count, the edge is now.
		uint32_t per = (CTRL_ARR+18)/36;    //control period, us
		uint32_t cnt = tim->CNT;
		if(cnt>per*3/2){
			u->HallTime[i] = cnt;
			since = 0;
		}else{
			u->HallTime[i] = tim->CCR1;    //us since the previous edge
			since = cnt+HALL_FILTER_US;    //us since this edge
		}
		//an interval below 100 us cannot be real (1000 rpm: 670 us): a spike made two captures (the timer
		//input filter should stop them); the previous interval stands in
		if(u->HallTime[i]<100) u->HallTime[i] = u->HallTime[(i+5)%6];
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
	//at +-30 deg (5461), within 1.5 deg; other motors: the calibrated edges (bndLo / bndHi, HALL_ANG). At an edge the rotor is on the boundary just crossed; between
	//edges the angle is interpolated at the measured speed, at most one sector. The old code put the
	//angle on the sector centre at the edge (30 deg ahead) and interpolated as if this ran every 62.5 us
	//instead of every pwm period (2*ARR/72 MHz = 227.6 us), so the angle lagged and jumped.
	if(!com){
		//The tracker starts from the last interval alone, at the second edge of a start or after a reversal. (On
		//the average of six, > 65 ms each, it took ~6 edges: FOC position moves started and reversed on sector
		//centres held 10-30 ms and stepped 60 deg el at the edges, the rotor lurched from stall to ~100 rpm, and
		//the speed loop answered with +-5 A per edge: 2-2.4 m/s^2 rms, peaks 14, 35-70 Hz.)
		if(edge&&u->HallTime[i]>=65535){
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
			uint32_t per = (CTRL_ARR+18)/36;    //control period, us
			uint32_t k = CTRL_ARR*16384/54;    //65536*2*ARR/72/6 / interval: per period, 65536 per turn
			uint32_t q = k/ivl, r = k%ivl;    //(k<<16)/ivl in 32 bits: 2^32 per turn
			int32_t wMeas = s*(int32_t)((q<<16)+(((r<<8)/ivl)<<8));
			uint32_t bnd = (uint32_t)(s>0 ? bndLo[raw] : bndHi[raw])<<16;    //the edge just crossed
			uint32_t meas = bnd+(uint32_t)(wMeas/(int32_t)per*(int32_t)since);
			//compared with this period's prediction. It was compared with the last period's angle (the
			//advance below runs only between edges): every error one period of travel too large (6 deg
			//at 300 rpm, 15 at 750), the speed 1/(intervals - 1) too high, the angle a sawtooth of a
			//period's travel per sector, and with the capture race above ~10 % of the edges past the
			//15 deg snap. Measured at 300 rpm: mean error -5.6 deg = one period, sd 6.6 deg.
			uint32_t pred = pllTh+(uint32_t)pllW;
			int32_t e = (int32_t)(meas-pred);
			if(!pllOn||(pllW>0)!=(s>0)||e>(2731<<16)||e<-(2731<<16)){    //start, reversal, > 15 deg off
				//(a half correction at speed instead left the error in fast accelerations: braking
				//current that grew with speed, a 15 A trip at 13 A field weakening)
				pllTh = meas; pllW = wMeas; pllOn = 1;
			}else{
				int32_t n = (int32_t)(ivl/per);
				//gain per edge 1/8 (speed 1/64); 1/16 (1/256) with 6 A of d current or more: the edges move
				//with the stator current (armature reaction: intervals +-9 % at 820 rpm with 13 A of d
				//current, far more than the rotor can change), the tracker passed that into the frame, and
				//with a large d current a frame error is torque. 749 rpm, 9 A of field weakening: 0.46-0.53
				//instead of 0.70 m/s^2; with 5 A 1/8 shook less (1.2 against 1.4).
				int32_t gk = (focIdNow<=-6000||focIdNow>=6000) ? 4 : 3;
				pllTh = pred+(uint32_t)(e/(1<<gk));
				pllW += e/(1<<(2*gk))/(n<1 ? 1 : n);
			}
			pllBnd = bnd;
			hallAngleSure = 1;
		}else if(millis-lastcommutate>250||(millis-edgeMs)*1000>2*u->HallTime[i]+5000){
			//standing: middle of the sector; also after two intervals without an edge (an abrupt stop:
			//the extrapolated angle stood up to a sector off, the q current made little torque)
			u->Angle = (int16_t)u->CWAngleTab[raw];
			u->IncAngleMax = 0;
			hallAngleSure = 0;
			pllOn = 0;
		}else if(pllOn){
			//at most one sector (and a period) past the last edge: no edge where one was due means the
			//rotor is slower; the angle waits there and the d current is off until the next edge (a
			//negative d current in a frame ahead of the rotor brakes it, it stalled and rocked). 1.25
			//sectors: the sectors differ by up to 5 deg el, the edges move ~10 % with the current. (Two
			//sectors at speed, against late edges that were the capture race and the prediction: after an
			//abrupt stop the frame stood up to 120 deg off.)
			uint32_t next = pllTh+(uint32_t)pllW;
			int32_t travel = (int32_t)(next-pllBnd), aw = pllW<0 ? -pllW : pllW;
			if(u->MoveDir<0) travel = -travel;
			if(travel<=(13653<<16)+aw) pllTh = next; else hallAngleSure = 0;
		}
		if(pllOn){
			//output lead 10/8 periods: measured with FOC at 300 and 480 rpm, where the d voltage the loops
			//settle at (no d current) is 0 (frame on the rotor). (It was 9/8 with the tracker that ran
			//a period ahead at the edges.) With the control every n-th pwm period (pwmDiv): the outputs
			//take effect at the next update, n pwm periods later, and hold n periods, centred 1.5 n
			//later; less the measured 1/4 pwm period: (24 - 4/n)/16 control periods (n = 1: 10/8)
			int32_t aw = pllW<0 ? -pllW : pllW;
			u->IncAngle = (int16_t)(aw>>16);
			u->Angle = (int16_t)((pllTh+(uint32_t)(pllW/16*(24-4/pwmDiv)))>>16)+u->MoveDir*CWShift;
			//the tracker angle is the one when the hall timer is read here, ~25 us (1/8 period) after the
			//counter zero, the current sample's time
			hallAngleSample = (int16_t)((pllTh-(uint32_t)(pllW/8))>>16);
		}else{
			hallAngleSample = u->Angle;
		}
	}
	//block commutation: evaluated every time, so a direction change or a mode switch takes effect at
	//once instead of at the next hall edge
	if(com&&!blockbrake&&!regenShort&&hall_to_pos[u->RunHallValue]!=step){
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
	//the previous state is the last valid one: an invalid reading (0 or 7: two sensors switching at once,
	//noise at high current) for one poll made the next edge compare against it and not count. The
	//edge was lost (the odometer step too), the angle tracker waited a sector for it and then jumped
	//85 deg: a 21 A trip with field weakening at 600 rpm.
	if(hall_to_pos[raw]) u->PreRaw = raw; else hallInvalid++;
	//fine position: the odometer (the sector centre) plus the tracker's angle in the sector while it runs
	//(the odometer counts up as the electrical angle falls); in the FOC position hold the angle the rotor
	//is held at (foc.c); standing otherwise the sector centre
	int32_t pf = iOdom*POS_FINE;
	if(focHold) pf = focHoldPos;
	else if(pllOn&&hallAngleSure&&!com) pf -= (int32_t)(int16_t)(hallAngleSample-(int16_t)u->CWAngleTab[u->PreRaw])*POS_FINE/10923;
	hallPosFine = pf;
}

