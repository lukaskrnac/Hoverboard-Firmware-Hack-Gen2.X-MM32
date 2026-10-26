#include "mcu.h"
#include "main.h"
#include "interrupt.h"
#include "hal_gpio.h"
#include "pinout.h"
#include "remoteUartBus.h"
#include "bldc.h"
#include "calculation.h"
#include "hal_tim.h"
#include "hal_conf.h"
#include <math.h>
#include "hallhandle.h"
#include "foc.h"
#include "inputmode.h"

static volatile uint16_t abspwm;
volatile uint32_t lastcommutate;
volatile uint8_t gatesoff=0;
volatile uint8_t coastreq=0;
volatile uint8_t blockbrake=0;    //block commutation: low side short brake active, the hall isr does not commutate
static uint16_t brakeduty;        //permille of the period the low sides are on while braking
static int32_t posHold;           //position mode: target kept while the remote is silent
volatile int32_t posCmd;          //position target from the CANSimple protocol, hall steps (int32)
volatile int32_t posVelFf;        //its velocity feed forward, rpm
static uint32_t inT;              //last speedupdate(), for the input modes
static int32_t vRamp;             //pwm control: the applied command, permille, slewed (VOLT_SLEW)
#define VOLT_SLEW 1               //permille per ms: full scale in 1 s
volatile uint8_t cmdSource = 1;   //protocol of the last command frame: 1 v1 ('/'), 2 CANSimple
volatile uint8_t axisState = AXIS_CLOSED_LOOP;    //ODrive AxisState: IDLE 1 = gates off, CLOSED_LOOP 8
volatile uint32_t axisErrors;     //ODrive Error bits, the conditions active now (see bldc.h)
volatile uint32_t axisDisarm;     //the error that switched the axis to IDLE, until Clear_Errors
volatile uint8_t posDone;         //position mode: at the target
static uint8_t posHeld;

//position mode: hall step error -> speed set point (P, limited)
static int32_t PositionSpeed(int32_t err){
	int32_t vmax = cfgOrDefault(POS_RPM, 300);
	if(err<=1&&err>=-1) return 0;    //at the target within the hall step: hold, the speed integrator decays
	int32_t v = err*cfgOrDefault(POS_KP, 20)/10;
	if(v>vmax) v = vmax; else if(v<-vmax) v = -vmax;
	return v;
}
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


uint8_t hallpos(uint8_t forward){
	uint8_t HallA;
	uint8_t HallB;
	uint8_t HallC;
	uint8_t HallValue;

	HallA = digitalRead(HALLAPIN);
	HallB = digitalRead(HALLBPIN);
	HallC = digitalRead(HALLCPIN);

	if(forward == 1)
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




void speedupdate(void){
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
		//armed: v1 frames arm and disarm with state bit 64, CANSimple with the axis state (a v1 master
		//cannot set that). A disarm by error (Estop) holds for both until Clear_Errors.
		uint8_t idle = cmdSource==1 ? (wState & STATE_DISABLE)!=0 : axisState==AXIS_IDLE;
		//a phase current trip disarms a CANSimple axis until Clear_Errors (ODrive). Clearing it by itself
		//at command 0 re-armed while the set point was still high (VEL_RAMP): the wheel shot off. v1
		//masters have no Clear_Errors: there the trip still clears at command 0.
		if(focTrip&&cmdSource==2) axisDisarm |= ERR_CURRENT_LIMIT_VIOLATION;
		gatesoff = lowbatperm || axisDisarm || idle;
		uint8_t coast = 0, brake = 0;
		//regeneration (braking, slowing down in sine/FOC) into a supply that cannot take it back, e.g. a
		//bench supply: coast while the bus is 3 V above the full battery voltage
		uint8_t overvolt = BAT_FULL>20000&&BAT_FULL<65000&&vbat*10>(int32_t)BAT_FULL+3000;
		axisErrors = (lowbatperm ? ERR_DC_BUS_UNDER_VOLTAGE : 0)|(overvolt ? ERR_DC_BUS_OVER_VOLTAGE : 0)|
		             (overcurrent ? ERR_DC_BUS_OVER_CURRENT : 0)|(focTrip ? ERR_CURRENT_LIMIT_VIOLATION : 0);
		if((speed==0&&CONTROL(DRIVEMODE)!=CTRL_POSITION)||gatesoff){
			FocClearTrip();    //zero command, or disarmed (the only way in position mode: its command is a target)
		}
		//position: the command is the target odometer (hall steps, like the Gen2.x-GD32 iOdom input);
		//without commands the wheel holds where it is instead of going back to target 0
		//set points through the input mode (inputmode.c), limited by vel_limit. Disarmed and in pwm/torque
		//control they follow the wheel (bumpless start). A velocity command timeout sets speed 0, which
		//VEL_RAMP ramps down too (a step to 0 brakes hard: regeneration a bench supply cannot take).
		//pwm control: the command slews at VOLT_SLEW and coasts once the ramp is at 0 (it switched the
		//gates off at once: above base speed with field weakening the back emf then charged the bus
		//through the body diodes). Estop / IDLE still switch the gates off at once.
		int32_t posErr = 0, posSpeed = 0;
		int32_t vlim = cfgOrDefault(POS_RPM, 300);    //vel_limit, rpm
		int32_t dtms = (int32_t)(millis-inT);
		inT = millis;
		if(dtms>20) dtms = 20;
		uint8_t ctl = CONTROL(DRIVEMODE), timedOut = RemoteTimedOut();
		uint8_t held = gatesoff||focTrip;    //gates off: set points and controllers follow the wheel
		//regeneration limit: above the full battery voltage (BAT_FULL) the bus cannot take energy back (a
		//full battery, a bench supply): ramps that slow the wheel down pause and FOC gives no braking
		//current; the wheel slows by friction. Above BAT_FULL + 3 V it coasts (overvolt, below). A 1 s
		//voltage ramp from 1200 rpm (sine field weakening) pumped a bench supply to 46.6 V.
		uint8_t regen = BAT_FULL>20000&&BAT_FULL<65000&&fvbat*10>(int32_t)BAT_FULL;
		if(held||ctl==CTRL_PWM||ctl==CTRL_TORQUE) InputTrack(iOdom, realspeed);
		int32_t spd = ctl==CTRL_SPEED ? InputVel(speed, dtms, regen, realspeed) : speed;    //velocity set point
		if(held||ctl!=CTRL_PWM){
			vRamp = 0;
		}else{
			int32_t st = VOLT_SLEW*dtms, d = speed-vRamp;
			if(regen&&vRamp&&(d>0)!=(vRamp>0)) st = 0;    //towards 0: pause while the bus is high
			vRamp += d>st ? st : (d<-st ? -st : d);
		}
		if(spd>vlim) spd = vlim; else if(spd<-vlim) spd = -vlim;
		if(ctl==CTRL_POSITION){
			if(timedOut){
				if(!posHeld){ posHold = iOdom; posHeld = 1; }
			}else{
				posHold = cmdSource==2 ? posCmd : speed;
				posHeld = 0;
			}
			int32_t ff = 0, target = posHold;
			if(!held) target = InputPos(posHold, cmdSource==2&&!timedOut ? posVelFf : 0, dtms, &ff);
			posErr = target-iOdom;
			posDone = InputPosDone()&&posErr<=1&&posErr>=-1;
			posSpeed = ff+PositionSpeed(posErr);
			if(posSpeed>vlim) posSpeed = vlim; else if(posSpeed<-vlim) posSpeed = -vlim;
		}
		if(held){
			pwm=0;
			SpeedCtrlReset();
			FocReset();
		}else if(COMMUTATION(DRIVEMODE)==COMMUT_FOC){
			//command 0: pwm coasts (gates off), torque 0 current, speed holds 0 rpm (brakes)
			switch(CONTROL(DRIVEMODE)){
				case CTRL_PWM:
					coast = vRamp==0;
					FocSetVoltage(vRamp);
					break;
				default:{
					int32_t i;    //mA
					if(ctl==CTRL_TORQUE){
						i = (int32_t)speed*10;    //0.01 A
					}else if(ctl==CTRL_POSITION){
						i = FocSpeedCtrl(posSpeed, fastspeed);
					}else{
						//below 10 rpm the last hall interval is > 65 ms old: the loop limit-cycled +-55
						//rpm around a 0 rpm set point on it. Read as 0 there, as before the ms timing.
						int32_t m = fastspeed;
						i = FocSpeedCtrl(spd, (m<10&&m>-10) ? 0 : m);
					}
					//regeneration limit (see above): no braking current while the bus is above BAT_FULL
					if(regen&&((i>0&&realspeed<0)||(i<0&&realspeed>0))) i = 0;
					FocSetCurrent(i);
					break;
				}
			}
			pwm = speed;
		}else{
			uint8_t blk = COMMUTATION(DRIVEMODE)==COMMUT_BLOCK;
			switch(CONTROL(DRIVEMODE)){
				case CTRL_PWM:    //command 0 coasts (after the ramp)
					coast = vRamp==0;
					pwm = blk ? vRamp*PWM_RES/1000 : vRamp*40;    //duty counts / Q15 modulation (+-32000 linear)
					break;
				case CTRL_SPEED:    //rpm; command 0 brakes and holds
					if(spd==0){
						//block: short the motor with the low sides (it cannot brake through its normal
						//switching). Sine: the modulation ramps to 0 at the speed loop's slew rate (3 per
						//mille/ms; the motor regenerates as it falls below the back emf), then the zero
						//vector shorts the motor and holds. Running the speed loop at 0 rpm hunted +-20 rpm.
						if(blk){
							pwm = 0;
							brake = 1;
						}else{
							int32_t st = 32768*6/1000;    //this runs every ~2 ms
							pwm = pwm>st ? pwm-st : (pwm<-st ? pwm+st : 0);
						}
						SpeedCtrlReset();
					}else{
						SpeedPlant p;
						SpeedParams(DRIVEMODE, &p);
						//permille of full scale: duty (PWM_RES) for block, modulation (32768) for sine
						pwm = SpeedCtrl(spd,realspeed,&p)*(blk ? (int32_t)PWM_RES : 32768)/1000;
					}
					break;
				case CTRL_TORQUE:    //0.01 A; command 0 coasts
					if(speed==0){
						coast = 1;
						pwm = 0;
						TorqueCtrlReset();
					}else if(blk){
						//direction from the sign; the shunt current is a magnitude
						int32_t sgn = speed>0 ? 1 : -1;
						pwm = sgn*TorqueCtrlBlock((int32_t)speed*10*sgn, fitotal*10, fastspeed*sgn)*(int32_t)PWM_RES/1000;
					}else{
						//sine, no current sense: V = I*R + back emf (the sine kv), as modulation, plus the
						//dead band of the sine plant (dead time, diodes: 1 % ~ 140 mV) in the command's direction
						int32_t v = vbat>1000 ? vbat : 2400;
						SpeedPlant p;
						SpeedParams(SINE_SPEED, &p);
						int32_t mv = (int32_t)speed*10*cfgOrDefault(MOTOR_R, 300)/1000+fastspeed*5774/p.kv;
						pwm = mv*5676/v+(speed>0 ? 1 : -1)*p.d0*32768/1000;
						if(pwm>32000) pwm = 32000; else if(pwm<-32000) pwm = -32000;
					}
					break;
				case CTRL_POSITION:    //holds at the target like the speed modes at 0 (short brake / zero vector)
					if(posErr==0&&posSpeed==0&&fastspeed==0){
						pwm = 0;
						brake = blk;
						SpeedCtrlReset();
					}else{
						SpeedPlant p;
						SpeedParams(blk ? COM_SPEED : SINE_SPEED, &p);
						pwm = SpeedCtrl(posSpeed,realspeed,&p)*(blk ? (int32_t)PWM_RES : 32768)/1000;
					}
					break;
				default:
					pwm = 0;
					coast = 1;
					break;
			}
		}
		//overvoltage: coast, but only below ~90 % of the speed the bus voltage allows (sine kv). Above
		//it the back emf is higher than the bus: with the gates off it charges the bus through the
		//diodes (a bench supply went to 36 V and shut down); the regeneration limit acts there instead.
		int32_t kvs = SINE_KV ? SINE_KV : 242, vbase = kvs*fvbat/1000;    //rpm at 100 % modulation
		if(overvolt&&(realspeed<0 ? -realspeed : realspeed)<vbase*9/10){
			coast = 1;
			brake = 0;
		}
		BlockBrake(brake);
		coastreq = coast;
		if(pwm>0){
		dir=1;
		}else{
		dir=-1;
		}
		abspwm=pwm<0 ? -pwm : pwm;    // 4095~-4095
		if(COMMUTATION(DRIVEMODE)==COMMUT_BLOCK){
			uint16_t ccr = blockbrake ? (uint16_t)((uint32_t)PWM_RES*brakeduty/1000) : abspwm;
			TIM1->CCR1=ccr;
			TIM1->CCR2=ccr;
			TIM1->CCR3=ccr;
		}
	}
}

//Block commutation short brake: high sides off, the three low sides switched together. With CCxE off
//the complementary output follows OCxREF itself (on while CNT < CCR, as in commutate()), so the motor's phases are shorted for brakeduty of the
//period. The duty ramps up over ~0.3 s (the brake torque is proportional to speed: a sudden full short
//at speed is a current spike) and stays at 100 % while standing. The hall isr does not commutate
//while blockbrake is set; ending the brake forces a commutation.
void BlockBrake(uint8_t on){
	if(!on||COMMUTATION(DRIVEMODE)!=COMMUT_BLOCK){
		if(blockbrake){
			NVIC_DisableIRQ(ADC_COMP_IRQn);
			blockbrake = 0;
			brakeduty = 0;
			step = 0;    //the hall isr commutates at its next run
			if(COMMUTATION(DRIVEMODE)==COMMUT_BLOCK) commutate();
			NVIC_EnableIRQ(ADC_COMP_IRQn);
		}
		return;
	}
	if(!blockbrake){
		NVIC_DisableIRQ(ADC_COMP_IRQn);
		blockbrake = 1;
		brakeduty = 0;
		TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_RES;
		for(uint8_t ch=0;ch<3;ch++){
			uint16_t c = ch==0 ? TIM_Channel_1 : (ch==1 ? TIM_Channel_2 : TIM_Channel_3);
			TIM_SelectOCxM(TIM1, c, TIM_OCMode_PWM1);
			TIM_CCxCmd(TIM1, c, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, c, TIM_CCxN_Enable);
		}
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
		NVIC_EnableIRQ(ADC_COMP_IRQn);
	}
	if(brakeduty<1000){
		brakeduty += 6;    //called every ~2 ms
		if(brakeduty>1000) brakeduty = 1000;
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

//Change the drive mode while the motor may be running. The adc isr reads DRIVEMODE several times per
//run and in block mode commutates (writes CCER) itself, and pwm has a different unit per mode (duty
//counts / Q15 modulation: a sine value left in CCR1-3 is full duty in block mode). So the gates go
//off and the isr is masked (a few us) while the mode, the outputs and the controllers are replaced;
//the isr arms the gates again and the speed controllers restart from the measured speed.
static void TIMOCInit(void);

uint8_t DriveModeSupported(uint8_t mode){
	if(COMMUTATION(mode)==COMMUT_FOC){
		return mode<=FOC_POSITION&&FocAvailable();
	}
	return mode<=SINE_POSITION;
}

void SetDriveMode(uint8_t mode){
	if(mode==DRIVEMODE||!DriveModeSupported(mode)){
		return;
	}
	NVIC_DisableIRQ(ADC_COMP_IRQn);
	gatesOutput(0);
	pwm = 0;
	abspwm = 0;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	DRIVEMODE = mode;
	blockbrake = 0;    //TIMOCInit() sets the outputs up from scratch
	SpeedCtrlReset();
	FocInit();
	TIMOCInit();
	NVIC_EnableIRQ(ADC_COMP_IRQn);
}

static void TIMOCInit(void){
	if(COMMUTATION(DRIVEMODE)==COMMUT_BLOCK){
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
