#include "mcu.h"
#include "main.h"
#include "interrupt.h"
#include "hal_gpio.h"
#include "pinout.h"
#include "remoteUartBus.h"
#include "bldc.h"
#include "commutation.h"
#include "calculation.h"
#include "hal_tim.h"
#include "hal_conf.h"
#include <math.h>
#include "hallhandle.h"
#include "foc.h"
#include "inputmode.h"
#include "cansimple.h"
#include "FOC_Math.h"

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
static int32_t posSp;             //FOC position: speed set point, slewed (rpm)
static int32_t posI;              //position I, rpm << 4

//position I and D (POS_KI / POS_KD, default off), rpm. I on the error in 1/POS_FINE step within 4 steps
//of the target only (outside it resets: on a long move it would wind up and overshoot); D on trajectory
//speed - wheel speed. run 0 (the FOC hold, sine/block at the target): both off, I reset.
static int32_t PosID(int32_t errF, int32_t ff, int32_t meas, int32_t dtms, int32_t vlim, uint8_t run){
	if(!run){ posI = 0; return 0; }
	if(errF>4*POS_FINE||errF<-4*POS_FINE) posI = 0;
	else posI += cfgOrDefault(POS_KI, 0)*errF/POS_FINE*dtms*16/10000;
	if(posI>vlim<<4) posI = vlim<<4; else if(posI<-(vlim<<4)) posI = -(vlim<<4);
	return (posI>>4)+cfgOrDefault(POS_KD, 0)*(ff-meas)/100;
}

//speed for the FOC speed loop: fastspeed (the last hall interval) below 60 rpm, realspeed (6 intervals,
//one electrical turn: the hall sector widths cancel) above 120, blended between. The sectors differ by
//up to 5 deg el and the edges jitter with the current: on fastspeed the loop (every 2 ms) turned that
//into torque steps, broadband 20-250 Hz vibration that grew with speed (480 rpm: 0.78 m/s^2, 0.29 on
//realspeed, a coasting wheel 0.33). realspeed lags ~3 intervals: below 60 rpm the loop limit-cycled on
//it (0 -> 20 rpm: 115 % overshoot). With field weakening fastspeed again, faded in over its first 1 A:
//there the loop swung +-13 rpm on realspeed (1.6 m/s^2 at 570 rpm with 5 A, 1.2 on fastspeed).
static int32_t FocSpeedMeas(void){
	int32_t f = fastspeed, a = f<0 ? -f : f, m;
	if(a<=60) m = f;
	else if(a>=120) m = realspeed;
	else m = (f*(120-a)+realspeed*(a-60))/60;
	int32_t fw = -FocFieldWeakening();
	if(fw>=1000) return f;
	return (m*(1000-fw)+f*fw)/1000;
}

//position gain, 0.1 rpm per hall step: POS_KP, default FOC 133 (ODrive's pos_gain 20 (turns/s)/turn),
//block / sine 20 (they act on whole hall steps; more hunts around the target)
int32_t PosGain(void){
	return cfgOrDefault(POS_KP, COMMUTATION(DRIVEMODE)==COMMUT_FOC ? 133 : 20);
}

//block / sine position mode: hall step error -> speed set point (P, limited)
static int32_t PositionSpeed(int32_t err){
	int32_t vmax = cfgOrDefault(POS_RPM, 300);
	if(err<=1&&err>=-1) return 0;    //at the target within the hall step: hold, the speed integrator decays
	int32_t v = err*PosGain()/10;
	if(v>vmax) v = vmax; else if(v<-vmax) v = -vmax;
	return v;
}
uint8_t lowbatcount=0;
uint8_t lowbatperm=0;

const uint8_t hall_to_pos[8] = HALL_TO_POS_INIT;


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

//TIM1 channels 1-3: output compare mode (all three) and the CCER enable bits CCxE / CCxNE (the polarity
//bits stay), then the COM event: with CCPC set (TIM1_init) these registers are preloaded and switch
//together at that event
#define CCER_EN (TIM_CCER_CC1EN|TIM_CCER_CC1NEN|TIM_CCER_CC2EN|TIM_CCER_CC2NEN|TIM_CCER_CC3EN|TIM_CCER_CC3NEN)
static void tim1Outputs(uint32_t mode, uint32_t en){
	TIM1->CCMR1 = (TIM1->CCMR1&~(TIM_CCMR1_OC1M|TIM_CCMR1_OC2M))|mode|mode<<8;
	TIM1->CCMR2 = (TIM1->CCMR2&~TIM_CCMR2_OC3M)|mode;
	TIM1->CCER = (TIM1->CCER&~CCER_EN)|en;
	TIM1->EGR = TIM_EventSource_COM;
}

//block commutation (commutation.h)
void commutate(void){//U,V,W(A,B,C)
	static const uint16_t en[7] = BLOCK_CCER_INIT;
	tim1Outputs(TIM_OCMode_PWM1, step<7 ? en[step] : 0);    //other steps: 0,0,0
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
		//a disarm by error leaves the axis IDLE (ODrive): Clear_Errors alone switched the gates back on
		//with the old set point; now the master arms again (Set_Axis_State CLOSED_LOOP_CONTROL)
		if(axisDisarm) axisState = AXIS_IDLE;
		gatesoff = lowbatperm || axisDisarm || idle || (charging&&cfgBool(CHARGE_LOCK, 0));
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
		uint8_t posRest = 0;    //FOC position without the hold: at the target, standing
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
		if(held||ctl==CTRL_PWM||ctl==CTRL_TORQUE) InputTrack(hallPosFine, realspeed);
		//velocity set point; the command limited first, the ramp would wind up to it
		int32_t vin = speed>vlim ? vlim : (speed<-vlim ? -vlim : speed);
		int32_t spd = ctl==CTRL_SPEED ? InputVel(vin, dtms, regen, realspeed) : speed;
		if(held||ctl!=CTRL_PWM){
			vRamp = 0;
		}else{
			int32_t st = VOLT_SLEW*dtms, d = speed-vRamp;
			if(regen&&vRamp&&(d>0)!=(vRamp>0)) st = 0;    //towards 0: pause while the bus is high
			vRamp += d>st ? st : (d<-st ? -st : d);
		}
		if(spd>vlim) spd = vlim; else if(spd<-vlim) spd = -vlim;
		int32_t tq = ctl==CTRL_TORQUE ? InputTorque(speed, dtms, held) : speed;    //torque set point, 0.01 A
		//enable_overspeed_error: disarm above vel_limit * vel_limit_tolerance (ODrive)
		if(cfgBool(OVERSPEED_ERROR, 0)&&!gatesoff){
			int32_t lim = vlim*cfgOrDefault(VEL_LIMIT_TOL, 120)/100;
			if(realspeed>lim||realspeed<-lim) axisDisarm |= ERR_VELOCITY_LIMIT_VIOLATION;
		}
		uint8_t focPos = ctl==CTRL_POSITION&&COMMUTATION(DRIVEMODE)==COMMUT_FOC&&!held;
		if(!focPos){ FocHoldStop(); posSp = realspeed; }
		if(held||ctl!=CTRL_POSITION) posI = 0;
		if(ctl==CTRL_POSITION){
			//targets and the estimate in 1/POS_FINE step
			if(timedOut){
				if(!posHeld){ posHold = hallPosFine; posHeld = 1; }
			}else{
				posHold = cmdSource==2 ? posCmd : speed*POS_FINE;
				posHeld = 0;
			}
			int32_t ff = 0, target = posHold;
			if(!held) target = InputPos(posHold, cmdSource==2&&!timedOut ? posVelFf : 0, dtms, &ff);
			int32_t errF = target-hallPosFine;
			if(focPos){
				//FOC: P on the fine position (the tracker's angle in the sector while moving) into the speed
				//loop, no dead band; within 1.5 steps and below 25 rpm the hold (foc.c) takes the rotor to the
				//target's electrical angle. (It was P with a +-1 step dead band at 2 rpm per step: a 1 step
				//command did nothing, it stopped up to 4 deg off, ~1.2 s for half a turn.)
				int32_t af = fastspeed<0 ? -fastspeed : fastspeed;
				int32_t p = errF/POS_FINE*PosGain()/10+(errF%POS_FINE)*PosGain()/10/POS_FINE;
				uint8_t holdOn = (uint8_t)cfgBool(HOLD_ENABLE, 1);
				int32_t hw = cfgOrDefault(HOLD_WINDOW, 150)*POS_FINE/100, ac = cfgOrDefault(POS_ACCEL, 2000);    //0.01 turns/s^2
				if(focHold){
					int32_t dh = target-focHoldPos;
					if(focHoldSlip||dh>3*POS_FINE||dh<-3*POS_FINE){ FocHoldStop(); FocSpeedReset(); }
					else FocHoldTarget(target);
				}else if(holdOn&&errF<=hw&&errF>=-hw&&af<=25&&ff<=25&&ff>=-25){
					FocHoldStart(target);
				}
				p += PosID(errF, ff, fastspeed, dtms, vlim, !focHold);
				int32_t dh = target-focHoldPos;
				//arrived: in the hold at the target; without the hold within half a step, standing (it was
				//the hold only: with it off trajectory_done never came)
				posDone = InputPosDone()&&(focHold ? dh<=2&&dh>=-2 : !holdOn&&errF<=POS_FINE/2&&errF>=-POS_FINE/2&&af<=2);
				//and no faster than it can stop from at 20 turns/s^2 (1800 steps/s^2): sqrt(2 a err) steps/s =
				//sqrt(14 errF) * 2/3 rpm. With P alone half a turn overshot by 7 steps (still ~110 rpm 3 steps
				//before the target).
				int32_t ae = errF<0 ? -errF : errF; if(ae>1000000) ae = 1000000;
				int32_t pl = (int32_t)MCM_Sqrt(ae/16*(ac*9/80))*2/3;    //accel_limit (default 20 turns/s^2)
				if(p>pl) p = pl; else if(p<-pl) p = -pl;
				if(!holdOn&&ae<=POS_FINE/2) p = 0;    //without the hold: no hunting inside the step
				//and no current there once standing: the speed loop (3.3x below 60 rpm) acted on the last hall
				//interval's stale +-3 rpm and the estimate's whole step jumps at a hall edge: +-1.5 A at ~16 Hz
				//for up to 0.8 s after a long move. It restarts from 0 when the error leaves the half step.
				posRest = !holdOn&&ae<=POS_FINE/2&&af<=5;
				//and it accelerates at that rate too (1200 rpm/s): from a standing start the full current
				//limit (10 A, ~3700 rpm/s here) outran the hall angle tracker, it snapped, a 32 A trip
				int32_t sl = ac*3*dtms/5000+1;
				if(focHold) posSp = fastspeed;
				posSp += p+ff-posSp>sl ? sl : (p+ff-posSp<-sl ? -sl : p+ff-posSp);
				posSpeed = posSp;
			}else{
				posErr = errF>=0 ? (errF+POS_FINE/2)/POS_FINE : -((-errF+POS_FINE/2)/POS_FINE);    //whole steps
				posDone = InputPosDone()&&posErr<=1&&posErr>=-1;
				int32_t v = PositionSpeed(posErr);    //0 at the target (+-1 step): I and D off there
				posSpeed = ff+v+PosID(errF, ff, realspeed, dtms, vlim, v!=0);
			}
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
						i = tq*10;    //0.01 A
						//enable_torque_mode_vel_limit (ODrive): the torque falls to 0 at vel_limit (vel_gain slope)
						if(cfgBool(TORQUE_VEL_LIMIT, 1)){
							int32_t kv = cfgOrDefault(FOC_SPD_KP, 30), hi = kv*(vlim-realspeed), lo = kv*(-vlim-realspeed);
							if(i>hi) i = hi; else if(i<lo) i = lo;
						}
					}else if(ctl==CTRL_POSITION){
						//fastspeed: the 6 interval average lags in the reversals and hard stops of position
						//moves (143 rpm read against 21 in a reversal: the loop drove 10 A the wrong way, trip)
						if(posRest) FocSpeedReset();
						i = focHold||posRest ? 0 : FocSpeedCtrl(posSpeed, fastspeed, 0);
						//inertia (ODrive): torque feed forward of the trajectory's acceleration
						if(!focHold&&INERTIA&&INERTIA!=0xFFFF) i += (int32_t)((float)INERTIA*0.1f*InputAccel()/TorqueConstant());
					}else{
						//below 10 rpm the last hall interval is > 65 ms old: the loop limit-cycled +-55
						//rpm around a 0 rpm set point on it. Read as 0 there, as before the ms timing.
						int32_t m = FocSpeedMeas();
						i = FocSpeedCtrl(spd, (m<10&&m>-10) ? 0 : m, 1);
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
					pwm = blk ? vRamp*(int32_t)PWM_PERIOD/1000 : vRamp*40;    //duty counts / Q15 modulation (+-32000 linear)
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
						//permille of full scale: duty (PWM_PERIOD) for block, modulation (32768) for sine
						pwm = SpeedCtrl(spd,realspeed,&p)*(blk ? (int32_t)PWM_PERIOD : 32768)/1000;
					}
					break;
				case CTRL_TORQUE:    //0.01 A; command 0 coasts
					if(tq==0){
						coast = 1;
						pwm = 0;
						TorqueCtrlReset();
					}else if(blk){
						//direction from the sign; the shunt current is a magnitude
						int32_t sgn = tq>0 ? 1 : -1;
						pwm = sgn*TorqueCtrlBlock(tq*10*sgn, fitotal*10, fastspeed*sgn)*(int32_t)PWM_PERIOD/1000;
					}else{
						//sine, no current sense: V = I*R + back emf (the sine kv), as modulation, plus the
						//dead band of the sine plant (dead time, diodes: 1 % ~ 140 mV) in the command's direction
						int32_t v = vbat>1000 ? vbat : 2400;
						SpeedPlant p;
						SpeedParams(SINE_SPEED, &p);
						int32_t mv = tq*10*cfgOrDefault(MOTOR_R, 300)/1000+fastspeed*5774/p.kv;
						pwm = mv*5676/v+(tq>0 ? 1 : -1)*p.d0*32768/1000;
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
						pwm = SpeedCtrl(posSpeed,realspeed,&p)*(blk ? (int32_t)PWM_PERIOD : 32768)/1000;
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
			uint16_t ccr = blockbrake ? (uint16_t)((uint32_t)PWM_PERIOD*brakeduty/1000) : abspwm;
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
		TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_PERIOD;
		tim1Outputs(TIM_OCMode_PWM1, TIM_CCER_CC1NEN|TIM_CCER_CC2NEN|TIM_CCER_CC3NEN);    //low sides only
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
	if(COMMUTATION(DRIVEMODE)==COMMUT_BLOCK) commutate();    //the outputs of the current step
	else tim1Outputs(TIM_OCMode_PWM2, CCER_EN);    //sine / FOC: all six switch complementary
}
