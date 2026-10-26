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

static volatile uint16_t abspwm;
volatile uint32_t lastcommutate;
volatile uint8_t gatesoff=0;
volatile uint8_t coastreq=0;
volatile uint8_t blockbrake=0;    //block commutation: low side short brake active, the hall isr does not commutate
static uint16_t brakeduty;        //permille of the period the low sides are on while braking
static int32_t posHold;           //position mode: target kept while the remote is silent
static uint8_t posHeld;

//position mode: hall step error -> speed set point (P, limited)
static int32_t PositionSpeed(int32_t err){
	int32_t vmax = cfgOrDefault(POS_RPM, 100);
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
		gatesoff = lowbatperm || (wState & STATE_DISABLE);
		uint8_t coast = 0, brake = 0;
		//regeneration (braking, slowing down in sine/FOC) into a supply that cannot take it back, e.g. a
		//bench supply: coast while the bus is 3 V above the full battery voltage
		uint8_t overvolt = BAT_FULL>20000&&BAT_FULL<65000&&vbat*10>(int32_t)BAT_FULL+3000;
		if(speed==0){
			FocClearTrip();
		}
		//position: the command is the target odometer (hall steps, like the Gen2.x-GD32 iOdom input);
		//without commands the wheel holds where it is instead of going back to target 0
		int32_t posErr = 0, posSpeed = 0;
		if(CONTROL(DRIVEMODE)==CTRL_POSITION){
			if(RemoteTimedOut()){
				if(!posHeld){ posHold = iOdom; posHeld = 1; }
			}else{
				posHold = speed;
				posHeld = 0;
			}
			posErr = posHold-iOdom;
			posSpeed = PositionSpeed(posErr);
		}
		if(gatesoff){
			pwm=0;
			SpeedCtrlReset();
			FocSpeedReset();
		}else if(COMMUTATION(DRIVEMODE)==COMMUT_FOC){
			//command 0: pwm coasts (gates off), torque 0 current, speed holds 0 rpm (brakes)
			switch(CONTROL(DRIVEMODE)){
				case CTRL_PWM:
					coast = speed==0;
					FocSetVoltage(speed);
					break;
				case CTRL_TORQUE:
					FocSetCurrent((int32_t)speed*10);    //0.01 A
					break;
				case CTRL_POSITION:
					FocSetCurrent(FocSpeedCtrl(posSpeed, fastspeed));
					break;
				default:{
					//below 10 rpm the last hall interval is > 65 ms old: the loop limit-cycled +-55 rpm
					//around a 0 rpm set point on it. Read as 0 there, as before the ms interval timing.
					int32_t m = fastspeed;
					FocSetCurrent(FocSpeedCtrl(speed, (m<10&&m>-10) ? 0 : m));
					break;
				}
			}
			pwm = speed;
		}else{
			uint8_t blk = COMMUTATION(DRIVEMODE)==COMMUT_BLOCK;
			switch(CONTROL(DRIVEMODE)){
				case CTRL_PWM:    //command 0 coasts
					coast = speed==0;
					pwm = blk ? speed*PWM_RES/1000 : speed*40;    //duty counts / Q15 modulation (+-32000 linear)
					break;
				case CTRL_SPEED:    //rpm; command 0 brakes and holds
					if(speed==0){
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
						pwm = SpeedCtrl(speed,realspeed,&p)*(blk ? (int32_t)PWM_RES : 32768)/1000;
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
					if(posErr==0&&fastspeed==0){
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
		if(overvolt){
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
