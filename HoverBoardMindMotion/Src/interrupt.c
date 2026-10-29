#include "hal_tim.h"
#include "hal_conf.h"
#include "hal_adc.h"
#include <stdio.h>
#include "pinout.h"
#include "interrupt.h"
#include "main.h"
#include "hardware.h"
#include "bldc.h"
#include "remoteUartBus.h"
#include "calculation.h"

#include "ipark.h"
#include "pwm_gen.h"
#include "hallhandle.h"
#include "foc.h"

static uint32_t itotaloffset=0;
volatile uint8_t poweron=0;
static uint16_t vrefint;    //internal 1.2V reference conversion, filtered (below)
static volatile uint32_t adcticks;    //adc interrupts since boot (one per control period, pinout.h)
volatile uint8_t realdir=0;
volatile uint8_t overcurrent;    //soft current limit active (fitotal > SOFT_ILIMIT)

//Bus current in sine on a board without phase current pins. At the control sample (the counter's zero) all
//low sides conduct and the shunt reads ~0. In centre aligned pwm the switch state depends on the counter
//value only: between the smallest and the middle compare value one high side is on (the shunt carries that
//phase's current), between the middle and the largest two (minus the third phase's current). So after the
//control step the adc trigger moves to TIM1 CC4 in the middle of one of the two windows (they alternate),
//its conversion is read in a short second run of this isr, which sets the trigger back to TRGO. The bus
//current = (window 1 x I1 + window 2 x I2) / period. A window too short for a settled sample counts as 0
//(it carries little). BusSampleCheck() (SysTick) restores the trigger if a sample never came.
//The scan converts the channels in channel order, 20 adc clocks each (7.5 sample + 12.5) = 120 timer counts
//at the /6 adc clock: the shunt's sample comes busLead counts after the trigger. With the phase pins before
//it (bench: PA2, PA6, then the shunt on PA7) that is ~3.6 us, past the end of a 6 us window when the trigger
//sat in its middle: the watchdog saw the edge (20 hits per 20 s at 8 turns/s, 0.5 A real). The trigger
//comes on the down count (the counter falls while time runs), so it is placed busLead above the middle
//(measured: busLead below it hit edges in 9-14 us windows).
#define BUS_MINW 648                 //timer counts (9 us): the sample >= 4.5 us from both switching edges (at 6 us
                                     //the watchdog still saw the edges now and then: 3 per 20 s at 8 turns/s)
static volatile uint8_t busPending, busWhich;
static uint16_t busLead;
static volatile uint32_t busT;
static int32_t busI[2];              //0.01 A, the last sample of each window
static int32_t shuntCurrent(int32_t a){    //an adc reading of the shunt -> 0.01 A (0 below the zero)
	int32_t raw = a-(int32_t)itotaloffset, zero = cfgOrDefault(ITOTAL_ZERO, 0);
	return raw>0&&vrefint ? (int32_t)ITOTAL_DIVIDER*(raw*100+zero)*120/vrefint/100 : 0;
}
static void sort3(int32_t* v){    //ascending
	int32_t t;
	if(v[0]>v[1]){ t = v[0]; v[0] = v[1]; v[1] = t; }
	if(v[1]>v[2]){ t = v[1]; v[1] = v[2]; v[2] = t; }
	if(v[0]>v[1]){ t = v[0]; v[0] = v[1]; v[1] = t; }
}
static void adcTrigger(uint32_t src){ ADC1->ADCR = (ADC1->ADCR&~ADC_CR_TRGSEL)|src; }
void BusSampleCheck(void){
	if(busPending&&millis-busT>2){ busPending = 0; adcTrigger(ADC_CR_T1_TRIG); }
}


//commutation interrupt
void TIM1_BRK_UP_TRG_COM_IRQHandler(void){
  TIM_ClearITPendingBit(TIM1, TIM_IT_COM);
}

void ADC1_COMP_IRQHandler(void){
	if(ADC_GetFlagStatus(ADC1, ADC_IT_AWD) != RESET) {
    ADC_ClearFlag(ADC1, ADC_IT_AWD);
		//sine / FOC with phase current pins sample the shunt in the zero vector (all low sides on), where it
		//carries no current, only switching noise: at pwmres 2048 near full modulation the window is a few us,
		//the shunt (3rd in the scan) lands on an edge, ~80 A equivalent spikes (each one a break, the gates off
		//for 2 periods, the current loop restarting from 0 V against the back-EMF: a 7 A kick). There only the
		//conversion inside an active vector (CC4, busPending, see busv below) counts.
		if(busPending||!(COMMUTATION(DRIVEMODE)!=COMMUT_BLOCK&&FocAvailable())) TIM_GenerateEvent(TIM1, TIM_EventSource_Break);
  }
	if(RESET != ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
		if(busPending){    //the CC4 sample inside an active vector: the bus current, then back to the control sample
			adcTrigger(ADC_CR_T1_TRIG);
			busPending = 0;
			busI[busWhich] = shuntCurrent(analogRead(ITOTALPIN));
			ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
			return;
		}
		adcticks++;
		//the compare values active in this period (written by the last control step, loaded at this update)
		int32_t cc[3] = { (int32_t)TIM1->CCR1, (int32_t)TIM1->CCR2, (int32_t)TIM1->CCR3 };
		{    //the internal 1.2 V reference (VDD for the bus voltage and the shunt current): a reading now and then
		     //is far off (seen ~26 instead of ~1489 counts: the bus voltage read 57x, the FOC voltage limit
		     //overflowed negative, one period without q voltage, a 4 A current kick, a knock at speed with field
		     //weakening, ~1 per second). VDD changes slowly: filtered, readings 1/16 off it ignored (not at boot).
			int32_t vr = (int32_t)(ADC1->ADDR15&0xFFF), f = vrefint;
			if(!f||poweron<127) f = f ? f+(vr-f)/4 : vr;
			else if(vr>f-f/16&&vr<f+f/16) f += (vr-f)/8;
			vrefint = (uint16_t)f;
		}
		if(poweron<127){//cancel out adc offset
			itotaloffset+=analogRead(ITOTALPIN);
			poweron++;
		}else if(poweron==127){//divide by 128
			itotaloffset = itotaloffset>>7;
			for(uint8_t i=0;i<ADCCOUNT;i++){
				if(adcs[i].io==ITOTALPIN){
					busLead = 23;    //the middle of the 7.5 cycle sample, 120 per channel converted before the shunt
					for(uint32_t m=ADC1->ADCHS&((1u<<adcs[i].channel)-1);m;m&=m-1) busLead += 120;
					if(AWDG){    //hardware current limit, only if the current sense pin has an adc channel
						ADC_AnalogWatchdogCmd(ADC1, ENABLE);
						ADC_AnalogWatchdogThresholdsConfig(ADC1, (uint16_t)(AWDG+itotaloffset), 0);
						ADC_AnalogWatchdogSingleChannelConfig(ADC1, adcs[i].channel);
						ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
					}
					break;
				}
			}
			poweron++;
		}else{
			//integer math only: this runs every pwm period, with soft-float double math the isr took 45% cpu (gcc).
			//VDD = 1.2V*4096/vrefint, so DIVIDER*adc*VDD*100/4096 (unit 0.01) = DIVIDER*adc*120/vrefint.
			//32 bit is enough for dividers below 4369.
			if(vrefint){
				uint32_t vg = VBAT_GAIN;    //0.001: gain*adc*120/1000 = gain*adc/25*3 (32 bit)
				int32_t vb = (int32_t)(vg&&vg!=0xFFFF ? vg*analogRead(VBATPIN)/25u*3u/vrefint
				                                      : (uint32_t)VBAT_DIVIDER*analogRead(VBATPIN)*120u/vrefint);
				//and the bus voltage channel's own glitches (bus traffic couples into the adc): a reading 1/12 off
				//the last is held for up to 3 periods (a real change, regeneration, takes far longer)
				static uint8_t vbHeld;
				if(vbat>0&&(vb>vbat+vbat/12||vb<vbat-vbat/12)&&vbHeld<3) vbHeld++;
				else{ vbat = vb; vbHeld = 0; }
				//a zero below adc 0 (ITOTAL_ZERO) reads 0 up to that current (0.76 A on the bench board), then
				//the shifted value
				itotal = shuntCurrent(analogRead(ITOTALPIN));
			}
			uint8_t foc = COMMUTATION(DRIVEMODE)==COMMUT_FOC, sine = COMMUTATION(DRIVEMODE)==COMMUT_SINE;
			//sine without phase current pins: the bus current from the active vectors (above); with phase pins
			//(itotal from them, FocMeasure()) the same samples for the adc watchdog
			uint8_t busv = (foc||sine)&&(AWDG||!FocAvailable())&&ITOTALPIN<PINCOUNT&&(TIM1->BDTR&TIM_BDTR_MOEN);
			sort3(cc);
			if(busv) itotal = ((cc[1]-cc[0])*busI[0]+(cc[2]-cc[1])*busI[1])/(int32_t)PWM_PERIOD;
			else busI[0] = busI[1] = 0;
			if(foc||sine){
				//phase currents (if the board has the pins); itotal from them: the shunt reads ~0 at this
				//sample time (all low sides on)
				FocMeasure();
			}
			avgvbat();
			avgItotal();
			//SOFT_ILIMIT in A against the 64 sample average (~15 ms): one adc count of a direct shunt is
			//~0.3 A and the raw value spikes on switching noise. Fast faults are the adc watchdog's job (AWDG).
			overcurrent = SOFT_ILIMIT&&fitotal>(int32_t)SOFT_ILIMIT*100;
			//the only place the gates are switched while running
			gatesOutput(!(gatesoff||coastreq||overcurrent||(foc ? FocHoldOff() : sine&&focTrip)));
			HALL1.CMDDIR = -dir;
			HALLModuleCalc(&HALL1);    //also with the gates off: speed and odometry keep tracking a coasting wheel
			if(COMMUTATION(DRIVEMODE)==COMMUT_SINE){
				//torque direction from the sign (the angle follows the rotor, not the command);
				//negated so a positive command gives positive speed. +-32000: linear svpwm range
				int32_t q = -pwm, adv = 0;
				//field weakening: a command beyond the linear range (32000, SINE_VOLT 800..1000 permille)
				//becomes phase advance in the direction of motion, up to FW_ADVANCE degrees at 40000
				int32_t fw = cfgOrDefault(FW_ADVANCE, 0);
				if(q>32000){ adv = fw ? (q-32000)*fw*182/8000 : 0; q = 32000; if(adv>fw*182) adv = fw*182; }
				else if(q<-32000){ adv = fw ? (q+32000)*fw*182/8000 : 0; q = -32000; if(adv<-fw*182) adv = -fw*182; }
				ipark1.Ds = 0;
				ipark1.Qs = q;
				ipark1.Theta = (int16_t)(HALL1.Angle+adv);    //182 = 65536/360
				IPARK_MACRO1(&ipark1);
				pwm_gen.Mode = SEVENMODE;
				pwm_gen.Alpha = ipark1.Alpha;
				pwm_gen.Beta  = ipark1.Beta;
				PWM_GEN_calc(&pwm_gen);
				Update_PWM(&pwm_gen);
				FocObserve();
			}else if(foc){
				FocControl();
			}
			if(busv){    //the next bus sample: the other window of the vectors just written. The compare registers
				//have no preload: the new values act at once, also in the period of this sample (the window of the
				//values read at the isr's start was ~12 deg el off at 10 turns/s in sine: 8 watchdog hits per 30 s)
				busWhich ^= 1;
				int32_t n[3] = { (int32_t)TIM1->CCR1, (int32_t)TIM1->CCR2, (int32_t)TIM1->CCR3 };
				sort3(n);
				int32_t a = n[busWhich], b = n[busWhich+1], c = (a+b)/2+busLead;
				if(b-a>=BUS_MINW&&c<(int32_t)PWM_PERIOD){
					TIM1->CCR4 = (uint16_t)c;
					busT = millis;
					busPending = 1;
					adcTrigger(ADC_CR_TIM1_CC4);
				}else busI[busWhich] = 0;
			}
		}
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
}
