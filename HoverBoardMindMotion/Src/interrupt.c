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
volatile uint8_t realdir=0;
volatile uint8_t overcurrent;    //soft current limit active (fitotal > SOFT_ILIMIT)
//hardware current limit (the adc watchdog AWDG, or a comparator on the break input OCPPIN): each break
//keeps the gates off for HWOC_HOLD control periods (with AOE set the timer re-armed them at the next
//update: a short circuit was chopped every period, never stopped). Only the break input counts: a hit
//adds HWOC_HIT to a score that falls by 1 per control period, ~10 hits in 100 ms set hwOcFault
//(speedupdate(): ERR_DRV_FAULT); the adc watchdog never sets a fault (its samples see switching noise)
#define HWOC_HOLD 2
#define HWOC_HIT 44
volatile uint8_t hwOcFault;
static uint8_t hwOcHold;
static uint16_t hwOcScore;

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
//Phase current pins first. The scan converts its channels in channel order, 1.67 us each from the
//counter's zero (the centre of the low side time); a phase pin behind other channels is sampled late (F031
//board: vbat ch4, shunt ch7, phase C ch8, B ch9: B at ~5.6 us). Under load the low side time of the phase
//with the highest duty fell below that, B read a saturated amplifier (-58 A), CURRENT_LIMIT_VIOLATION at
//2 A of I_q (300 rpm, a hand on the wheel). In sine / FOC the control sample leaves out adcRest (the
//channels below the highest phase pin that are not phase pins) and the CC4 run after the control step (the
//bus sample, else one at mid period) converts them too: every channel keeps its own result register.
//adcRest 0: nothing to leave out (the phase pins are the lowest channels, as on the SPIN05 bench board).
extern uint32_t adcRest;    //initialize.c adc_Init()
static volatile uint8_t busPending, busWhich;
static uint16_t busLead;
static volatile uint32_t busT;
static int32_t busI[2];              //0.01 A, the last sample of each window
static int32_t shuntCurrent(int32_t a){    //an adc reading of the shunt -> 0.01 A (0 below the zero)
	int32_t raw = a-(int32_t)itotaloffset, zero = cfgOrDefault(ITOTAL_ZERO, 0);
	//divider * mV-ish / 100, divided first: the product in one went negative above ~144 A at divider 333
	int32_t x = raw>0&&vrefint ? (raw*100+zero)*120/vrefint : 0;
	return x>0 ? (int32_t)((uint32_t)ITOTAL_DIVIDER*(uint32_t)x/100u) : 0;
}
static void sort3(int32_t* v){    //ascending
	int32_t t;
	if(v[0]>v[1]){ t = v[0]; v[0] = v[1]; v[1] = t; }
	if(v[1]>v[2]){ t = v[1]; v[1] = v[2]; v[2] = t; }
	if(v[0]>v[1]){ t = v[0]; v[0] = v[1]; v[1] = t; }
}
static void adcTrigger(uint32_t src){ ADC1->ADCR = (ADC1->ADCR&~ADC_CR_TRGSEL)|src; }
//MCU temperature, 0.1 C (DS_MM32SPIN05 table 39 note 5: 25 + (value * VDDA - offset * 3300) / (4096 * 4.801 mV/C),
//offset per chip at 0x1FFFF7F6). The 0.6 us sample is enough here: 7.5..239.5 cycles read alike (43.2..43.4 C).
int32_t McuTemp(void){
	if(!vrefint) return 0;
	//uV: value * 1.2 V * 4096 / vrefint / 4096, offset * 3.3 V / 4096 (= * 103125 / 128); 32 bit up to 4095 counts
	int32_t uv = (int32_t)(ADC1->ADDR14&0xFFF)*120000/vrefint*10-(int32_t)(*(volatile uint16_t*)0x1FFFF7F6&0xFFF)*103125/128;
	return 250+uv*10/4801;
}
void BusSampleCheck(void){
	if(busPending&&millis-busT>2){ busPending = 0; adcTrigger(ADC_CR_T1_TRIG); }    //(adcRest: the next CC4 run restores the scan)
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
		if(busPending||!(COMMUTATION(DRIVEMODE)!=COMMUT_BLOCK&&FocAvailable())){
			TIM_GenerateEvent(TIM1, TIM_EventSource_Break);
			gatesOutput(0);    //AOE off too: the gates stay off until the control step (HWOC_HOLD) arms them
			TIM1->SR = ~TIM_SR_BI;    //no hit (only the break input counts)
			hwOcHold = HWOC_HOLD+1;
		}
  }
	if(RESET != ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
		if(busPending){    //the CC4 sample inside an active vector: the bus current, then back to the control sample
			ADC1->ADCHS &= ~adcRest;
			adcTrigger(ADC_CR_T1_TRIG);
			busPending = 0;
			busI[busWhich] = shuntCurrent(analogRead(ITOTALPIN));
			ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
			return;
		}
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
			TIM1->SR = ~TIM_SR_BI;    //break events before this (start-up) do not count
			poweron++;
		}else{
			//integer math only: this runs every pwm period, with soft-float double math the isr took 45% cpu (gcc).
			//VDD = 1.2V*4096/vrefint, so DIVIDER*adc*VDD*100/4096 (unit 0.01) = DIVIDER*adc*120/vrefint.
			//32 bit is enough for dividers below 4369 (larger ones are clamped).
			if(vrefint){
				uint32_t vg = VBAT_GAIN, vd = VBAT_DIVIDER, a = analogRead(VBATPIN);    //gain 0.001: gain*adc*120/1000 = gain*adc/25*3
				if(vd>4368) vd = 4368;
				int32_t vb = (int32_t)(vg&&vg!=0xFFFF ? vg*a/25u*3u/vrefint : vd*a*120u/vrefint);
				//and the bus voltage channel's own glitches (bus traffic couples into the adc): a reading 1/12 off
				//the last is held for up to 3 periods (a real change, regeneration, takes far longer)
				//block brake above BAT_FULL + 0.5 V: the full short at once, on the raw reading (bldc.c). Braking current
				//into a bus that cannot take it (a full battery, a bench supply) raises it ~20 V/ms; the main loop's
				//2 ms on the filtered value let a bench supply reach its 41 V OVP (F031, braking from 5 turns/s)
				if(blockbrake&&BAT_SET(BAT_FULL)&&vb*10>(int32_t)BAT_FULL+500) TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_PERIOD+1;
				static uint8_t vbHeld;
				if(vbat>0&&(vb>vbat+vbat/12||vb<vbat-vbat/12)&&vbHeld<3) vbHeld++;
				else{ vbat = vb; vbHeld = 0; }
				//a zero below adc 0 (ITOTAL_ZERO) reads 0 up to that current (0.76 A on the bench board), then
				//the shifted value
				itotal = shuntCurrent(analogRead(ITOTALPIN));
			}
			uint8_t foc = COMMUTATION(DRIVEMODE)==COMMUT_FOC, sine = COMMUTATION(DRIVEMODE)==COMMUT_SINE;
			//sine: the bus current from the active vectors (above; with phase pins FocMeasure() replaces it while
			//their samples are valid, below the top modulation); FOC: the same samples for the adc watchdog
			uint8_t busv = (sine||(foc&&AWDG))&&ITOTALPIN<PINCOUNT&&(TIM1->BDTR&TIM_BDTR_MOEN);
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
			if(TIM1->SR&TIM_SR_BI){    //a break event since the last period (adc watchdog or break input)
				TIM1->SR = ~TIM_SR_BI;
				hwOcHold = HWOC_HOLD;
				if((hwOcScore += HWOC_HIT)>=10*HWOC_HIT){ hwOcFault = 1; hwOcScore = 0; }
			}else if(hwOcHold) hwOcHold--;
			if(hwOcScore) hwOcScore--;
			//the only place the gates are switched while running
			gatesOutput(!(gatesoff||coastreq||overcurrent||hwOcHold||(foc ? FocHoldOff() : sine&&focTrip)));
			HALL1.CMDDIR = -dir;
			HALLModuleCalc(&HALL1);    //also with the gates off: speed and odometry keep tracking a coasting wheel
			if(COMMUTATION(DRIVEMODE)==COMMUT_SINE){
				//torque direction from the sign (the angle follows the rotor, not the command);
				//negated so a positive command gives positive speed. +-32000: linear svpwm range
				int32_t q = -pwm, adv = 0;
				//field weakening: a command beyond the linear range (32000, SINE_VOLT 800..1000 permille)
				//becomes phase advance in the direction of motion, up to FW_ADVANCE degrees at 40000
				int32_t fw = cfgOrDefault(FW_ADVANCE, 0);
				if(fw>90) fw = 90;    //degrees; (q-32000)*fw*182 in 32 bit up to ~1470
				if(q>32000){ adv = fw ? (q-32000)*fw*182/8000 : 0; q = 32000; if(adv>fw*182) adv = fw*182; }
				else if(q<-32000){ adv = fw ? (q+32000)*fw*182/8000 : 0; q = -32000; if(adv<-fw*182) adv = -fw*182; }
				ipark1.Ds = 0;
				ipark1.Qs = q*derate>>10;    //temperature derating (TEMP_DERATE)
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
			int32_t at = 0;    //a CC4 run at this counter value
			if(busv){    //the next bus sample: the other window of the vectors just written. The compare registers
				//have no preload: the new values act at once, also in the period of this sample (the window of the
				//values read at the isr's start was ~12 deg el off at 10 turns/s in sine: 8 watchdog hits per 30 s)
				busWhich ^= 1;
				int32_t n[3] = { (int32_t)TIM1->CCR1, (int32_t)TIM1->CCR2, (int32_t)TIM1->CCR3 };
				sort3(n);
				int32_t a = n[busWhich], b = n[busWhich+1], c = (a+b)/2+busLead;
				if(b-a>=BUS_MINW&&c<(int32_t)PWM_PERIOD) at = c;
				else busI[busWhich] = 0;
			}
			if(adcRest&&(foc||sine)){    //phase pins first (adcRest): the rest in a CC4 run, the bus sample's or one at mid period
				if(!at) at = (int32_t)PWM_PERIOD/2;
				ADC1->ADCHS |= adcRest;
			}
			if(at){
				TIM1->CCR4 = (uint16_t)at;
				busT = millis;
				busPending = 1;
				adcTrigger(ADC_CR_TIM1_CC4);
			}
		}
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
}
