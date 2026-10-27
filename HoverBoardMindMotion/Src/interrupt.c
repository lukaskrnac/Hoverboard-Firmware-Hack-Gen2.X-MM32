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
static uint16_t vrefint;    //last internal 1.2V reference conversion
static volatile uint32_t adcticks;    //adc interrupts since boot (one per pwm period)
volatile uint8_t realdir=0;
static uint8_t buzzLevel;
volatile uint8_t overcurrent;    //soft current limit active (fitotal > SOFT_ILIMIT)


//commutation interrupt
void TIM1_BRK_UP_TRG_COM_IRQHandler(void){
  TIM_ClearITPendingBit(TIM1, TIM_IT_COM);
}

void ADC1_COMP_IRQHandler(void){
	if(ADC_GetFlagStatus(ADC1, ADC_IT_AWD) != RESET) {
    ADC_ClearFlag(ADC1, ADC_IT_AWD);
		TIM_GenerateEvent(TIM1, TIM_EventSource_Break);
  }
	if(RESET != ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
		adcticks++;
		if(poweron<127){//cancel out adc offset
			itotaloffset+=analogRead(ITOTALPIN);
			poweron++;
		}else if(poweron==127){//divide by 128
			itotaloffset = itotaloffset>>7;
			if(AWDG){    //hardware current limit, only if the current sense pin has an adc channel
				for(uint8_t i=0;i<ADCCOUNT;i++){
					if(adcs[i].io==ITOTALPIN){
						ADC_AnalogWatchdogCmd(ADC1, ENABLE);
						ADC_AnalogWatchdogThresholdsConfig(ADC1, (uint16_t)(AWDG+itotaloffset), 0);
						ADC_AnalogWatchdogSingleChannelConfig(ADC1, adcs[i].channel);
						ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
						break;
					}
				}
			}
			poweron++;
		}else{
			//integer math only: this runs every pwm period, with soft-float double math the isr took 45% cpu (gcc).
			//VDD = 1.2V*4096/vrefint, so DIVIDER*adc*VDD*100/4096 (unit 0.01) = DIVIDER*adc*120/vrefint.
			//32 bit is enough for dividers below 4369.
			vrefint = ADC1->ADDR15;
			if(vrefint){
				vbat = (int32_t)((uint32_t)VBAT_DIVIDER*analogRead(VBATPIN)*120u/vrefint);
				//a zero below adc 0 (ITOTAL_ZERO) reads 0 up to that current (0.76 A on the bench board), then
				//the shifted value
				int32_t raw = (int32_t)analogRead(ITOTALPIN)-(int32_t)itotaloffset;
				int32_t zero = cfgOrDefault(ITOTAL_ZERO, 0);
				itotal = raw>0 ? (int32_t)ITOTAL_DIVIDER*(raw*100+zero)*120/vrefint/100 : 0;
			}
			uint8_t foc = COMMUTATION(DRIVEMODE)==COMMUT_FOC, sine = COMMUTATION(DRIVEMODE)==COMMUT_SINE;
			if(foc||sine){
				//phase currents (if the board has the pins); itotal from them: the shunt reads ~0 at this
				//sample time (all low sides on)
				FocMeasure();
			}
			if(buzzTone||buzzLevel){    //passive buzzer: half the pwm frequency, 2.2 kHz
				buzzLevel = buzzTone&&!buzzLevel;
				digitalWrite(BUZZERPIN, buzzLevel);
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
		}
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
}
