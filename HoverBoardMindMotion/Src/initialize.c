//initialize all pheberals
#include "pinout.h"
#include "hal_gpio.h"
#include "hal_rcc.h"
#include "hal_comp.h"
#include "hal_tim.h"
#include "initialize.h"
#include "hal_conf.h"
#include "hardware.h"
#include "main.h"

//normal io
void io_init(void){
	//enable gpio clock
	RCC->AHBENR |= RCC_AHBENR_GPIOA|RCC_AHBENR_GPIOB|RCC_AHBENR_GPIOC|RCC_AHBENR_GPIOD;    //(RCC_AHBPeriphClockCmd x 4)
	if(LEDRPIN<PINCOUNT){
		pinMode(LEDRPIN, OUTPUT);
	}
	if(LEDGPIN<PINCOUNT){
		pinMode(LEDGPIN, OUTPUT);
	}
	if(LEDBPIN<PINCOUNT){
		pinMode(LEDBPIN, OUTPUT);
	}
	if(LEDUPIN<PINCOUNT){
		pinMode(LEDUPIN, OUTPUT);
	}
	if(LEDDPIN<PINCOUNT){
		pinMode(LEDDPIN, OUTPUT);
	}
	if(LATCHPIN<PINCOUNT){
		pinMode(LATCHPIN, OUTPUT);
	}
	if(BUTTONPIN<PINCOUNT){
		pinMode(BUTTONPIN, INPUT);
	}
	BuzzerInit();
	static const uint8_t pull[] = { INPUT, INPUT, INPUT_PULLUP, INPUT_PULLDOWN };    //0 / 1 floating, 2 up, 3 down
	if(CHARGEPIN<PINCOUNT){
		uint32_t p = cfgOrDefault(CHARGE_PULL, 1);
		pinMode(CHARGEPIN, pull[p<=3 ? p : 0]);
	}
	for(uint8_t k=0;k<2;k++){    //infrared foot sensors: IR_MODE bits 2-3 (+4 for the second)
		if(IR_PIN(k)<PINCOUNT) pinMode(IR_PIN(k), pull[cfgOrDefault(IR_MODE, 0)>>(4*k+2)&3]);
	}
}

//The buzzer: an active one (own oscillator) on a gpio output, a passive one (BUZZER_PASSIVE) gets a 2.2 kHz
//square wave from a timer output (BuzzerEntry(); BuzzerSet() switches the pin between that and gpio low).
//A passive buzzer on a pin without one of those timers stays silent (PinFinder warns at the detection).
static uint8_t buzTimer;    //1 = the passive buzzer's timer runs

int8_t BuzzerEntry(void){    //the buzs entry of BUZZERPIN, -1 none
	for(uint8_t i=0;i<BUZCOUNT;i++) if(buzs[i].io==BUZZERPIN) return (int8_t)i;
	return -1;
}

void BuzzerInit(void){
	if(BUZZERPIN>=PINCOUNT) return;
	digitalWrite(BUZZERPIN, 0);
	pinMode(BUZZERPIN, OUTPUT);
	int8_t e = BuzzerEntry();
	if(!cfgBool(BUZZER_PASSIVE, 0)||e<0) return;
	uint8_t t = buzs[e].tim&3;
	TIM_TypeDef* tim = (TIM_TypeDef*)(TIM14_BASE+0x400u*t);    //TIM14, TIM16, TIM17 are 0x400 apart
	RCC->APB2ENR |= RCC_APB2ENR_TIM14<<t;                       //enable bits 16, 17, 18
	tim->PSC = SystemCoreClock/1000000-1;    //1 us
	tim->ARR = 454;                          //2.2 kHz
	tim->CCR1 = 227;
	tim->CCMR1 = TIM_OCMode_PWM1|TIM_CCMR1_OC1PE;
	tim->CCER = (buzs[e].tim&BZ_N) ? TIM_CCER_CC1NEN : TIM_CCER_CC1EN;
	if(t) tim->BDTR = TIM_BDTR_MOEN;         //TIM16 / TIM17: main output enable (TIM14 has no BDTR)
	tim->EGR = TIM_EGR_UG;
	tim->CR1 = TIM_CR1_CEN;
	pinModeAF(BUZZERPIN, buzs[e].af);
	buzTimer = 1;
}

void BuzzerSet(uint8_t on){
	if(BUZZERPIN>=PINCOUNT) return;
	if(cfgBool(BUZZER_PASSIVE, 0)) pinMode(BUZZERPIN, on&&buzTimer ? OUTPUT_AF : OUTPUT);    //gpio: low
	else digitalWrite(BUZZERPIN, on);
}

void HALL_Init(void){
	pinMode(HALLAPIN, INPUT);
	pinMode(HALLBPIN, INPUT);
	pinMode(HALLCPIN, INPUT);
}

//The timer XORs its CH1, CH2 and CH3 inputs (TI1S): the hall pins A, B, C must be on those three channels of
//one timer, in any order (a pin can be on several channels: halltims has an entry for each). The halltims
//entries of A, B, C in e; returns 1 TIM2, 0 TIM3, -1 if the pins are wired to no such combination.
int8_t HallTimEntries(uint8_t* e){
	int8_t h = 0;    //backtracking over the entries of A (h 0), B, C (pinstorage 0-2)
	e[0] = 0xFF;
	while(h>=0){
		uint8_t i = ++e[h];
		if(i>=TIMCOUNT){ h--; continue; }
		if(halltims[i].io!=pinstorage[h]) continue;
		uint8_t c = halltims[i].tc, fits = 1;
		for(int8_t k=0;k<h;k++){    //the same timer, another channel
			uint8_t d = (uint8_t)(c^halltims[e[k]].tc);
			if(d&HT_TIM2||!(d&3)) fits = 0;
		}
		if(!fits) continue;
		if(h==2) return (int8_t)(c>>2);
		e[++h] = 0xFF;
	}
	return -1;
}

uint8_t HALLTIM_Init(u32 arr, u16 psc){    //hall sensor hardware speed sensing
	uint8_t e[3];
	int8_t t = HallTimEntries(e);
	uint8_t tim2 = t!=0;    //no valid combination: speed sensing will not work, but keep it deterministic
	for(uint8_t h=0;h<3&&t>=0;h++) pinModeAF(halltims[e[h]].io, halltims[e[h]].af);
	//register writes, the values of the vendor HAL calls this replaced (TIM_TimeBaseInit, TIM_ICInit, ...)
	RCC->APB1ENR |= RCC_APB1ENR_TIM2|RCC_APB1ENR_TIM3;
	TIM_TypeDef* tim = tim2 ? TIM2 : TIM3;
	tim->ARR = arr;
	tim->PSC = psc;
	tim->EGR = TIM_EGR_UG;
	//IC1 = TRC (the XOR of CH1-CH3, TI1S), rising. Filter fDTS/32, 8 samples (~14 us; input filter clock
	//18 MHz: CKD/4): a switching spike on a hall line made two captures within a microsecond, the edge was
	//measured as 0 us (speed 0 in the angle tracker: a freeze, then an 80 deg jump, a 20 A trip with field
	//weakening at 600 rpm)
	tim->CCMR1 = 0xFu<<4|TIM_CCMR1_CC1S_TRC;
	tim->CCER = TIM_CCER_CC1EN;
	tim->DIER = TIM_DIER_CC1I;    //(the interrupt is not enabled in the NVIC)
	tim->CR2 = TIM_CR2_TI1S;
	tim->SMCR = TIM_SMCR_TS_TI1F_ED|TIM_SMCR_SMS_RESET|TIM_SMCR_MSM;    //every hall edge resets the counter
	tim->CR1 = TIM_CR1_CKD_DIV4|TIM_CR1_URS|TIM_CR1_CEN;    //up, 1 us ticks
	return tim2;
}


//6 bldc pin
void BLDC_init(void){
	for(uint8_t i=0;i<3;i++){    //high sides PA8-10 (CH1-3), low sides PB13-15 (CH1N-3N)
		gpioAF(GPIOA, (uint8_t)(8+i), GPIO_AF_2);
		gpioAF(GPIOB, (uint8_t)(13+i), GPIO_AF_2);
		gpioMode(GPIOA, (uint8_t)(8+i), GPIO_Mode_AF_PP);
		gpioMode(GPIOB, (uint8_t)(13+i), GPIO_Mode_AF_PP);
	}
}

//timer1 automatic output. Register writes, the values the vendor HAL's struct inits wrote (TIM_TimeBaseInit,
//TIM_OCxInit, TIM_BDTRConfig: 1 KB less, checked by a register dump of the running firmware)
void TIM1_init(u16 arr, u16 psc){
	RCC->APB2ENR |= RCC_APB2ENR_TIM1;
	//centre aligned 1, clock division 1, an update every period (repetition 1)
	TIM1->CR1 = (TIM1->CR1&~(TIM_CR1_CKD|TIM_CR1_CMS|TIM_CR1_DIR))|TIM_CounterMode_CenterAligned1;
	TIM1->RCR = 1;
	TIM1->ARR = arr;
	TIM1->PSC = psc;
	TIM1->EGR = TIM_PSCReloadMode_Immediate;
	//channels 1-3: PWM2, both outputs on, pulse 0, low sides inverted if configured; channel 4: PWM2 at 1, the
	//adc trigger (output on, no pin)
	TIM1->CCMR1 = (TIM1->CCMR1&~(TIM_CCMR1_OC1M|TIM_CCMR1_OC2M))|TIM_OCMode_PWM2|TIM_OCMode_PWM2<<8;
	TIM1->CCMR2 = (TIM1->CCMR2&~(TIM_CCMR2_OC3M|TIM_CCMR2_OC4M))|TIM_OCMode_PWM2|TIM_OCMode_PWM2<<8;
	uint32_t np = INVERT_LOWSIDE==1 ? TIM_CCER_CC1NP : 0, ch = TIM_CCER_CC1EN|TIM_CCER_CC1NEN|np;
	TIM1->CCER = ch|ch<<4|ch<<8|TIM_CCER_CC4EN;
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
	TIM1->CCR4 = 1;
	//idle states (MOE clear: gates off, a break): the pin level as is, the polarity bits do not apply. Inverted
	//low sides idle high, else gates off would switch all three low sides on and short the motor (it brakes)
	uint32_t ois = INVERT_LOWSIDE==1 ? TIM_CR2_OIS1N|TIM_CR2_OIS2N|TIM_CR2_OIS3N : 0;
	TIM1->CR2 = (TIM1->CR2&~(TIM_CR2_OIS1|TIM_CR2_OIS1N|TIM_CR2_OIS2|TIM_CR2_OIS2N|TIM_CR2_OIS3|TIM_CR2_OIS3N|TIM_CR2_OIS4))|ois|TIM_CR2_CCPC;
	TIM1->DIER |= TIM_DIER_COMI;
	//automatic output enable (the gates come on at the next update), break input, dead time 1. MOE stays
	//clear here as after the HAL's BDTR write: AOE sets it
	TIM1->BDTR = TIM_BDTR_OSSR|TIM_BDTR_OSSI|1|TIM_BDTR_BKEN|(AWDG ? TIM_BDTR_BKP : 0)|TIM_BDTR_AOEN;
	if(!AWDG){
		if(OCPREFPIN<PINCOUNT){
			/*
			RCC_APB2PeriphClockCmd(RCC_APB2ENR_COMP, ENABLE);
			GPIO_InitTypeDef GPIO_InitStruct;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStruct.GPIO_Pin = HARD_ILIMIT_PIN;
			GPIO_Init(HARD_ILIMIT_PORT, &GPIO_InitStruct);
			GPIO_InitStruct.GPIO_Pin = HARD_ILIMIT_COMP_REF_PIN;
			GPIO_Init(HARD_ILIMIT_COMP_REF_PORT, &GPIO_InitStruct);

			COMP_InitTypeDef COMP_InitStructure;
			COMP_DeInit(COMP1);
			COMP_InitStructure.COMP_InvertingInput 		= HARD_ILIMIT_COMP_INVERTING;
			COMP_InitStructure.COMP_NonInvertingInput = HARD_ILIMIT_COMP_NONINVERTING;
			COMP_InitStructure.COMP_Output    				= COMP_Output_TIM1BKIN;
			COMP_InitStructure.COMP_OutputPol 				= COMP_OutputPol_NonInverted;
			COMP_InitStructure.COMP_Hysteresis  			= COMP_Hysteresis_High;
			COMP_InitStructure.COMP_Mode        			= COMP_Mode_HighSpeed;
			COMP_InitStructure.COMP_Filter      			= COMP_Filter_4_Period;
			COMP_Init(COMP1, &COMP_InitStructure);
			COMP_Cmd(COMP1, ENABLE);
			*/
		}else if(OCPPIN<PINCOUNT){
			pinMode(OCPPIN, INPUT);
			for(uint8_t i=0;i<TIMBKCOUNT;i++){
				if(ocps[i].io==OCPPIN){
					pinModeAF(OCPPIN, ocps[i].af);
				}
			}
		}
	}
	TIM1->CR1 |= TIM_CR1_CEN;
}

//interrupt
void NVIC_Configure(u8 ch, u8 pri){
	NVIC_InitTypeDef  NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = ch;
	NVIC_InitStruct.NVIC_IRQChannelPriority = pri;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}
//uart
void UARTX_Init(u32 baudrate,uint8_t uart){    //8N1, rx + tx, dma requests; registers as the HAL's UART_Init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);
	UART_TypeDef* u = uart==1 ? UART1 : UART2;
	u->CCR = (u->CCR&~(UART_CCR_CHAR|UART_CCR_SPB0|UART_CCR_SPB1|UART_CCR_PEN|UART_CCR_PSEL))|UART_WordLength_8b;
	uint32_t div = SystemCoreClock/baudrate;    //both APB buses run at the core clock (system_mm32spin05.c: no prescaler)
	u->BRR = div/16;
	u->FRA = div%16;
	u->GCR = (u->GCR&~UART_GCR_AUTOFLOW)|UART_GCR_TX|UART_GCR_RX|UART_GCR_UART|UART_GCR_DMA;
}

//the uart whose tx entry is TXPIN and rx entry is RXPIN (a pin can be on two uarts: PB7 is UART1 RX and
//UART2 TX on the SPIN05); with no full pair the uart with one of them
uint8_t UART_GPIO_Init(void){
	uint8_t uart=1, best=0, t=UARTCOUNT, r=UARTCOUNT;
	for(uint8_t u=1;u<=2;u++){
		uint8_t ut=UARTCOUNT, ur=UARTCOUNT;
		for(uint8_t i=0;i<UARTCOUNT;i++){
			if(uarts[i].uart!=u) continue;
			if(uarts[i].tx&&uarts[i].io==TXPIN) ut=i;
			if(!uarts[i].tx&&uarts[i].io==RXPIN) ur=i;
		}
		uint8_t n=(uint8_t)((ut<UARTCOUNT)+(ur<UARTCOUNT));
		if(n>best){ best=n; uart=u; t=ut; r=ur; }
	}
	if(t<UARTCOUNT){
		//open drain: several boards share one RemoteUartBus line, the master's rx must have a pull-up. Its
		//rising edges limit the baud (above 230400 with the usual pull-ups): txpushpull for one board alone
		pinModeAF(uarts[t].io,uarts[t].af);
		pinMode(uarts[t].io, cfgBool(TX_PUSHPULL, 0) ? OUTPUT_AF : OUTPUT_AF_OD);
	}
	if(r<UARTCOUNT){
		pinModeAF(uarts[r].io,uarts[r].af);
		pinMode(uarts[r].io, INPUT);
	}
	return uart;
}

void adc_Init(void){
	ADC_InitTypeDef  ADC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_PRESCALE = ADC_PCLK2_PRESCALE_6;							//for 72MHz
	ADC_InitStructure.ADC_Mode = ADC_Mode_Single_Period;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC1_ExternalTrigConv_T1_TRIG;    //TIM1 TRGO: every pwmDiv-th update (main.c)
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ExternalTrigConvCmd(ADC1,ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_VoltReference, 0, ADC_SampleTime_7_5Cycles);
	ADC_VrefintCmd(ENABLE);
	ADC1->ADCHS |= ADC_CHSR_CHT;    //the internal temperature sensor (channel 14, after the pins in the scan: busLead unchanged)
	ADC_Cmd(ADC1, ENABLE);
	//the battery, bus shunt and (both) phase current pins that have an adc channel: analog, in the scan
	uint8_t phases = IPHASEAPIN<PINCOUNT&&IPHASEBPIN<PINCOUNT;
	for(uint8_t i=0;i<ADCCOUNT;i++){
		uint16_t io = adcs[i].io;
		if(io==VBATPIN||io==ITOTALPIN||(phases&&(io==IPHASEAPIN||io==IPHASEBPIN))){
			pinMode(io, INPUT_ADC);
			ADC_RegularChannelConfig(ADC1, adcs[i].channel, 0, ADC_SampleTime_7_5Cycles);
		}
	}
#ifdef MCU_OPAMPS
	//SPIN25: op-amp n drives its output pin (OP1 PA6, OP2 PB2, OP3 PB10; OP4's PA9 is a PWM pin); enabled
	//when that pin is a configured current input (gain and inputs: the board's resistors). OPAMP_CSR
	//bit 8*n, the op-amps are on the COMP interface clock.
	static const uint8_t opOut[3] = { PA6, PB2, PB10 };
	uint32_t csr = 0;
	for(uint8_t k=0;k<3;k++){
		if(opOut[k]==ITOTALPIN||opOut[k]==IPHASEAPIN||opOut[k]==IPHASEBPIN) csr |= 1u<<(8*k);
	}
	if(csr){
		RCC->APB2ENR |= 1u<<15;
		*(volatile uint32_t*)0x40013C10 = csr;
	}
#endif

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}

//Waits are bounded: a pending prescaler/reload write only completes in the watchdog's LSI domain. On
//this MM32 the DBGMCU IWDG_STOP bit freezes that domain while debug is enabled (from the first SWD
//attach to power off), not only while halted, so the vendor PVU/RVU_CheckStatus() loops hung forever
//after a soft start. The firmware does not set IWDG_STOP; tools/mm32flash.py sets it only while it holds
//the core halted. A sync takes a few LSI cycles (25 us each).
static void IwdgWait(u16 flag){
	uint32_t t = millis;
	while(IWDG_GetFlagStatus(flag) == SET && millis-t < 3);
}

void Iwdg_Init(u16 IWDG_Prescaler, u16 Reload){
	//Start the internal low-speed clock and wait for the clock to be ready
	RCC_LSICmd(ENABLE);
	uint32_t t = millis;
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET && millis-t < 10);

	//Setting Clock Pre-division Frequency
	IwdgWait(IWDG_FLAG_PVU);
	IWDG_WriteAccessCmd(0x5555);
	IWDG_SetPrescaler(IWDG_Prescaler);

	//Setting overload register values
	IwdgWait(IWDG_FLAG_RVU);
	IWDG_WriteAccessCmd(0x5555);
	IWDG_SetReload(Reload);

	//Loading and Enabling Counter
	IWDG_ReloadCounter();
	IWDG_Enable();
}
