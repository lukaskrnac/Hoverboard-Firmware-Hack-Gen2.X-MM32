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
#ifdef TARGET_MM32SPIN25
#include "spin25-redefine.h"
#include "HAL_device.h"
#endif


//normal io
void io_init(void){
	//enable gpio clock
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
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
	if(BUZZERPIN<PINCOUNT){
		pinMode(BUZZERPIN, OUTPUT);
	}
	if(CHARGEPIN<PINCOUNT){
		static const uint8_t pull[] = { INPUT, INPUT_PULLUP, INPUT_PULLDOWN };
		uint32_t p = cfgOrDefault(CHARGE_PULL, 1);
		pinMode(CHARGEPIN, pull[p>=1&&p<=3 ? p-1 : 0]);
	}
}

void HALL_Init(void){
	pinMode(HALLAPIN, INPUT);
	pinMode(HALLBPIN, INPUT);
	pinMode(HALLCPIN, INPUT);
}

uint8_t HALLTIM_Init(u32 arr, u16 psc){    //hall sensor hardware speed sensing
	uint8_t tim2=1;    //no hall pin on a timer input: speed sensing will not work, but keep it deterministic
	for(uint8_t i=0;i<TIMCOUNT;i++){
		if(halltims[i].io==HALLAPIN||halltims[i].io==HALLBPIN||halltims[i].io==HALLCPIN){
			pinModeAF(halltims[i].io,halltims[i].af);
			tim2=halltims[i].tim2;
		}
	}
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
	TIM_ICInitTypeDef  TIM_ICInitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
	TIM_TimeBaseStruct.TIM_Period = arr;
	TIM_TimeBaseStruct.TIM_Prescaler = psc;
	//Setting Clock Segmentation
	//input filter clock 18 MHz: with the filter below a spike shorter than ~14 us is ignored
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	///TIM Upward Counting Mode
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(tim2 ? TIM2 : TIM3, &TIM_TimeBaseStruct);

	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_TRC;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	//fDTS/32, 8 samples (~14 us): a switching spike on a hall line made two captures within a
	//microsecond, the edge was measured as 0 us (speed 0 in the angle tracker: a freeze, then an
	//80 deg jump, a 20 A trip with field weakening at 600 rpm)
	TIM_ICInitStruct.TIM_ICFilter = 0xF;
	TIM_ICInit(tim2 ? TIM2 : TIM3, &TIM_ICInitStruct);
	TIM_ITConfig(tim2 ? TIM2 : TIM3, TIM_IT_CC1, ENABLE);

	TIM_SelectHallSensor(tim2 ? TIM2 : TIM3,ENABLE);
	TIM_UpdateRequestConfig(tim2 ? TIM2 : TIM3,TIM_UpdateSource_Regular);
	TIM_SelectInputTrigger(tim2 ? TIM2 : TIM3, TIM_TS_TI1F_ED);
	TIM_SelectSlaveMode(tim2 ? TIM2 : TIM3, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(tim2 ? TIM2 : TIM3, TIM_MasterSlaveMode_Enable);

	TIM_Cmd(tim2 ? TIM2 : TIM3, ENABLE);
	//NVIC_Configure(tim2 ? TIM2_IRQn : TIM3_IRQn, 1);
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

//timer1 automatic output
void TIM1_init(u16 arr, u16 psc){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef  TIM_BDTRInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM1, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	//Setting Clock Segmentation
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
	///TIM Upward Counting Mode
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCNPolarity = INVERT_LOWSIDE ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 1; 							//0% duty cycle
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_COM, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_BDTRStructInit(&TIM_BDTRInitStructure);
	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = AWDG ? TIM_BreakPolarity_High : TIM_BreakPolarity_Low ;
	TIM_BDTRInitStructure.TIM_DeadTime = 1;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
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
	TIM_Cmd(TIM1, ENABLE);
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
void UARTX_Init(u32 baudrate,uint8_t uart){
	UART_InitTypeDef UART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);
	//Baud rate
	UART_StructInit(&UART_InitStructure);
	UART_InitStructure.BaudRate = baudrate;
	UART_InitStructure.WordLength = UART_WordLength_8b;
	UART_InitStructure.StopBits = UART_StopBits_1;
	UART_InitStructure.Parity = UART_Parity_No;
	UART_InitStructure.HWFlowControl = UART_HWFlowControl_None;
	UART_InitStructure.Mode = UART_Mode_Rx | UART_Mode_Tx;
	UART_Init(uart==1 ? UART1 : UART2, &UART_InitStructure);
	UART_Cmd(uart==1 ? UART1 : UART2, ENABLE);
	UART_DMACmd(uart==1 ? UART1 : UART2, UART_DMAReq_EN, ENABLE);
}

uint8_t UART_GPIO_Init(void){
	uint8_t uart=1;
	for(uint8_t i=0;i<UARTCOUNT;i++){
		if(uarts[i].io==TXPIN||uarts[i].io==RXPIN){
			pinModeAF(uarts[i].io,uarts[i].af);
			if(uarts[i].tx){
				//open drain: several boards share one RemoteUartBus line, the master's rx must have a pull-up
				pinMode(uarts[i].io, OUTPUT_AF_OD);
			}else{
				pinMode(uarts[i].io, INPUT);
			}
			uart=uarts[i].uart;
		}
	}
	return uart;
}

void adc_Init(void){
	ADC_InitTypeDef  ADC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_PRESCALE = ADC_PCLK2_PRESCALE_6;							//for 72MHz
	ADC_InitStructure.ADC_Mode = ADC_Mode_Single_Period;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ExternalTrigConvCmd(ADC1,ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_VoltReference, 0, ADC_SampleTime_7_5Cycles);
	ADC_VrefintCmd(ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	if(VBATPIN<PINCOUNT){
		pinMode(VBATPIN, INPUT_ADC);
		for(uint8_t i=0;i<ADCCOUNT;i++){
			if(adcs[i].io==VBATPIN){
				ADC_RegularChannelConfig(ADC1, adcs[i].channel, 0, ADC_SampleTime_7_5Cycles);
				break;
			}
		}
	}
	if(ITOTALPIN<PINCOUNT){
		pinMode(ITOTALPIN, INPUT_ADC);
		for(uint8_t i=0;i<ADCCOUNT;i++){
			if(adcs[i].io==ITOTALPIN){
				ADC_RegularChannelConfig(ADC1, adcs[i].channel, 0, ADC_SampleTime_7_5Cycles);
				break;
			}
		}
	}
	if(IPHASEAPIN<PINCOUNT&&IPHASEBPIN<PINCOUNT){
		pinMode(IPHASEAPIN, INPUT_ADC);
		pinMode(IPHASEBPIN, INPUT_ADC);
		for(uint8_t i=0;i<ADCCOUNT;i++){
			if(adcs[i].io==IPHASEAPIN||adcs[i].io==IPHASEBPIN){
				ADC_RegularChannelConfig(ADC1, adcs[i].channel, 0, ADC_SampleTime_7_5Cycles);
			}
		}
	}

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
