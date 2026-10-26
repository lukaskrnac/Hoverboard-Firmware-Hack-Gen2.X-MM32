#include "../Inc/mcu.h"
#include "../Inc/main.h"
#include "../Inc/interrupt.h"
#include "hal_gpio.h"
#include "hal_rcc.h"
#include "hal_adc.h"           
#include "hal_tim.h"
#include "hal_iwdg.h"
#include "../Inc/delay.h"
#include "../Inc/pinout.h"  
#include "../Inc/initialize.h"
#include "../Inc/uart.h"
#include "../Inc/bldc.h"
#include "../Inc/remoteUartBus.h"
#include "../Inc/sim_eeprom.h"
#include "../Inc/hardware.h"
#include "../Inc/calculation.h"

#include "../Inc/ipark.h"
#include "../Inc/FOC_Math.h"
#include "../Inc/pwm_gen.h"
#include "../Inc/PID.h"
#include "../Inc/hallhandle.h"

volatile uint8_t step=1;//very importatnt to set to 1 or it will not work
volatile uint32_t millis;
static uint32_t lastupdate;
static uint32_t lastflicker;
volatile int32_t iOdom;
volatile int8_t dir=1;
uint8_t sRxBuffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile int vbat;
volatile int itotal;
volatile int fvbat;
volatile int fitotal;
uint8_t hallposprev=1;
volatile int speed=0;
volatile int pwm=0;
volatile int realspeed=0;
volatile int frealspeed=0;
volatile uint8_t  wState = STATE_LED_BATTLEVEL;    //battery LEDs until the remote sends a state
static uint8_t flicker=0;
uint8_t uarten=1;
uint8_t halltimen=1;
////////////////////////////////////////////////////////////////////////////////////////////
//  compile device specefic firmware for mass produce
//  change EEPROMEN to 0
#define EEPROMEN 1
//  copy pinstorage initializer from autodetect and replace the default line below
uint16_t pinstorage[64]={0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDCAB, 31, 250, 0, 19200, 8192, 1, 30, 0, 10, 300, 1, 1, 42000, 32000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
////////////////////////////////////////////////////////////////////////////////////////////


//every wait after the watchdog started: in steps well below its timeout (a 2 x 150 ms melody note
//between two feeds was enough for a watchdog reset; with the button still held that rebooted the board
//instead of switching it off)
static void DelayFed(uint32_t ms){
	while(ms){
		uint32_t chunk = ms > 50 ? 50 : ms;
		DELAY_Ms(chunk);
		IWDG_ReloadCounter();
		ms -= chunk;
	}
}

s32 main(void){
	DELAY_Init();
	if(!restorecfg()&&EEPROMEN){    //if data in eeprom is not valid, do not boot up
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOC, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
		while(1){
			for(uint8_t i=0;i<33;i++){
				if(i!=10&&i!=11){
					pinMode(i, INPUT_PULLUP);
				}
			}	
			DELAY_Ms(1000);
			for(uint8_t i=0;i<33;i++){
				if(i!=10&&i!=11){
					pinMode(i, INPUT_PULLDOWN);
				}
			}	
			DELAY_Ms(1000);
		}
	}
	
	//initialize normal gpio
	io_init();
	//hall gpio init
	HALL_Init();
	//hall timer init
	halltimen=HALLTIM_Init(65535, SystemCoreClock/1000000-1);    //1 us ticks (the timer divides by psc+1)
	//initialize 6 bldc pins
	BLDC_init();
	//initialize timer
	TIM1_init(PWM_RES, 0);
	//systick config
	//timer1 commutation interrupt config
	NVIC_Configure(TIM1_BRK_UP_TRG_COM_IRQn, 1);
	//vbat
	adc_Init();
	//adc interrupt
	NVIC_Configure(ADC_COMP_IRQn, 0);
	#ifdef WATCHDOG
	Iwdg_Init(IWDG_Prescaler_32, 625);    //LSI 40 kHz / 32 / 625 = 500 ms (LSI tolerance: >= ~330 ms); waits below use DelayFed()
	#endif
	//serial1.begin(19200);
	uarten=UART_GPIO_Init();
	UARTX_Init((uint32_t)BAUD,uarten);
	//uart interrupt
	//uart dma
	if(uarten==1){
		DMA_NVIC_Config(DMA1_Channel3, (u32)&UART1->RDR, (u32)sRxBuffer, 1);
		NVIC_Configure(DMA1_Channel2_3_IRQn, 1);
	}else{
		DMA_NVIC_Config(DMA1_Channel5, (u32)&UART2->RDR, (u32)sRxBuffer, 1);
		NVIC_Configure(DMA1_Channel4_5_IRQn, 1);
	}
	//latch on power
	if(LATCHPIN<PINCOUNT){    //have latch
		DelayFed(100);    //some board the micro controller can reset in time and turn back on
		digitalWrite(LATCHPIN, 1);
		while(digitalRead(BUTTONPIN)&&BUTTONPIN<PINCOUNT){    //wait while release button
			if(BUZZERPIN<PINCOUNT){    //have buzzer
				digitalWrite(BUZZERPIN, 1);
				DelayFed(2);
				digitalWrite(BUZZERPIN, 0);
				DelayFed(2);
			}else{
				__NOP();
				__NOP();
			}
			IWDG_ReloadCounter();
		}	
	}
	if(BUZZERPIN<PINCOUNT){
		for(int i=0;i<5;i++){    //power on melody
			digitalWrite(BUZZERPIN, 1);
			DelayFed(50);
			digitalWrite(BUZZERPIN, 0);
			DelayFed(50);
		}
	}
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);    //Software start conversion
	UART_SendString("hello World\n\r");    //debug uart
	
	PWM_GEN_init(&pwm_gen);
	InitNormalization(300,4000,4000,&RP);
	HALLModuleInit(&HALL1);
	InitPI();
	MovingAvgInit(&IDData);
	MovingAvgInit(&IQData);
	MovingAvgInit(&SpeedFdk);
	PID_Init();
  while(1) {
		uint8_t state = wState;
		digitalWrite(LEDUPIN, (state&STATE_LED_UP)!=0);    //unconfigured pins are ignored
		digitalWrite(LEDDPIN, (state&STATE_LED_DOWN)!=0);
		if(!(state&STATE_LED_BATTLEVEL)){    //the remote sets the LEDs
			digitalWrite(LEDGPIN, (state&STATE_LED_GREEN)!=0);
			digitalWrite(LEDBPIN, (state&STATE_LED_ORANGE)!=0);
			digitalWrite(LEDRPIN, (state&STATE_LED_RED)!=0);
		}else if(LEDRPIN<PINCOUNT&&LEDGPIN<PINCOUNT&&LEDBPIN<PINCOUNT){
			int32_t bat_range = (int32_t)BAT_FULL-BAT_EMPTY;
			int32_t bat_10 = BAT_EMPTY+bat_range*10/100;
			int32_t bat_20 = BAT_EMPTY+bat_range*20/100;
			int32_t bat_50 = BAT_EMPTY+bat_range*50/100;
			int32_t bat_60 = BAT_EMPTY+bat_range*60/100;
			int32_t bat_70 = BAT_EMPTY+bat_range*70/100;
			if(BAT_FULL>20000&&BAT_FULL<65000&&BAT_EMPTY>20000&&BAT_EMPTY<65000&&BAT_FULL>BAT_EMPTY){
				if(vbat*10>bat_70){
					digitalWrite(LEDRPIN,0);
					digitalWrite(LEDGPIN,1);
					digitalWrite(LEDBPIN,0);
				}else if(vbat*10>bat_60){
					digitalWrite(LEDRPIN,0);
					digitalWrite(LEDGPIN,1);
					digitalWrite(LEDBPIN,1);
				}else if(vbat*10>bat_50){
					digitalWrite(LEDRPIN,0);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,1);	
				}else if(vbat*10>bat_20){
					digitalWrite(LEDRPIN,1);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,1);
				}else if(vbat*10>bat_10){
					digitalWrite(LEDRPIN,1);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,0);
				}else{
					digitalWrite(LEDRPIN,flicker);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,0);
				}
			}else{
				digitalWrite(LEDRPIN,digitalRead(HALLAPIN));
				digitalWrite(LEDGPIN,digitalRead(HALLBPIN));
				digitalWrite(LEDBPIN,digitalRead(HALLCPIN));
			}
		}
		if(BUZZERPIN<PINCOUNT){
			if(lowbatcount>=10||lowbatperm){
				digitalWrite(BUZZERPIN,flicker);
			}else{
				digitalWrite(BUZZERPIN,0);
			}
		}
		if(millis-lastupdate>1){//speed pid loop
			speedupdate();
		  lastupdate=millis;
		}
		if(millis-lastflicker>700){//speed pid loop
			flicker=!flicker;
		  lastflicker=millis;
		}
		if(LATCHPIN<PINCOUNT&&BUTTONPIN<PINCOUNT){
			if(digitalRead(BUTTONPIN)){    //button press for shutdown
				//gates off for good: speedupdate() is not called again, so gatesoff stays set and the adc
				//isr keeps them off. Zeroing CCR1-3 did not stop the motor: the isr rewrites them in sine
				//mode, and the gates stayed armed. Switched here too in case the isr is the part that hangs.
				gatesoff = 1;
				pwm = 0;
				gatesOutput(0);
				if(BUZZERPIN<PINCOUNT){
					for(int i=0;i<3;i++){    //power off melody
						digitalWrite(BUZZERPIN, 1);
						DelayFed(150);
						digitalWrite(BUZZERPIN, 0);
						DelayFed(150);
					}
				}
				while(digitalRead(BUTTONPIN)) {    //wait for release
					__NOP();
					IWDG_ReloadCounter();
				}
				digitalWrite(LATCHPIN, 0);    //last line to ever be executed
				while(1){    //still powered (latch failed, debugger supply): stay off. A watchdog reset
					IWDG_ReloadCounter();    //would boot and set the latch again
				}
			}
		}
		IWDG_ReloadCounter();    //feed the watchdog
  }//main loop	
}

