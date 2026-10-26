#include "mcu.h"
#include "main.h"
#include "interrupt.h"
#include "hal_gpio.h"
#include "hal_rcc.h"
#include "hal_adc.h"
#include "hal_tim.h"
#include "hal_iwdg.h"
#include "delay.h"
#include "pinout.h"
#include "initialize.h"
#include "uart.h"
#include "bldc.h"
#include "remoteUartBus.h"
#include "sim_eeprom.h"
#include "hardware.h"
#include "calculation.h"

#include "pwm_gen.h"
#include "hallhandle.h"
#include "foc.h"

volatile uint8_t step=1;//very importatnt to set to 1 or it will not work
volatile uint32_t millis;
static uint32_t lastupdate;
static uint32_t lastflicker;
volatile int32_t iOdom;
volatile int8_t dir=1;
volatile int vbat;
volatile int itotal;
volatile int fvbat;
volatile int fitotal;
volatile int speed=0;
volatile int pwm=0;
volatile int realspeed=0;
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

	if(CFG_VERSION<1){    //old drive mode numbers; saved with the next config frame
		if(DRIVEMODE==2) DRIVEMODE=SINE_VOLT;
		else if(DRIVEMODE==3) DRIVEMODE=SINE_SPEED;
		CFG_VERSION=1;
	}
	if(CFG_VERSION<2){    //baud was stored in baud, now in 100 baud (115200 did not fit the 16 bit word)
		BAUD = BAUD>=1200 ? BAUD/100 : 1152;
		CFG_VERSION=2;
	}
	if(CFG_VERSION<3){    //input mode words: PinFinder's "reserved" (35, 43 held 0 / 1), now default
		INPUT_MODE = VEL_RAMP_RATE = INPUT_FILTER_BW = TRAJ_VEL = TRAJ_ACCEL = TRAJ_DECEL = 0xFFFF;
		CFG_VERSION=3;
	}
	if(!DriveModeSupported(DRIVEMODE)) DRIVEMODE=COM_VOLT;

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
	UARTX_Init((uint32_t)BAUD*100,uarten);    //BAUD in units of 100 (config version 2)
	SerialInit();    //rx ring and tx by DMA, parsed in the main loop (RemotePoll)
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
	HALLModuleInit(&HALL1);
	FocInit();
  while(1) {
		RemotePoll();    //received frames and replies, as soon as they arrive
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
