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
#include "imu.h"

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
volatile uint8_t buzzTone;    //passive buzzer: the adc isr makes the tone
volatile uint8_t charging;    //charger connected (CHARGEPIN, debounced)
volatile uint8_t footPads;    //infrared foot sensors covered: bit k = IR_PIN(k) (debounced)
static uint8_t chargeRaw;
static uint32_t chargeT;
uint8_t uarten=1;
uint8_t halltimen=1;
uint8_t pwmDiv=1;    //pwm periods per control period (pinout.h)
////////////////////////////////////////////////////////////////////////////////////////////
//  compile device specefic firmware for mass produce
//  change EEPROMEN to 0
#define EEPROMEN 1
//  copy pinstorage initializer from autodetect and replace the default line below
uint16_t pinstorage[128]=PINSTORAGE_DEFAULTS;
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

//buzzer on for ms (then off). An active buzzer (own oscillator) is switched on; a passive one (BUZZER_PASSIVE)
//gets a square wave, here by busy waiting (the adc isr is not running yet at the melodies)
static void Beep(uint32_t ms){
	if(cfgBool(BUZZER_PASSIVE, 0)){
		uint32_t load = SysTick->LOAD+1, last = SysTick->VAL, half = SystemCoreClock/4400, n = half;
		uint8_t lvl = 0;
		for(uint32_t t=millis;millis-t<ms;){    //2.2 kHz: toggles every SystemCoreClock/4400 SysTick counts
			uint32_t v = SysTick->VAL, d = last>=v ? last-v : last+load-v;
			last = v;
			if(d>=n){ lvl = !lvl; digitalWrite(BUZZERPIN, lvl); n = half; } else n -= d;
			IWDG_ReloadCounter();
		}
	}else{
		digitalWrite(BUZZERPIN, 1);
		DelayFed(ms);
	}
	digitalWrite(BUZZERPIN, 0);
}

static void setRGB(uint8_t r, uint8_t g, uint8_t b){    //RGB LED (unconfigured pins are ignored)
	digitalWrite(LEDRPIN, r);
	digitalWrite(LEDGPIN, g);
	digitalWrite(LEDBPIN, b);
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

	ConfigMigrate();
	if(!DriveModeSupported(DRIVEMODE)) DRIVEMODE=COM_VOLT;
	//pwm period 2048..16383 (17.6..2.2 kHz). The control (the adc isr: 53-73 us in sine, 88-110 in FOC) runs
	//at ~4.39 kHz whatever the pwm: every pwmDiv-th period below 8192 (2: 8.8 kHz, 3: 13.2, 4: 17.6). It ran
	//every period, and from ~8.8 kHz the isr overran its period: the sine angle stood still while the field
	//kept driving, up to ~100 W into a standing motor. The phase current modulation limit follows the
	//shorter low side time (foc.c). The RAM value only takes effect here (the code uses the timer's ARR).
	if(PWM_RES<2048||PWM_RES>16383) PWM_RES = 8192;
	pwmDiv = PWM_RES>=8192 ? 1 : (uint8_t)((8192+PWM_RES/2)/PWM_RES);

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
	//the adc on TRGO = the update event, every pwmDiv-th period at the counter's zero (the centre of the low
	//side time; the repetition counter counts half periods). The compare registers reload there too, so a
	//control step's outputs hold for its whole period. RCR is preloaded: from the next update on.
	TIM1->RCR = (uint32_t)(2*pwmDiv-1);
	TIM1->CR2 = (TIM1->CR2&~TIM_CR2_MMS)|TIM_CR2_MMS_UPDATE;
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
				Beep(2);
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
			Beep(50);
			DelayFed(50);
		}
	}
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);    //Software start conversion
	UART_SendString("hello World\n\r");    //debug uart

	PWM_GEN_init(&pwm_gen);
	HALLModuleInit(&HALL1);
	FocInit();
	ImuInit();    //configured IMU pins: the chip set up, read in the main loop
  while(1) {
		RemotePoll();    //received frames and replies, as soon as they arrive
		ImuPoll();
		uint8_t state = wState;
		digitalWrite(LEDUPIN, (state&STATE_LED_UP)!=0);    //unconfigured pins are ignored
		digitalWrite(LEDDPIN, (state&STATE_LED_DOWN)!=0);
		if(!(state&STATE_LED_BATTLEVEL)){    //the remote sets the LEDs
			setRGB((state&STATE_LED_RED)!=0, (state&STATE_LED_GREEN)!=0, (state&STATE_LED_ORANGE)!=0);
		}else if(LEDRPIN<PINCOUNT&&LEDGPIN<PINCOUNT&&LEDBPIN<PINCOUNT){
			if(BAT_FULL>20000&&BAT_FULL<65000&&BAT_EMPTY>20000&&BAT_EMPTY<65000&&BAT_FULL>BAT_EMPTY){
				//battery level: above 70 % of the range green, 60 % green + orange (the LEDB pin), 50 % orange,
				//20 % red + orange, 10 % red, below that red flashing
				static const uint8_t level[5][2] = { {70, 2}, {60, 3}, {50, 1}, {20, 5}, {10, 4} };    //percent, rgb bits
				int32_t bat_range = (int32_t)BAT_FULL-BAT_EMPTY;
				uint8_t rgb = flicker ? 4 : 0;
				for(uint8_t k=0;k<5;k++){
					if(vbat*10>(int32_t)BAT_EMPTY+bat_range*level[k][0]/100){ rgb = level[k][1]; break; }
				}
				setRGB(rgb&4, rgb&2, rgb&1);
			}else{
				setRGB(digitalRead(HALLAPIN), digitalRead(HALLBPIN), digitalRead(HALLCPIN));
			}
		}
		if(BUZZERPIN<PINCOUNT){    //low battery alarm
			uint8_t on = (lowbatcount>=10||lowbatperm)&&flicker;
			if(cfgBool(BUZZER_PASSIVE, 0)) buzzTone = on;    //the adc isr toggles the pin
			else digitalWrite(BUZZERPIN, on);
		}
		if(CHARGEPIN<PINCOUNT){    //charger connected, 50 ms debounce
			uint8_t c = digitalRead(CHARGEPIN)==(cfgOrDefault(CHARGE_LEVEL, 2)==2);
			if(c!=chargeRaw){ chargeRaw = c; chargeT = millis; }
			else if(millis-chargeT>50) charging = c;
		}
		for(uint8_t k=0;k<2;k++){    //infrared foot sensors, 50 ms debounce like the charger
			static uint8_t footRaw;
			static uint32_t footT[2];
			if(IR_PIN(k)>=PINCOUNT) continue;
			uint8_t c = digitalRead(IR_PIN(k))==((cfgOrDefault(IR_MODE, 0)>>(4*k)&3)!=1);    //level 1 low, else high
			if(c!=(footRaw>>k&1)){ footRaw ^= (uint8_t)(1<<k); footT[k] = millis; }
			else if(millis-footT[k]>50) footPads = (uint8_t)((footPads&~(1<<k))|c<<k);
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
						Beep(150);
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
