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
volatile uint8_t powerOffReq;    //CANSimple hoverboard.power_off: the power button's shutdown

//the power button: a digital pin (1 = pressed), or with buttonadc an adc pin above that count (a board whose
//press only raises a voltage, the F031 board: the switched supply 19.5 -> 24.6 V on PA6)
static uint8_t buttonDown(void){
	if(BUTTONPIN>=PINCOUNT) return 0;
	return BUTTON_ANALOG ? analogRead(BUTTONPIN)>BUTTON_ADC : digitalRead(BUTTONPIN);
}
volatile int speed=0;
volatile int pwm=0;
volatile int realspeed=0;
volatile uint8_t  wState = STATE_LED_BATTLEVEL;    //battery LEDs until the remote sends a state
static uint8_t flicker=0;
volatile uint8_t charging;    //charger connected (CHARGEPIN, debounced)
volatile uint8_t footPads;    //infrared foot sensors covered: bit k = IR_PIN(k) (debounced)
static uint8_t chargeRaw;
static uint32_t chargeT;
uint8_t uarten=1;
uint8_t halltimen=1;
uint8_t pwmDiv=1;    //pwm periods per control period (pinout.h)
//the board configuration (pinout.h), read from flash at boot (sim_eeprom.c). Many boards of one type: one
//image with firmware and config (the web console's image.html), no special build.
uint16_t pinstorage[128];    //zero: a valid record replaces words 0-63 (else no boot); 64-127 set in main()


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

//buzzer on for ms (then off): BuzzerSet() (initialize.c: an active buzzer on, a passive one gets the timer's
//2.2 kHz)
static void Beep(uint32_t ms){
	BuzzerSet(1);
	DelayFed(ms);
	BuzzerSet(0);
}

static void setRGB(uint8_t r, uint8_t g, uint8_t b){    //RGB LED (unconfigured pins are ignored)
	digitalWrite(LEDRPIN, r);
	digitalWrite(LEDGPIN, g);
	digitalWrite(LEDBPIN, b);
}

//no valid config (hall / uart pins unknown): do not boot, nothing driven (PinFinder writes one). The fault
//shows on whatever the board has: every pin (not SWD; the bridge pins are not in the pin list) pulled up for
//0.2 s, down for 0.8 s; LEDs and an active buzzer behind a transistor follow a pull-up (flash / beep). A
//known power latch stays pulled up (the pull-down switched the board off; the pull-up holds it); with no config at all it stays on only
//while the power button is held.
static void __attribute__((noinline, noreturn)) noConfig(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOA|RCC_AHBENR_GPIOB|RCC_AHBENR_GPIOC|RCC_AHBENR_GPIOD;
	for(uint8_t on=1;;on^=1){
		for(uint8_t i=0;i<PINCOUNT;i++) if(i!=PA13&&i!=PA14) pinMode(i, on||i==LATCHPIN ? INPUT_PULLUP : INPUT_PULLDOWN);
		DELAY_Ms(on ? 200 : 800);
	}
}

s32 main(void){
	DELAY_Init();
	//the extension words unset unless the config has an extension record (a 256 byte initialized table cost
	//its whole size in flash; only these defaults ever count)
	for(uint32_t i=64;i<128;i++) pinstorage[i] = 0xFFFF;
	pinstorage[EXT_MAGIC_POS] = EXT_MAGIC;
	if(!restorecfg()) noConfig();

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
		while(buttonDown()){    //wait while release button
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
			if(BAT_SET(BAT_FULL)&&BAT_SET(BAT_EMPTY)&&BAT_FULL>BAT_EMPTY){
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
			static uint8_t alarm;
			uint8_t on = (lowbatcount>=10||lowbatperm)&&flicker;
			if(on!=alarm){ alarm = on; BuzzerSet(on); }
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
		if(LATCHPIN<PINCOUNT){
			//a press switches off only after the button was released for 300 ms since power-on (the press that
			//switched the board on is still held while it boots; an analog button reads 0 for the adc's first
			//few ms, far below that), and counts after 100 ms held: an analog button's switched supply decays
			//slowly through the threshold after the release, its noise read as a new press
			static uint8_t btnArmed;
			static uint32_t tDown, tUp;    //last seen pressed / released
			if(buttonDown()) tDown = millis; else tUp = millis;
			if(millis-tDown>300) btnArmed = 1;
			if(powerOffReq||(btnArmed&&millis-tUp>100)){    //button press (or hoverboard.power_off) for shutdown
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
				while(buttonDown()) {    //wait for release
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
