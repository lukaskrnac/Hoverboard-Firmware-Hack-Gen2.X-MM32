#ifndef HARDWARE_H
#define HARDWARE_H

#include "mcu.h"
#include "hal_conf.h"
#include "hal_gpio.h"
#include "hal_adc.h"



#define IOA 1
#define IOB 2
#define IOC 3
#define IOD 4


//pin table sizes of the selected MCU (hardware.c; mcu.h)
#if defined(MCU_SPIN25)
#define PINCOUNT 34
#define ADCCOUNT 12
#define TIMCOUNT 21
#define UARTCOUNT 9
#define TIMBKCOUNT 4
#define I2CCOUNT 12
#define BUZCOUNT 11
#elif defined(MCU_NO_UART2)
#define PINCOUNT 33
#define ADCCOUNT 10
#define TIMCOUNT 21
#define UARTCOUNT 4
#define TIMBKCOUNT 4
#define I2CCOUNT 12
#define BUZCOUNT 9
#else
#define PINCOUNT 33
#define ADCCOUNT 10
#define TIMCOUNT 21
#define UARTCOUNT 10
#define TIMBKCOUNT 4
#define I2CCOUNT 12
#define BUZCOUNT 9
#endif


#define OUTPUT GPIO_Mode_Out_PP
#define OUTPUT_AF GPIO_Mode_AF_PP
#define OUTPUT_AF_OD GPIO_Mode_AF_OD
#define OUTPUT_OD GPIO_Mode_Out_OD
#define INPUT GPIO_Mode_FLOATING
#define INPUT_PULLUP GPIO_Mode_IPU
#define INPUT_PULLDOWN GPIO_Mode_IPD
#define INPUT_ADC GPIO_Mode_AIN

enum{
	PA0=0,
	PA1=1,
	PA2=2,
	PA3=3,
	PA4=4,
	PA5=5,
	PA6=6,
	PA7=7,
	PA11=8,
	PA12=9,
	PA13=10,
	PA14=11,
	PA15=12,
	PB0=13,
	PB1=14,
	PB2=15,
	PB3=16,
	PB4=17,
	PB5=18,
	PB6=19,
	PB7=20,
	PB8=21,
	PB9=22,
	PB10=23,
	PB11=24,
	PB12=25,
	PC13=26,
	PC14=27,
	PC15=28,
	PD0=29,
	PD1=30,
	PD2=31,
	PD3=32,
	PD7=33,
};

typedef struct{
  uint8_t io;
  uint16_t channel;
}MM32ADC;
typedef struct{
  uint8_t io;
  uint8_t af;
  uint8_t tc;    //HT_TIM2 (else TIM3) | channel 0..2 (CH1..CH3): the hall XOR input needs CH1, CH2, CH3 of one timer
}MM32TIM23;
#define HT_TIM2 4
#define HT(tim2, ch) ((tim2)*HT_TIM2+(ch)-1)
typedef struct{
  uint8_t io;
  uint8_t af;
  uint8_t tx;
  uint8_t uart;
}MM32UART;
typedef struct{
  uint8_t io;
  uint16_t af;
}MM32TIMBK;
typedef struct{    //I2C1 pins (the IMU)
  uint8_t io;
  uint8_t af;
  uint8_t scl;    //1 SCL, 0 SDA
}MM32I2C;
typedef struct{    //timer outputs for the passive buzzer's square wave: TIM14 / TIM16 / TIM17 CH1 or CH1N
  uint8_t io;
  uint8_t af;
  uint8_t tim;    //BZ_TIM14 / BZ_TIM16 / BZ_TIM17, + BZ_N for the CH1N output
}MM32BUZ;
enum{ BZ_TIM14, BZ_TIM16, BZ_TIM17, BZ_N = 4 };


//pin capability tables for the selected MCU, hardware.c
extern const MM32ADC adcs[ADCCOUNT];
extern const MM32TIM23 halltims[TIMCOUNT];
extern const MM32UART uarts[UARTCOUNT];
extern const MM32TIMBK ocps[TIMBKCOUNT];
extern const MM32I2C i2cs[I2CCOUNT];
extern const MM32BUZ buzs[BUZCOUNT];

//io: a config word (16 bit: 0x0100 is no pin, not PA0)
void pinMode(uint16_t io,uint8_t state);
uint8_t digitalRead(uint16_t io);
uint16_t analogRead(uint16_t io);
void digitalWrite(uint16_t io, uint8_t state);
void pinModeAF(uint16_t io,uint8_t af);
uint8_t PinPortBit(uint8_t io);    //port (0 = A) << 4 | pin number, 0xFF for io >= PINCOUNT
void gpioMode(GPIO_TypeDef* gpio, uint8_t pin, uint8_t mode);    //by port and pin number (pins outside the table)
void gpioAF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af);




#endif
