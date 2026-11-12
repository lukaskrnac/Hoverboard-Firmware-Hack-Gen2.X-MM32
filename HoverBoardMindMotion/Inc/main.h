#ifndef MAIN_H
#define MAIN_H

#include "mcu.h"

// motor and bus state, defined in main.c. volatile: written or read in interrupts (SysTick, ADC, uart rx dma)
extern volatile uint8_t step;            //6 step commutation position 1..6
extern volatile uint32_t millis;         //ms since boot, SysTick
extern volatile int32_t iOdom;           //hall steps, signed
extern volatile int8_t dir;              //commanded direction 1 / -1
extern volatile int vbat;                //battery voltage 0.01V
extern volatile int itotal;              //bus current 0.01A
extern volatile int fvbat;               //filtered vbat
extern volatile int fitotal;             //filtered itotal
extern volatile uint8_t powerOffReq;     //hoverboard.power_off (CANSimple): the main loop switches off
extern volatile int speed;               //setpoint from the remote
extern volatile int pwm;
extern volatile int realspeed;           //measured speed
extern volatile uint8_t wState;          //state byte from the remote
extern volatile uint8_t charging;        //charger connected
extern volatile uint8_t footPads;        //infrared foot sensors covered, bit 0 / 1 = ir1 / ir2
extern uint8_t uarten;          //uart in use: 1 = UART1, 2 = UART2
extern uint8_t halltimen;       //hall timer: 1 = TIM2, 0 = TIM3

#endif
