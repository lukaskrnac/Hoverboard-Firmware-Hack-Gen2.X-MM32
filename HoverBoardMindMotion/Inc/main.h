#ifndef MAIN_H
#define MAIN_H

#include "mcu.h"

// motor and bus state, defined in main.c
extern uint8_t step;            //6 step commutation position 1..6
extern uint32_t millis;         //ms since boot, SysTick
extern int32_t iOdom;           //hall steps, signed
extern int8_t dir;              //commanded direction 1 / -1
extern uint8_t sRxBuffer[10];   //uart rx dma buffer
extern int vbat;                //battery voltage 0.01V
extern int itotal;              //bus current 0.01A
extern int fvbat;               //filtered vbat
extern int fitotal;             //filtered itotal
extern uint8_t hallposprev;
extern int speed;               //setpoint from the remote
extern int pwm;
extern int realspeed;           //measured speed
extern int frealspeed;          //filtered realspeed
extern uint8_t wState;          //state byte from the remote
extern uint8_t uarten;          //uart in use: 1 = UART1, 2 = UART2
extern uint8_t halltimen;       //hall timer: 1 = TIM2, 0 = TIM3

#endif
