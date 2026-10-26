#ifndef BLDC_H
#define BLDC_H

#include "mcu.h"

void commutate();
uint8_t hallpos(uint8_t dir);
void speedupdate();
void TIMOCInit();

extern volatile uint16_t abspwm;
extern volatile uint32_t lastcommutate;
extern uint8_t lowbatcount;
extern uint8_t lowbatperm;
extern const uint8_t hall_to_pos[8];

enum{
	COM_VOLT=0,
	COM_SPEED=1,
	SINE_VOLT=2,
	SINE_SPEED=3,
	FOC_VOLT=4,
	FOC_SPEED=5,
	FOC_TORQUE=6,
};

#endif
