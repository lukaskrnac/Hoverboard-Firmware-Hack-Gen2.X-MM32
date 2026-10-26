#ifndef BLDC_H
#define BLDC_H

#include "mcu.h"

void commutate(void);
uint8_t hallpos(uint8_t forward);    //1: raw hall state, else complemented (7-raw)
void speedupdate(void);
void SetDriveMode(uint8_t mode);    //safe while running, see bldc.c
void gatesOutput(uint8_t on);

extern volatile uint8_t gatesoff;    //1: gates must stay off (disarmed by the remote, low battery), applied by the adc isr
extern volatile uint8_t coastreq;    //1: gates off for now, the drive mode's behaviour at command 0 (coast)
extern volatile uint32_t lastcommutate;
extern uint8_t lowbatcount;
extern uint8_t lowbatperm;
extern const uint8_t hall_to_pos[8];

//drive mode = commutation*4 + control. Control 0..3 is the Gen2.x-GD32 iDrivingMode (pwm, speed, torque,
//position): a remote written for that firmware gets block commutation with the same meaning.
enum{ COMMUT_BLOCK=0, COMMUT_SINE=1, COMMUT_FOC=2 };
enum{ CTRL_PWM=0, CTRL_SPEED=1, CTRL_TORQUE=2, CTRL_POSITION=3 };
#define COMMUTATION(mode) ((mode)>>2)
#define CONTROL(mode)     ((mode)&3)
enum{
	COM_VOLT=0, COM_SPEED=1, COM_TORQUE=2, COM_POSITION=3,
	SINE_VOLT=4, SINE_SPEED=5, SINE_TORQUE=6, SINE_POSITION=7,
	FOC_VOLT=8, FOC_SPEED=9, FOC_TORQUE=10, FOC_POSITION=11,
};
uint8_t DriveModeSupported(uint8_t mode);    //implemented, and the board has what it needs

#endif
