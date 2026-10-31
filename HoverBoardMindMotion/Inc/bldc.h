#ifndef BLDC_H
#define BLDC_H

#include "mcu.h"

void commutate(void);
uint8_t hallpos(uint8_t forward);    //1: raw hall state, else complemented (7-raw)
void speedupdate(void);
void SetDriveMode(uint8_t mode);    //safe while running, see bldc.c
void gatesOutput(uint8_t on);
void BlockBrake(uint8_t on);    //block commutation short brake (speedupdate)
extern volatile uint8_t blockbrake;
extern volatile uint8_t regenShort;    //RegenShort() drives the outputs (sine / block)
uint8_t RegenShort(uint8_t over);    //adc isr: bus above BAT_FULL + 0.5 V while turning (sine / block)

extern volatile uint8_t gatesoff;    //1: gates must stay off (disarmed by the remote, low battery), applied by the adc isr
extern volatile uint8_t coastreq;    //1: gates off for now, the drive mode's behaviour at command 0 (coast)
extern volatile uint32_t lastcommutate;
extern uint8_t lowbatcount;
extern uint8_t lowbatperm;
extern volatile int16_t derate;    //1/1024 of the current limit / max duty left by the temperature (TEMP_DERATE)
void ClearErrors(void);    //Clear_Errors: disarm reason, FOC trip, low battery once above empty again
extern const uint8_t hall_to_pos[8];

//CANSimple (ODrive) axis state and error bits, cansimple.c
#define AXIS_IDLE 1
#define AXIS_CLOSED_LOOP 8
#define ERR_DRV_FAULT                0x20u     //hardware current limit (adc watchdog / break input) repeating
#define ERR_DC_BUS_OVER_VOLTAGE      0x100u
#define ERR_DC_BUS_UNDER_VOLTAGE     0x200u
#define ERR_DC_BUS_OVER_CURRENT      0x400u
#define ERR_CURRENT_LIMIT_VIOLATION  0x1000u
#define ERR_DC_BUS_OVER_REGEN_CURRENT 0x800u
#define ERR_INVERTER_OVER_TEMP       0x4000u
#define ERR_VELOCITY_LIMIT_VIOLATION 0x8000u
#define ERR_WATCHDOG_TIMER_EXPIRED   0x1000000u
#define ERR_ESTOP_REQUESTED          0x2000000u
#define ERR_SPINOUT_DETECTED         0x4000000u
#define POS_FINE 256    //position set points and estimate: 1/256 hall step (90 steps per turn: 0.016 deg)
int32_t PosGain(void);    //position gain, 0.1 rpm per hall step (POS_KP or the drive mode's default)
extern volatile int32_t posCmd;    //CANSimple position target, 1/POS_FINE hall step
extern volatile int32_t posVelFf;    //Set_Input_Pos Vel_FF, rpm
extern volatile uint8_t cmdSource;
extern volatile uint8_t axisState;
extern volatile uint32_t axisErrors;
extern volatile uint32_t axisDisarm;
extern volatile uint8_t posDone;

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
