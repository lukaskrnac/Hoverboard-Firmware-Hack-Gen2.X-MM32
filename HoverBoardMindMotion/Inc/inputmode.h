#ifndef INPUTMODE_H
#define INPUTMODE_H
#include "mcu.h"

//ODrive input modes (inputmode.c): set point shaping between the command and the controllers,
//called from speedupdate(). Set points in hall steps and steps/s.
enum{ IM_PASSTHROUGH = 1, IM_VEL_RAMP = 2, IM_POS_FILTER = 3, IM_TRAP_TRAJ = 5 };

uint8_t InputModeSupported(uint32_t m);
uint8_t InputMode(void);                        //the config word; unsupported values: PASSTHROUGH
void InputTrack(int32_t steps, int32_t rpm);    //not controlling: the set points follow the wheel
int32_t InputVel(int32_t rpm, int32_t dtms, uint8_t holdDown, int32_t meas);    //velocity: rpm set point (VEL_RAMP); holdDown: no slowing
int32_t InputPos(int32_t target, int32_t ffRpm, int32_t dtms, int32_t* velFf);    //position: steps, ff rpm
uint8_t InputPosDone(void);                     //the filter / trajectory has arrived

extern float inPosSp, inVelSp;                  //position and velocity set points, steps, steps/s

#endif
