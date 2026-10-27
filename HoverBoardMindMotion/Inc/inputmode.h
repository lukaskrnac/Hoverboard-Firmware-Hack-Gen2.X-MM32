#ifndef INPUTMODE_H
#define INPUTMODE_H
#include "mcu.h"

//ODrive input modes (inputmode.c): set point shaping between the command and the controllers,
//called from speedupdate(). Set points in hall steps and steps/s.
enum{ IM_PASSTHROUGH = 1, IM_VEL_RAMP = 2, IM_POS_FILTER = 3, IM_TRAP_TRAJ = 5, IM_TORQUE_RAMP = 6 };

uint8_t InputModeSupported(uint32_t m);
int32_t InputTorque(int32_t cmd, int32_t dtms, uint8_t held);    //torque: 0.01 A, TORQUE_RAMP
float InputAccel(void);    //the position set point's acceleration, turns/s^2 (TRAP_TRAJ, POS_FILTER)
uint8_t InputMode(void);                        //the config word; unsupported values: PASSTHROUGH
void InputTrack(int32_t pos, int32_t rpm);    //pos in 1/POS_FINE step;    //not controlling: the set points follow the wheel
int32_t InputVel(int32_t rpm, int32_t dtms, uint8_t holdDown, int32_t meas);    //velocity: rpm set point (VEL_RAMP); holdDown: no slowing
int32_t InputPos(int32_t target, int32_t ffRpm, int32_t dtms, int32_t* velFf);    //position: 1/POS_FINE step, ff rpm
uint8_t InputPosDone(void);                     //the filter / trajectory has arrived

extern float inPosSp, inVelSp;                  //position and velocity set points, 1/POS_FINE step (/s)

#endif
