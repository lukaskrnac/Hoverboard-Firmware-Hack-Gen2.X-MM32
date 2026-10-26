#ifndef CALCULATION_H
#define CALCULATION_H

#include "mcu.h"

void PID_Init();
int PID(int setpoint,int real);
void avgspeed();
int PID2PWM(int pid);
void avgItotal();
void avgvbat();
uint32_t updateMotorRPM(uint32_t halltime);
void PIDrst();
int SpeedCtrl(int sp, int meas);
void SpeedCtrlReset(void);

#endif
