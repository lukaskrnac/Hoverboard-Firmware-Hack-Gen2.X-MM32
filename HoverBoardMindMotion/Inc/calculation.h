#ifndef CALCULATION_H
#define CALCULATION_H

#include "mcu.h"

void PID_Init(void);
int PID(int setpoint,int real);
void avgspeed(void);
int PID2PWM(int pid);
void avgItotal(void);
void avgvbat(void);
uint32_t updateMotorRPM(uint32_t halltime);
void PIDrst(void);
typedef struct { int32_t kp, ki, kv, d0, tref; } SpeedPlant;    //see calculation.c
void SpeedParams(uint8_t mode, SpeedPlant* p);
int SpeedCtrl(int sp, int meas, const SpeedPlant* p);
void SpeedCtrlReset(void);

#endif
