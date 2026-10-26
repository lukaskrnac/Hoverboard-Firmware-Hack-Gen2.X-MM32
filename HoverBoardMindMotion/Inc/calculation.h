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
typedef struct { int32_t kp, ki, kv, d0, tref; } SpeedPlant;    //see calculation.c
void SpeedParams(uint8_t mode, SpeedPlant* p);
int SpeedCtrl(int sp, int meas, const SpeedPlant* p);
void SpeedCtrlReset(void);

#endif
