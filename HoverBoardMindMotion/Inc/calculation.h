#ifndef CALCULATION_H
#define CALCULATION_H

#include "mcu.h"

void avgItotal(void);
void avgvbat(void);
typedef struct { int32_t kp, ki, kv, d0, tref, umax; } SpeedPlant;    //see calculation.c
void SpeedParams(uint8_t mode, SpeedPlant* p);
int SpeedCtrl(int sp, int meas, const SpeedPlant* p);
void SpeedCtrlReset(void);
void TorqueCtrlReset(void);
int TorqueCtrlBlock(int32_t iref, int32_t imeas, int32_t rpm);    //COM_TORQUE: mA, mA, rpm -> permille duty (>= 0)

#endif
