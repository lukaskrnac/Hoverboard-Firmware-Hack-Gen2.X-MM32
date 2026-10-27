#ifndef CALCULATION_H
#define CALCULATION_H

#include "mcu.h"

void avgItotal(void);
void avgvbat(void);
typedef struct { int32_t kp, ki, kd, kv, d0, tref, umax; } SpeedPlant;    //see calculation.c
typedef struct { int32_t last, acc; } SpeedDeriv;    //rpm, rpm/s
int32_t SpeedDerivStep(SpeedDeriv* f, int32_t meas, int32_t dt);    //filtered d(meas)/dt, rpm/s
void SpeedParams(uint8_t mode, SpeedPlant* p);
int SpeedCtrl(int sp, int meas, const SpeedPlant* p);
void SpeedCtrlReset(void);
void TorqueCtrlReset(void);
int TorqueCtrlBlock(int32_t iref, int32_t imeas, int32_t rpm);    //COM_TORQUE: mA, mA, rpm -> permille duty (>= 0)

#endif
