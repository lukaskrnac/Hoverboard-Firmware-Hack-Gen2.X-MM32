#ifndef FOC_H
#define FOC_H
#include "mcu.h"

//Field oriented control on two measured phase currents (foc.c). Units: current mA, voltage mV.
//Signs in the motor frame: + = the direction of positive realspeed.

uint8_t FocAvailable(void);    //both phase current pins have an adc channel
uint8_t FocSensing(void);      //phase currents measured now (FOC, sine with the pins), offsets known
void FocInit(void);            //adc result registers of the current pins; at boot and on a mode change
void FocReset(void);           //controllers to zero (mode change, gates off)

//adc isr, FOC and sine commutation (sine: bus current, I_q and the trip from the phase currents)
void FocObserve(void);         //sine: I_d / I_q of this period, after HALLModuleCalc()
void FocMeasure(void);         //phase currents of this pwm period; sets itotal from them. Before the gates
uint8_t FocHoldOff(void);      //1: keep the gates off (no current offsets yet and the wheel turns, trip)
void FocControl(void);         //current loop and pwm, after HALLModuleCalc()

//main loop (speedupdate)
void FocSetCurrent(int32_t iq);      //torque: q current reference, mA
void FocSetVoltage(int32_t permille);    //q voltage, permille of the usable maximum, current limited
int32_t FocSpeedCtrl(int32_t sp, int32_t meas);    //rpm -> q current reference, mA
void FocSpeedReset(void);
void FocClearTrip(void);
int32_t FocIqSetpoint(void);         //mA
int32_t CurrentHardMax(void);        //mA, phase current trip

extern volatile int32_t focIdRef;     //d current reference, mA (field weakening, tests over SWD)
extern volatile int16_t focId, focIq; //measured, mA
extern volatile int16_t focIqF, focIdF;    //I_q, I_d averaged over ~7 ms, mA (telemetry)
extern volatile uint8_t focTrip;      //phase current above the trip level: gates off until the command is 0

#endif
