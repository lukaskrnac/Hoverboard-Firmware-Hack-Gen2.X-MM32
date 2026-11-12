#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "mcu.h"

extern volatile uint8_t poweron;     //counts adc conversions until the current offsets are calibrated (>127 = done)
extern volatile uint8_t overcurrent;    //soft current limit active
extern volatile uint8_t hwOcFault;    //the break input (hardware current limit) broke repeatedly, until speedupdate() reads it
extern volatile uint8_t realdir;     //measured direction from the hall sequence

//interrupt handlers (vector table in Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.c)
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void ADC1_COMP_IRQHandler(void);
int32_t McuTemp(void);    //MCU internal temperature sensor, 0.1 C
void BusSampleCheck(void);    //SysTick: restores the adc trigger if a sine bus current sample never came

#endif
