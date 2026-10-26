#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "mcu.h"

extern volatile uint8_t poweron;     //counts adc conversions until the current offsets are calibrated (>127 = done)
extern volatile uint8_t realdir;     //measured direction from the hall sequence

//interrupt handlers (vector table in Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.c)
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void DMA1_Channel2_3_IRQHandler(void);
void DMA1_Channel4_5_IRQHandler(void);
void ADC1_COMP_IRQHandler(void);

#endif
