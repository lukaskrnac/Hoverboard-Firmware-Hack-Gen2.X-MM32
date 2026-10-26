#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "mcu.h"

extern volatile uint8_t poweron;     //counts adc conversions until the current offsets are calibrated (>127 = done)
extern volatile uint8_t realdir;     //measured direction from the hall sequence
extern uint16_t vrefint;    //last internal 1.2V reference conversion

#endif
