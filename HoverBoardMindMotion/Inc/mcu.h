#ifndef MCU_H
#define MCU_H

// device header of the selected MCU: TARGET_MM32SPIN25, otherwise MM32SPIN05/06
#ifdef TARGET_MM32SPIN25
#include "HAL_device.h"
#include "spin25-redefine.h"
#else
#include "mm32_device.h"
#endif

extern u32 SystemCoreClock;    //system_mm32spin05.c

#endif
