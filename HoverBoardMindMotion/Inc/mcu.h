#ifndef MCU_H
#define MCU_H

// The MCU is selected by the Makefile (make MCU=...): MCU_SPIN05 (default), MCU_SPIN06, MCU_SPIN25, MCU_F031.
// All of them use the SPIN05 register header: TIM1/2/3, ADC1, UART, DMA, GPIO, RCC, FLASH, IWDG and the
// hardware divider have the same addresses and layout on all four (vendor SVD files compared). The
// MM32F031 ("q") reads the SPIN05's IDCODE (0xCC4460B1): the same die without the gate driver.
#include "mm32_device.h"

#if defined(MCU_SPIN25)
#define MCU_NAME "MM32SPIN25"
#define MCU_PLL 1         //SystemInit: the HSI (48 MHz / 4) through the PLL, no HSI_72M bit
#define MCU_OPAMPS 1      //4 op-amps with I/O pins, enabled for the configured current pins (initialize.c)
#elif defined(MCU_SPIN06)
#define MCU_NAME "MM32SPIN06"
#elif defined(MCU_F031)
#define MCU_NAME "MM32F031"
#define MCU_NO_UART2 1    //UART2 has no pins on the F031
#else
#define MCU_NAME "MM32SPIN05"
#endif

extern u32 SystemCoreClock;    //system_mm32spin05.c

#endif
