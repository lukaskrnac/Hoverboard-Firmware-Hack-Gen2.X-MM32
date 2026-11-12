#ifndef FOC_Math_h
#define FOC_Math_h

#include "mcu.h"                // device header of the selected MCU

typedef struct
{
	int16_t hCos;
	int16_t hSin;
} Trig_Components;

extern Trig_Components MCM_Trig_Functions(int16_t hAngle);
extern int32_t MCM_Sqrt(int32_t wInput);

#endif
