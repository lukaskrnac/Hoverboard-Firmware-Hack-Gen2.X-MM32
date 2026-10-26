#ifndef FOC_Math_h
#define FOC_Math_h

#ifdef TARGET_MM32SPIN25
#include "HAL_device.h"                 // Device header
#else
#include "mm32_device.h"                // Device header
#endif

typedef struct
{
	int16_t hCos;
	int16_t hSin;
} Trig_Components;

extern Trig_Components MCM_Trig_Functions(int16_t hAngle);
extern int32_t MCM_Sqrt(int32_t wInput);

#endif
