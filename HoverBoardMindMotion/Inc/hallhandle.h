#ifndef __HALLHANDLE_H
#define __HALLHANDLE_H
#include <string.h>
#include <stdint.h>
#include "../Inc/pinout.h"

#define SPEEDX			60000000/WINDINGS*2			//RPM


typedef struct
{
  uint8_t RunHallValue;   	
  uint8_t PreHallValue;   
  uint8_t PreRaw;             //hall state at the last call, direction independent
	int8_t	CMDDIR;
	int16_t	Angle;
	int16_t IncAngle;
	int16_t IncAngleMax;
	int16_t SpeedTemp;
	uint8_t Time100msCNT;
	uint32_t HallTime[8];		
	uint32_t HallTimeSum;
	uint16_t CWAngleTab[8];
}
HALLType;



extern HALLType HALL1;					

extern void HALLModuleInit(HALLType *u);
extern void HALLModuleCalc(HALLType *u);



#endif

