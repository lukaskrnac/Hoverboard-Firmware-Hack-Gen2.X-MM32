#ifndef __HALLHANDLE_H
#define __HALLHANDLE_H
#include <string.h>
#include <stdint.h>
#include "pinout.h"

#define SPEEDX			60000000/WINDINGS*2			//RPM


typedef struct
{
  uint8_t RunHallValue;
  uint8_t PreHallValue;
  uint8_t PreRaw;             //hall state at the last call, direction independent
	int8_t	CMDDIR;
	int8_t	MoveDir;            //measured: 1 electrical angle rising, -1 falling
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
extern volatile uint8_t hallAngleSure;    //sine/FOC angle known (hallhandle.c); FOC applies d current only then
extern volatile int32_t fastspeed;    //rpm from the last hall interval (hallhandle.c), signed like realspeed

extern void HALLModuleInit(HALLType *u);
extern void HALLModuleCalc(HALLType *u);



#endif
