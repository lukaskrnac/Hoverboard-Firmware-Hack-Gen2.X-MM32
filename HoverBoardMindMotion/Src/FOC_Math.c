/*-------------------- Includes -----------------------*/
#include "mcu.h"
#include "FOC_Math.h"

/*--------------------Private macro ------------------*/
//sine of a quarter turn in 64 steps and its end, Q15. MCM_Trig_Functions interpolates linearly between them:
//error < 4 / 32768 at any 16 bit angle. (It was a 256 step table looked up without interpolation: errors up
//to 201 / 32768 = 0.35 deg, and 512 bytes; this one is 130, which made room for the SPIN25 build.)
static const int16_t qSin[65] = {
	0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512,
	10278, 11039, 11793, 12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868,
	19519, 20159, 20787, 21403, 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319,
	26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113,
	31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757, 32767
};


/*------------------ Private functions ----------------*/
static int16_t sinQ15(uint16_t a){    //65536 per turn
	uint16_t q = a>>14, r = a&0x3FFF;    //quadrant, place in it (16384)
	if(q&1) r = (uint16_t)(0x4000-r);    //the second and fourth run backwards
	uint16_t i = r>>8, f = r&0xFF;
	int32_t v = i>=64 ? qSin[64] : qSin[i]+(((int32_t)(qSin[i+1]-qSin[i])*f)>>8);
	return (int16_t)(q&2 ? -v : v);
}

Trig_Components MCM_Trig_Functions(int16_t hAngle)    //hAngle 65536 per turn (0 = 0 rad), Q15 sin / cos
{
	Trig_Components c;
	c.hSin = sinQ15((uint16_t)hAngle);
	c.hCos = sinQ15((uint16_t)((uint16_t)hAngle+16384u));
	return c;
}

/*************************************
	Function Name：MCM_Sqrt
	Description：Square root calculation
	Input：wInput---Calculated value
	Output：wtemprootnew---Square root
**************************************/
int32_t MCM_Sqrt(int32_t wInput)
{
  uint8_t biter = 0u;
  int32_t wtemproot;
  int32_t wtemprootnew;

  if (wInput > 0)
  {
    if (wInput <= (int32_t)2097152)
    {
      wtemproot = (int32_t)128;
    }
    else
    {
      wtemproot = (int32_t)8192;
    }

    do
    {
      wtemprootnew = (wtemproot + wInput/wtemproot)/(int32_t)2;
	//		divtemp = Division(wInput,wtemproot);
		//	wtemprootnew = (wtemproot + divtemp)>>1;

      if (wtemprootnew == wtemproot)
      {
        biter = 6u;
      }
      else
      {
        biter ++;
        wtemproot = wtemprootnew;
      }
    }
    while (biter < 6u);
  }
  else
  {
    wtemprootnew = (int32_t)0;
  }

  return (wtemprootnew);
}
