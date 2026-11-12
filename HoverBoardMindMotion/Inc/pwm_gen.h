#ifndef PWM_GEN_H
#define PWM_GEN_H

/*-------------------- Includes -----------------------*/
#include "mcu.h"                // device header of the selected MCU

/*------------------- Private variables ---------------*/
#define SEVENMODE		1
#define FIVEMODE		0

typedef struct {
			int32_t	Alpha;
			int32_t	Beta;
			int16_t	CompA;
			int16_t	CompB;
			int16_t	CompC;
			uint8_t	Sector;
			uint8_t	Mode;
			int16_t	N_halfPeriod;
}PWM_GEN_T;


extern PWM_GEN_T pwm_gen;

extern void PWM_GEN_init(PWM_GEN_T *u1);
extern void PWM_GEN_calc(PWM_GEN_T *u1);
extern void Update_PWM(PWM_GEN_T *u1);
extern void SVPWM(void);
#endif
