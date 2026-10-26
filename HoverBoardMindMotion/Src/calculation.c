#include "../Inc/mcu.h"
#include "../Inc/main.h"
#include "../Inc/calculation.h"   
#include "../Inc/pinout.h"   
#include "../Inc/bldc.h"   

uint8_t Kp=4;
uint8_t Ki=10;
uint8_t Kd=180;
int max = 4000;
int min = -4000;
int qdSum = 0;
int lasterr=0;
int prevpwm=0;

uint8_t sindex;
int datasum;
int avgarr[32];
uint8_t vindex;
int vdatasum;
int vavgarr[32];
uint8_t aindex;
int adatasum;
int aavgarr[64];

void PID_Init(){
	switch(DRIVEMODE){
		case COM_SPEED:
			Kp=50;
			Ki=100;
			Kd=200;
			max = 4000;
			min = -4000;
		break;
		case SINE_SPEED:
			Kp=50;
			Ki=100;
			Kd=200;
			max = 40000;
			min = -40000;
		break;
	}

}
void PIDrst(){
	lasterr=qdSum=prevpwm=0;
}


int PID(int setpoint,int real){
	
	int qOut = 0;
	int Error;
	signed long  Up;
	signed long  Ui;
	signed long  Ud;
	//Calc Error
	Error = setpoint - real;
	//Calc proportional
	Up = (Kp * Error);
	//Calc integral
	qdSum = qdSum + Ki * Error;
	//Limit integral 
	if(qdSum >= (max)){
		qdSum = (max);
	}else if(qdSum <= min){
		qdSum = (min);
	}
	Ui = qdSum/100;
	Ud=(Error-lasterr)*Kd;
	lasterr=Error;
	qOut = Up + Ui + Ud;		
	//Out Limit
	if(qOut > max){
		qOut =max ;
	}else if(qOut < min){
		qOut = min;
	}
	return qOut;
	
}

void avgspeed(){
	
	if(sindex >= 32){
    sindex = 0;
  } 
  datasum = datasum + realspeed - avgarr[sindex];
  avgarr[sindex] = realspeed;
  frealspeed = (int)(datasum>>5);
  sindex++;
}
void avgvbat(){
	
	if(vindex >= 32){
    vindex = 0;
  } 
  vdatasum = vdatasum + vbat - vavgarr[vindex];
  vavgarr[vindex] = vbat;
  fvbat = (int)(vdatasum>>5);
  vindex++;
}
void avgItotal(){
	
	if(aindex >= 64){
    aindex = 0;
  } 
  adatasum = adatasum + itotal - aavgarr[aindex];
  aavgarr[aindex] = itotal;
  fitotal = (int)(adatasum>>6);
  aindex++;
}
int PID2PWM(int pid){
	int pwm = prevpwm += pid;
	if(pwm > max){
		pwm =max ;
	}else if(pwm < min){
		pwm = min;
	}
	if(prevpwm > max){
		prevpwm =max ;
	}else if(prevpwm < min){
		prevpwm = min;
	}
	return pwm*PWM_RES/4000;
}

//COM_SPEED: PI with feed forward on the duty, run from the main loop every ~2 ms.
//Plant measured on an unloaded wheel at 24 V: rpm = 0.945 * (duty_permille - 185), first order, tau 0.5 s.
//Feed forward = dead band + rpm / (kv * vbat). Default gains 2x below where steps from rest start to
//overshoot (100/200) and 4x below instability (200/400) on that wheel.
//Internal values are permille duty << 8.
#define SPD_UMAX     (950<<8)    //max 95% duty
#define SPD_IMAX     (300<<8)    //integrator range
#define SPD_SLEW     (3<<8)      //permille per ms
#define SPD_TREF     350         //ms, reference model: the error is taken against the set point filtered
                                 //like the wheel can follow it, so P and I only correct the model error
static int32_t spdI, spdU, spdRef;    //spdRef: rpm<<8
static uint8_t spdRun;
static uint32_t spdT;

void SpeedCtrlReset(void){
	spdI = spdU = 0;
	spdRun = 0;
	spdT = millis;
}

int SpeedCtrl(int sp, int meas){    //sp, meas in rpm; returns pwm counts, signed
	int32_t kp = SPEED_KP ? SPEED_KP : 50, ki = SPEED_KI ? SPEED_KI : 100;
	int32_t kv = SPEED_KV ? SPEED_KV : 389, d0 = SPEED_D0 ? SPEED_D0 : 185;
	int32_t v = vbat > 1000 ? vbat : 2400;          //0.01 V; until measured assume 24 V
	int32_t dt = millis - spdT;
	int32_t e, a, ff, u, di;
	if(sp > 1000) sp = 1000; else if(sp < -1000) sp = -1000;    //keeps the fixed point math in 32 bit
	spdT = millis;
	if(dt > 20) dt = 20;
	if(!spdRun){    //(re)start from the speed the wheel has, e.g. while coasting
		spdRef = meas<<8;
		spdRun = 1;
	}
	di = ((sp<<8)-spdRef)*dt/SPD_TREF;
	spdRef = di ? spdRef+di : sp<<8;    //snap once the step rounds to 0
	e = (spdRef>>8) - meas; a = sp < 0 ? -sp : sp;
	ff = (d0<<8) + a*256000/(kv*v/1000);           //permille<<8: 1000/(kv/10 * v/100) per rpm
	if(sp < 0) ff = -ff;
	di = ki*e*256/10*dt/1000;
	u = ff + kp*e*256/10 + spdI + di;
	if((u < SPD_UMAX || di < 0) && (u > -SPD_UMAX || di > 0)){    //anti windup: no integration into saturation
		spdI += di;
		if(spdI > SPD_IMAX) spdI = SPD_IMAX; else if(spdI < -SPD_IMAX) spdI = -SPD_IMAX;
	}
	u = ff + kp*e*256/10 + spdI;
	if(u > SPD_UMAX) u = SPD_UMAX; else if(u < -SPD_UMAX) u = -SPD_UMAX;
	if(u > spdU + SPD_SLEW*dt) u = spdU + SPD_SLEW*dt;
	else if(u < spdU - SPD_SLEW*dt) u = spdU - SPD_SLEW*dt;
	spdU = u;
	return (u>>8)*(int32_t)PWM_RES/1000;
}

uint32_t updateMotorRPM(uint32_t halltime){
    return  60000000 / (halltime * WINDINGS);
}



