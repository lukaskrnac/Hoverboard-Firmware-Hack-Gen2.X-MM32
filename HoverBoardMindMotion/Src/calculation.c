#include "mcu.h"
#include "main.h"
#include "calculation.h"
#include "pinout.h"
#include "bldc.h"

//running averages of the adc isr: 32 (vbat) and 64 (itotal) samples, one per pwm period
static uint8_t vindex;
static int vdatasum;
static int vavgarr[32];
static uint8_t aindex;
static int adatasum;
static int aavgarr[64];

void avgvbat(void){

	if(vindex >= 32){
    vindex = 0;
  }
  vdatasum = vdatasum + vbat - vavgarr[vindex];
  vavgarr[vindex] = vbat;
  fvbat = (int)(vdatasum>>5);
  vindex++;
}
void avgItotal(void){

	if(aindex >= 64){
    aindex = 0;
  }
  adatasum = adatasum + itotal - aavgarr[aindex];
  aavgarr[aindex] = itotal;
  fitotal = (int)(adatasum>>6);
  aindex++;
}

//COM_SPEED / SINE_SPEED: PI with feed forward and a reference model, run from the main loop every ~2 ms.
//Output in permille of full scale (duty for block, modulation 32768 for sine), internally permille<<8.
//Plants measured on an unloaded wheel at 24.3 V (first order):
//  block: rpm = 0.958 * (duty_permille - 185), tau 0.5 s (freewheels, no active braking)
//  sine:  rpm = 0.588 * (modulation_permille - 10), tau 0.06 s (complementary switching brakes too)
//Feed forward = dead band + rpm / (kv * vbat). The PI acts on the error against the set point filtered
//by tref (reference model), so it only corrects what the feed forward model misses.
#define SPD_UMAX     (950<<8)    //max 95%
#define SPD_IMAX     (300<<8)    //integrator range
#define SPD_SLEW     (3<<8)      //permille per ms
static int32_t spdI, spdU, spdRef;    //spdRef: rpm<<8
static uint8_t spdRun;
static uint32_t spdT;

static int32_t cfgOr(uint16_t v, int32_t def){ return v ? v : def; }

void SpeedParams(uint8_t mode, SpeedPlant* p){    //config fields, 0 = default
	if(mode==SINE_SPEED){
		//default gains: lowest overshoot of a sweep (<= 9%), 8x below instability (40/340) on the bench wheel
		p->kp = cfgOr(SINE_KP, 5); p->ki = cfgOr(SINE_KI, 85);
		p->kv = cfgOr(SINE_KV, 242); p->d0 = cfgOr(SINE_D0, 10); p->tref = 100;
	}else{
		//default gains 2x below where steps from rest start to overshoot (100/200), 4x below instability
		p->kp = cfgOr(SPEED_KP, 50); p->ki = cfgOr(SPEED_KI, 100);
		p->kv = cfgOr(SPEED_KV, 395); p->d0 = cfgOr(SPEED_D0, 185); p->tref = 350;
	}
}

void SpeedCtrlReset(void){
	spdI = spdU = 0;
	spdRun = 0;
	spdT = millis;
}

static int32_t FeedForward(int32_t rpm, const SpeedPlant* p, int32_t v){    //permille<<8, |rpm| <= 1000
	if(rpm==0) return 0;    //no dead band push at a 0 rpm set point (braking to standstill)
	int32_t a = rpm < 0 ? -rpm : rpm;
	int32_t ff = (p->d0<<8) + a*256000/(p->kv*v/1000);     //1000/(kv/10 * v/100) per rpm
	return rpm < 0 ? -ff : ff;
}

int SpeedCtrl(int sp, int meas, const SpeedPlant* p){    //sp, meas in rpm; returns permille, signed
	int32_t v = vbat > 1000 ? vbat : 2400;          //0.01 V; until measured assume 24 V
	int32_t dt = millis - spdT;
	int32_t e, ff, u, di;
	if(sp > 1000) sp = 1000; else if(sp < -1000) sp = -1000;    //keeps the fixed point math in 32 bit
	spdT = millis;
	if(dt > 20) dt = 20;
	if(meas > 1000) meas = 1000; else if(meas < -1000) meas = -1000;
	if(!spdRun){    //(re)start bumpless from the speed the wheel has (coasting, drive mode change):
		spdRef = meas<<8;    //reference and output from the measured speed. Starting the output at 0
		//braked a turning wheel in sine mode (to 75 from 150 rpm) while it slewed up. Below 20 rpm from 0:
		//the dead band part of the feed forward against the motion overshot a step from a slow drift
		spdU = (meas>20||meas<-20) ? FeedForward(meas, p, v) : 0;
		spdRun = 1;
	}
	di = ((sp<<8)-spdRef)*dt/p->tref;
	spdRef = di ? spdRef+di : sp<<8;    //snap once the step rounds to 0
	e = (spdRef>>8) - meas;
	ff = FeedForward(sp, p, v);
	di = p->ki*e*256/10*dt/1000;
	u = ff + p->kp*e*256/10 + spdI + di;
	if((u < SPD_UMAX || di < 0) && (u > -SPD_UMAX || di > 0)){    //anti windup: no integration into saturation
		spdI += di;
		if(spdI > SPD_IMAX) spdI = SPD_IMAX; else if(spdI < -SPD_IMAX) spdI = -SPD_IMAX;
	}
	u = ff + p->kp*e*256/10 + spdI;
	if(u > SPD_UMAX) u = SPD_UMAX; else if(u < -SPD_UMAX) u = -SPD_UMAX;
	if(u > spdU + SPD_SLEW*dt) u = spdU + SPD_SLEW*dt;
	else if(u < spdU - SPD_SLEW*dt) u = spdU - SPD_SLEW*dt;
	spdU = u;
	return u>>8;
}

//COM_TORQUE: current loop on the bus shunt. In block mode the adc samples at the pwm counter's zero,
//inside the high side on time, so the shunt carries the phase current of the conducting pair there
//(any duty > 0). Output in permille duty for the direction of the command: back emf feed forward plus
//PI on the filtered current (fitotal, ~15 ms). Duty for 1 A ~ 2R/Vbus: 25 permille at 24 V.
static int32_t trqI;    //permille << 8
static uint32_t trqT;

void TorqueCtrlReset(void){
	trqI = 0;
	trqT = millis;
}

int TorqueCtrlBlock(int32_t iref, int32_t imeas, int32_t rpm){    //mA (magnitude, imeas >= 0), rpm along the command
	SpeedPlant p;
	SpeedParams(COM_SPEED, &p);
	int32_t v = vbat > 1000 ? vbat : 2400;
	int32_t dt = millis - trqT;
	trqT = millis;
	if(dt > 20) dt = 20;
	if(rpm > 1000) rpm = 1000; else if(rpm < -1000) rpm = -1000;
	int32_t ff = (p.d0<<8)+rpm*256000/(p.kv*v/1000);    //permille<<8: FeedForward() of the speed along the command;
	if(ff < 0) ff = 0;    //no current flows below the dead band (block needs 18.5 % at standstill)
	int32_t e = iref - imeas;
	int32_t di = 800*e/1000*256/1000*dt;    //ki 800 permille per A*s
	int32_t u = ff + 5*e*256/1000 + trqI + di;    //kp 5 permille per A
	if((u < SPD_UMAX || di < 0) && (u > 0 || di > 0)){
		trqI += di;
		if(trqI > SPD_UMAX) trqI = SPD_UMAX; else if(trqI < -SPD_UMAX) trqI = -SPD_UMAX;
	}
	u = ff + 5*e*256/1000 + trqI;
	if(u > SPD_UMAX) u = SPD_UMAX; else if(u < 0) u = 0;    //no reverse duty: that would be plugging
	return u>>8;
}
