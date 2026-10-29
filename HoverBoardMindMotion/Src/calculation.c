#include "mcu.h"
#include "main.h"
#include "calculation.h"
#include "pinout.h"
#include "bldc.h"

//running averages of the adc isr: 32 (vbat) and 64 (itotal) samples, one per control period. The samples
//are kept as int16 (0.01 V / 0.01 A: +-327 V / A), clamped
static uint8_t vindex;
static int vdatasum;
static int16_t vavgarr[32];
static uint8_t aindex;
static int adatasum;
static int16_t aavgarr[64];

static int16_t clamp16(int v){ return (int16_t)(v>32767 ? 32767 : (v<-32768 ? -32768 : v)); }

void avgvbat(void){
	int16_t v = clamp16(vbat);
	if(vindex >= 32){
		vindex = 0;
	}
	vdatasum = vdatasum + v - vavgarr[vindex];
	vavgarr[vindex] = v;
	fvbat = (int)(vdatasum>>5);
	vindex++;
}
void avgItotal(void){
	int16_t v = clamp16(itotal);
	if(aindex >= 64){
		aindex = 0;
	}
	adatasum = adatasum + v - aavgarr[aindex];
	aavgarr[aindex] = v;
	fitotal = (int)(adatasum>>6);
	aindex++;
}

//COM_SPEED / SINE_SPEED: PI with feed forward and a reference model, run from the main loop every ~2 ms.
//Output in permille of full scale (duty for block, modulation 32768 for sine), internally permille<<8.
//Plants measured on an unloaded wheel at 24.3 V (first order):
//  block: rpm = 0.87 * (duty_permille - 53), tau 0.5 s (freewheels, no active braking; the old drive, both sides
//  chopping: 0.958 * (duty - 185), see commutation.h)
//  sine:  rpm = 0.588 * (modulation_permille - 10), tau 0.06 s (complementary switching brakes too)
//Feed forward = dead band + rpm / (kv * vbat). The PI acts on the error against the set point filtered
//by tref (reference model), so it only corrects what the feed forward model misses.
#define SPD_UMAX     (950<<8)    //max 95%
#define SPD_IMAX     (300<<8)    //integrator range
#define SPD_SLEW     (3<<8)      //permille per ms
static int32_t spdI, spdU, spdRef;    //spdRef: rpm<<8
static uint8_t spdRun;
static SpeedDeriv spdD;

//the speed's derivative for the D terms: per ms from the hall speed (it moves in steps), 20 ms filter
__attribute__((noinline)) int32_t SpeedDerivStep(SpeedDeriv* f, int32_t meas, int32_t dt){    //one copy, not inlined twice: -48 bytes
	if(dt<=0) return f->acc;
	int32_t a = (meas-f->last)*1000/dt;
	f->last = meas;
	f->acc += (a-f->acc)*dt/20;
	return f->acc;
}
static uint32_t spdT;

//0 and 65535 (an unset word) = default: 65535 was taken as a gain of 65535

void SpeedParams(uint8_t mode, SpeedPlant* p){    //config fields, 0 = default
	//kp, ki, kd, kv, d0 (SpeedPlant order): block (COM_SPEED) and sine words, defaults. Block: gains 2x below
	//where steps from rest start to overshoot (100/200), 4x below instability. Sine: lowest overshoot of a
	//sweep (<= 9%), 8x below instability (40/340) on the bench wheel
	static const uint8_t w[2][5] = { { 48, 49, 78, 50, 51 }, { 52, 53, 77, 54, 55 } };
	static const uint16_t def[2][5] = { { 50, 100, 0, 355, 53 }, { 5, 85, 0, 242, 10 } };
	uint8_t s = mode==SINE_SPEED;
	int32_t* f = &p->kp;
	for(uint8_t k=0;k<5;k++) f[k] = cfgOrDefault(pinstorage[w[s][k]], def[s][k]);
	p->tref = s ? 100 : 350;
	//above 100 % the sine path turns the excess into phase advance (field weakening), if enabled
	p->umax = s&&cfgOrDefault(FW_ADVANCE, 0) ? (1220<<8) : SPD_UMAX;
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
		spdD.last = meas; spdD.acc = 0;
	}
	di = ((sp<<8)-spdRef)*dt/p->tref;
	spdRef = di ? spdRef+di : sp<<8;    //snap once the step rounds to 0
	e = (spdRef>>8) - meas;
	ff = FeedForward(sp, p, v);
	di = p->ki*e*256/10*dt/1000;
	u = ff + p->kp*e*256/10 + spdI + di;
	if((u < p->umax || di < 0) && (u > -p->umax || di > 0)){    //anti windup: no integration into saturation
		spdI += di;
		if(spdI > SPD_IMAX) spdI = SPD_IMAX; else if(spdI < -SPD_IMAX) spdI = -SPD_IMAX;
	}
	u = ff + p->kp*e*256/10 + spdI;
	if(p->kd){    //D on the measured speed: 0.01 permille per rpm/s
		int32_t dd = p->kd*SpeedDerivStep(&spdD, meas, dt)/100;
		if(dd > 1000) dd = 1000; else if(dd < -1000) dd = -1000;
		u -= dd<<8;
	}
	if(u > p->umax) u = p->umax; else if(u < -p->umax) u = -p->umax;
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
	if(ff < 0) ff = 0;    //below the dead band (friction: ~5 % at standstill) no torque
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
