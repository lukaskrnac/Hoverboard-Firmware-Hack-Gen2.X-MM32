//ODrive input modes, run in speedupdate() (~2 ms):
//  PASSTHROUGH 1  the command as it comes (position: plus the Vel_FF of Set_Input_Pos)
//  VEL_RAMP    2  velocity control: the set point moves to input_vel at vel_ramp_rate
//  POS_FILTER  3  position control: ODrive's critically damped second order filter of input_pos
//                 (input_filter_bandwidth); its velocity is the speed loop's feed forward
//  TRAP_TRAJ   5  position control: ODrive's trapezoidal planner (trap_traj vel_limit, accel_limit,
//                 decel_limit) from the current set point and velocity to input_pos, planned again
//                 when input_pos changes; velocity feed forward as POS_FILTER
//The position loop (P on the hall steps, bldc.c) corrects what the wheel misses of the set point.
//The position set point is kept relative to the target (base): float has 24 bits, at 125000 steps one
//unit in the last place is already 0.008 steps.
#include <string.h>
#include "inputmode.h"
#include "pinout.h"
#include "cansimple.h"

float inPosSp, inVelSp;
static float rampRpm;
static int32_t base;       //target, steps
static float rel;          //position set point - base, steps
static uint8_t planned;
static float Xi, Xf, Vi, Ar, Dr, Vr, Ta, Tv, Td, Tf, yAcc, t;    //the trapezoid, as ODrive names it

static float centi(uint16_t w, int32_t def){ return (float)cfgOrDefault(w, def)/100.0f; }
static int32_t roundF(float f){ return (int32_t)(f<0 ? f-0.5f : f+0.5f); }

static float sqrtF(float x){    //Newton from the exponent halved (soft float: no sqrtf from libm)
	if(x<=0) return 0;
	uint32_t i; memcpy(&i, &x, 4);
	i = 0x1FBD1DF5u+(i>>1);
	float r; memcpy(&r, &i, 4);
	for(uint8_t k=0;k<4;k++) r = 0.5f*(r+x/r);
	return r;
}

uint8_t InputModeSupported(uint32_t m){
	return m==IM_PASSTHROUGH||m==IM_VEL_RAMP||m==IM_POS_FILTER||m==IM_TRAP_TRAJ;
}

uint8_t InputMode(void){
	uint16_t m = INPUT_MODE;
	return InputModeSupported(m) ? (uint8_t)m : IM_PASSTHROUGH;
}

void InputTrack(int32_t steps, int32_t rpm){
	base = steps; rel = 0;
	inPosSp = (float)steps;
	inVelSp = (float)rpm*(float)StepsPerTurn()/60.0f;
	rampRpm = (float)rpm;
	planned = 0;
}

int32_t InputVel(int32_t rpm, int32_t dtms, uint8_t holdDown, int32_t meas){
	if(InputMode()!=IM_VEL_RAMP){
		rampRpm = (float)rpm;
	}else{
		float step = centi(VEL_RAMP_RATE, 100)*60.0f*(float)dtms/1000.0f;    //rpm in this period
		float d = (float)rpm-rampRpm;
		if(holdDown&&rampRpm!=0.0f&&(d>0)!=(rampRpm>0)){
			//slowing down while the bus is high: the ramp pauses, but not above the wheel (it slows by
			//friction): a set point held above it made the speed loop drive it up again (3 Hz cycles)
			step = 0;
			float m = (float)meas;
			if(rampRpm>0 ? (m<rampRpm&&m>=0) : (m>rampRpm&&m<=0)) rampRpm = m;
		}
		rampRpm += d>step ? step : (d<-step ? -step : d);
	}
	inVelSp = rampRpm*(float)StepsPerTurn()/60.0f;
	return roundF(rampRpm);
}

static void plan(float spt){    //ODrive TrapezoidalTrajectory::planTrapezoidal, from rel to 0
	float vmax = centi(TRAJ_VEL, 200)*spt, amax = centi(TRAJ_ACCEL, 50)*spt, dmax = centi(TRAJ_DECEL, 50)*spt;
	float xi = rel, vi = inVelSp, xf = 0;
	float dx = xf-xi;
	float stop = vi*vi/(2.0f*dmax);                        //stopping distance
	float s = dx-(vi<0 ? -stop : stop)<0 ? -1.0f : 1.0f;   //direction of the cruise
	Ar = s*amax; Dr = -s*dmax; Vr = s*vmax;
	if(s*vi>s*Vr) Ar = -s*amax;                            //faster than the cruise: decelerate to it
	Ta = (Vr-vi)/Ar;
	Td = -Vr/Dr;
	float dxmin = 0.5f*Ta*(Vr+vi)+0.5f*Td*Vr;              //distance to reach the cruise and stop
	if(s*dx<s*dxmin){    //short move: triangle
		float r = (Dr*vi*vi+2.0f*Ar*Dr*dx)/(Dr-Ar);
		Vr = s*sqrtF(r);
		Ta = (Vr-vi)/Ar; if(Ta<0) Ta = 0;
		Td = -Vr/Dr; if(Td<0) Td = 0;
		Tv = 0;
	}else{
		Tv = (dx-dxmin)/Vr;
	}
	Tf = Ta+Tv+Td;
	Xi = xi; Xf = xf; Vi = vi;
	yAcc = xi+vi*Ta+0.5f*Ar*Ta*Ta;
	t = 0;
}

static void eval(void){    //ODrive TrapezoidalTrajectory::eval
	if(t<Ta){
		rel = Xi+Vi*t+0.5f*Ar*t*t; inVelSp = Vi+Ar*t;
	}else if(t<Ta+Tv){
		rel = yAcc+Vr*(t-Ta); inVelSp = Vr;
	}else if(t<Tf){
		float td = t-Tf;
		rel = Xf+0.5f*Dr*td*td; inVelSp = Dr*td;
	}else{
		rel = Xf; inVelSp = 0;
	}
}

int32_t InputPos(int32_t target, int32_t ffRpm, int32_t dtms, int32_t* velFf){
	float spt = (float)StepsPerTurn(), dt = (float)dtms/1000.0f;
	uint8_t m = InputMode();
	uint8_t moved = target!=base;
	rel += (float)(base-target);    //exact for differences below 2^24 steps
	base = target;
	if(m==IM_TRAP_TRAJ){
		if(!planned||moved){
			plan(spt);
			planned = 1;
		}
		if(t<Tf) t += dt;
		eval();
	}else{
		planned = 0;
		if(m==IM_POS_FILTER){    //ODrive: ki = 2 * bandwidth, kp = ki^2 / 4 (critically damped)
			float ki = 2.0f*centi(INPUT_FILTER_BW, 200), kp = 0.25f*ki*ki;
			float acc = -kp*rel-ki*inVelSp;
			inVelSp += dt*acc;
			rel += dt*inVelSp;
		}else{
			rel = 0;
			inVelSp = (float)ffRpm*spt/60.0f;
		}
	}
	inPosSp = (float)base+rel;
	*velFf = roundF(inVelSp*60.0f/spt);
	return base+roundF(rel);
}

uint8_t InputPosDone(void){
	if(InputMode()==IM_TRAP_TRAJ) return t>=Tf;
	return inVelSp<0.5f&&inVelSp>-0.5f;
}
