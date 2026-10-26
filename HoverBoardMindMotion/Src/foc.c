//Field oriented control on two measured phase currents.
//
//Current sensing (MM32SPIN05PFOP bench board): the chip's two internal op-amps amplify the low side
//current of phases B and C, outputs on PA2 / PA6 (no register setup: they run from reset). An output
//is only valid while the low sides conduct and saturates otherwise (gates off, high side on), so the
//adc samples at the pwm counter's zero (TIM1 CC4 = 1, centre of the low side time), the zero current
//offsets are measured with the gates on and the zero vector applied, and the modulation is limited to
//90 % of the linear range (low side time at the sample >= ~5 us). Measured gain: 19.4 counts/A
//(51.5 mA per count), offset ~1520 counts.
//Received UART traffic (the master transmitting on the RemoteUartBus) couples into the op-amp outputs:
//at 115200 ~5 samples per second read ~170 counts (~9 A) off on one channel, mostly for one period,
//sometimes for 4 in a row; nothing without traffic, a longer adc sample time does not help. A median of
//3 let the 4 period events through (false CURRENT_LIMIT_VIOLATION trips), and comparing with the last
//accepted sample locked onto a borderline glitch (then rejected the true values). Now each channel keeps
//its last 9 raw samples: the newest is used when it is within GLITCH counts (~3 A) of their median,
//otherwise the median (runs of up to 4 bad samples do not move it). No delay normally; a real step
//(> ~1 A per period is not possible here at these voltages anyway) shows after up to 4 periods.
//A trip needs two filtered samples in a row.
//
//Loop: every pwm period in the adc isr. Park angle = hall estimator angle minus its 1.5 period lead
//(that lead is for the pwm output; the currents were sampled now); PI per axis in mV with back emf feed
//forward on q; d voltage first, q gets the rest of the voltage circle; integration stops in saturation.
#include "foc.h"
#include "main.h"
#include "pinout.h"
#include "hardware.h"
#include "bldc.h"
#include "hallhandle.h"
#include "ipark.h"
#include "pwm_gen.h"
#include "FOC_Math.h"
#include "calculation.h"

#define TS_US        228          //pwm period = 2*PWM_RES/72 MHz = 227.6 us
#define MOD_MAX      29491        //90 % of the linear svpwm range (32768)
#define INV_SQRT3_Q15 18919

static int32_t cfg(uint16_t v, int32_t def){ return v ? v : def; }
#define GAIN()   cfg(IPHASE_GAIN, 515)    //0.1 mA per adc count
#define IMAX()   (cfg(FOC_IMAX, 100)*100) //mA
#define KP()     cfg(CUR_KP, 400)         //mV per A
#define KI()     cfg(CUR_KI, 300)         //V per A*s

static volatile u32* adc1;    //result registers of the two current pins
static volatile u32* adc2;
static int32_t off1, off2;         //zero current, adc counts << 4
static int32_t calSum1, calSum2;
static uint8_t calN, offValid, armedPeriods, tripN;
#define GLITCH 60                  //adc counts (~3 A)
#define HIST 9
static int16_t hist1[HIST], hist2[HIST];    //raw samples per channel, ring
static uint8_t histPos;
volatile uint16_t focGlitches;     //samples replaced by the median, diagnostic
static int32_t ia, ib, ic;         //mA, this period
static int32_t intD, intQ;         //integrators, uV
static int32_t fwId;               //field weakening d current, mA (<= 0)

volatile int32_t focIdRef;
volatile int16_t focId, focIq;
volatile uint8_t focTrip;
static volatile int32_t focIqRef;  //mA, motor frame
static volatile int16_t focVCmd;   //permille, motor frame; voltage mode if nonzero
static volatile uint8_t focVoltMode;

static int32_t lastIdRef, lastIqRef;

//capture for tuning and fault analysis over SWD, per pwm period: id, iq (mA), vd, vq (mV, electrical
//frame), ia (mA), angle. focTraceN: write 0xFF = one shot from the next reference step of >= 200 mA;
//0xFE = ring buffer until a trip (then focTraceN = FOC_TRACE_N, the oldest entry at focTracePos)
#define FOC_TRACE_N 64
volatile int16_t focTrace[FOC_TRACE_N][6];
volatile uint8_t focTraceN = FOC_TRACE_N;
volatile uint8_t focTracePos;
volatile int16_t focTripI[3];    //phase currents of the sample that tripped, mA
static int32_t median9(const int16_t* h){    //insertion sort of a copy
	int16_t v[HIST];
	for(uint8_t i=0;i<HIST;i++){
		int16_t x = h[i]; int8_t j = (int8_t)(i-1);
		while(j>=0&&v[j]>x){ v[j+1] = v[j]; j--; }
		v[j+1] = x;
	}
	return v[HIST/2];
}

static volatile u32* adcResultReg(uint16_t io){
	if(io>=PINCOUNT) return 0;
	for(uint8_t i=0;i<ADCCOUNT;i++){
		if(adcs[i].io==io&&adcs[i].channel<=ADC_Channel_9){
			return &ADC1->ADDR0+adcs[i].channel;
		}
	}
	return 0;
}

int32_t FocIqSetpoint(void){    //mA, motor frame (voltage mode: the limit in the command's direction)
	return focVoltMode ? (focVCmd>0 ? IMAX() : (focVCmd<0 ? -IMAX() : 0)) : focIqRef;
}

int32_t CurrentHardMax(void){    //mA: phase current trip (ODrive current_hard_max)
	int32_t t = cfgOrDefault(CUR_TRIP, 0)*100;
	if(!t){ t = IMAX()*2; if(t<15000) t = 15000; }    //default: twice the limit, at least 15 A
	return t;
}

uint8_t FocAvailable(void){
	return adcResultReg(IPHASEAPIN)&&adcResultReg(IPHASEBPIN);
}

void FocInit(void){
	adc1 = adcResultReg(IPHASEAPIN);
	adc2 = adcResultReg(IPHASEBPIN);
	FocReset();
}

void FocReset(void){
	intD = intQ = 0;
	fwId = 0;
	focIqRef = 0;
	focVCmd = 0;
	FocSpeedReset();
}

uint8_t FocHoldOff(void){
	return focTrip||(!offValid&&realspeed!=0);    //offsets need the zero vector: it brakes a turning wheel
}

void FocClearTrip(void){
	focTrip = 0;
}

void FocMeasure(void){
	if(!adc1||!adc2) return;
	int32_t r1 = *adc1&0xFFF, r2 = *adc2&0xFFF;
	if(TIM1->BDTR&TIM_BDTR_MOEN){    //gates on during this sample
		if(armedPeriods<255) armedPeriods++;
	}else{
		armedPeriods = 0;
	}
	ia = ib = ic = 0;
	if(armedPeriods<2){    //outputs saturated, or the first period after arming
		for(uint8_t k=0;k<HIST;k++){ hist1[k] = (int16_t)r1; hist2[k] = (int16_t)r2; }
		return;
	}
	if(armedPeriods==2){ for(uint8_t k=0;k<HIST;k++){ hist1[k] = (int16_t)r1; hist2[k] = (int16_t)r2; } }
	hist1[histPos] = (int16_t)r1; hist2[histPos] = (int16_t)r2;
	histPos = (uint8_t)((histPos+1)%HIST);
	int32_t m1 = median9(hist1), m2 = median9(hist2);
	if(r1-m1>GLITCH||m1-r1>GLITCH){ r1 = m1; focGlitches++; }
	if(r2-m2>GLITCH||m2-r2>GLITCH){ r2 = m2; focGlitches++; }
	if(!offValid){    //FocControl() applies the zero vector until the offsets are known
		if(armedPeriods>=4&&realspeed==0){
			calSum1 += r1; calSum2 += r2;
			if(++calN==64){
				off1 = calSum1>>2; off2 = calSum2>>2;    //mean << 4
				offValid = 1;
			}
		}
		return;
	}
	int32_t g = GAIN();
	int32_t c1 = (off1-(r1<<4))*g/160, c2 = (off2-(r2<<4))*g/160;    //mA, adc falls with current into the motor
	if(IPHASE_CFG&4){
		c1 = -c1; c2 = -c2;
	}
	switch(IPHASE_CFG&3){
		case 1: ib = c1; ic = c2; ia = -ib-ic; break;
		case 2: ia = c1; ic = c2; ib = -ia-ic; break;
		default: ia = c1; ib = c2; ic = -ia-ib; break;
	}
	int32_t trip = CurrentHardMax();
	if(ia>trip||ia<-trip||ib>trip||ib<-trip||ic>trip||ic<-trip){
		if(++tripN>=2){
			focTrip = 1;
			focTripI[0] = (int16_t)ia; focTripI[1] = (int16_t)ib; focTripI[2] = (int16_t)ic;
			if(focTraceN==0xFE) focTraceN = FOC_TRACE_N;    //freeze the ring
		}
	}else{
		tripN = 0;
	}
	//bus current = sum of duty * phase current; the CCRs read back are the ones active in this period
	int32_t n = PWM_RES;
	itotal = ((n-(int32_t)TIM1->CCR1)*ia+(n-(int32_t)TIM1->CCR2)*ib+(n-(int32_t)TIM1->CCR3)*ic)/n/10;
}

static void zeroVector(void){
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_RES/2;
}

void FocControl(void){
	if(!(TIM1->BDTR&TIM_BDTR_MOEN)){
		intD = intQ = 0;
		return;
	}
	if(!offValid||armedPeriods<2){
		intD = intQ = 0;
		zeroVector();
		return;
	}
	int32_t v = vbat>1000 ? vbat : 2400;    //0.01 V
	int32_t vmax = MOD_MAX*v/5676;          //mV; phase amplitude Vbus/sqrt(3) at modulation 32768
	int16_t th = HALL1.Angle;
	int16_t thS = fastspeed ? (int16_t)(th-HALL1.MoveDir*(HALL1.IncAngle*3/2)) : th;
	Trig_Components sc = MCM_Trig_Functions(thS);
	int32_t alpha = ia, beta = (ib-ic)*INV_SQRT3_Q15>>15;
	int32_t id = (alpha*sc.hCos>>15)+(beta*sc.hSin>>15);
	int32_t iq = (beta*sc.hCos>>15)-(alpha*sc.hSin>>15);
	focId = (int16_t)id;
	focIq = (int16_t)-iq;

	//references in the electrical frame: + q turns the angle up = negative realspeed
	int32_t imax = IMAX();
	int32_t idRef = focIdRef+fwId, iqRef;
	if(idRef>imax) idRef = imax; else if(idRef<-imax) idRef = -imax;
	int32_t iqmax = fwId ? MCM_Sqrt(imax*imax-idRef*idRef) : imax;    //the total current stays within the limit
	if(focVoltMode){
		iqRef = focVCmd>0 ? -iqmax : iqmax;    //the voltage limit below sets the voltage; this the current limit
	}else{
		iqRef = -focIqRef;
		if(iqRef>iqmax) iqRef = iqmax; else if(iqRef<-iqmax) iqRef = -iqmax;
	}
	int32_t ed = idRef-id, eq = iqRef-iq;
	if(ed>20000) ed = 20000; else if(ed<-20000) ed = -20000;
	if(eq>20000) eq = 20000; else if(eq<-20000) eq = -20000;
	int32_t kp = KP(), kiTs = KI()*TS_US;
	int32_t kv = cfg(SINE_KV, 242);
	int32_t vff = -fastspeed*5774/kv;    //back emf amplitude, mV: rpm/(sqrt(3)*kv/10); the 6 interval realspeed lags

	int32_t di = kiTs*ed/1000;
	intD += di;
	int32_t vd = kp*ed/1000+intD/1000;
	if(vd>vmax){ if(di>0) intD -= di; vd = vmax; }
	else if(vd<-vmax){ if(di<0) intD -= di; vd = -vmax; }

	int32_t vqmax = MCM_Sqrt(vmax*vmax-vd*vd);
	int32_t qhi = vqmax, qlo = -vqmax;
	if(focVoltMode){    //electrical frame: the command is negated
		int32_t vc = -(int32_t)focVCmd*vmax/1000;
		if(vc>0){ if(vc<qhi) qhi = vc; }else{ if(vc>qlo) qlo = vc; }
	}
	di = kiTs*eq/1000;
	intQ += di;
	int32_t vq = kp*eq/1000+intQ/1000+vff;
	if(vq>qhi){ if(di>0) intQ -= di; vq = qhi; }
	else if(vq<qlo){ if(di<0) intQ -= di; vq = qlo; }

	//field weakening: while q runs into the voltage circle (not a voltage command's own limit), the d
	//current goes negative at 2 mA per period (3 A in ~0.35 s) down to FW_IMAX, and back while q has
	//10 % of the circle left. The negative d current lowers the voltage the back emf needs on q.
	int32_t fwmax = cfgOrDefault(FW_IMAX, 0)*100;
	if(fwmax>imax*7/10) fwmax = imax*7/10;    //keep >= 70 % of the current limit for q (torque)
	if(fwmax){
		if((vq>=vqmax||vq<=-vqmax)&&fwId>-fwmax) fwId -= 2;
		else if(vq<vqmax*9/10&&vq>-vqmax*9/10&&fwId<0) fwId += 2;
		if(fwId<-fwmax) fwId = -fwmax;
	}else{
		fwId = 0;
	}

	if(focTraceN==0xFF){
		int32_t a = iqRef-lastIqRef, b = idRef-lastIdRef;
		if(a>=200||a<=-200||b>=200||b<=-200) focTraceN = 0;
	}
	uint8_t n = focTraceN;
	if(n==0xFE){
		n = focTracePos;
		focTracePos = (uint8_t)((n+1)%FOC_TRACE_N);
	}
	if(n<FOC_TRACE_N){
		volatile int16_t* e = focTrace[n];
		e[0] = (int16_t)id; e[1] = (int16_t)iq; e[2] = (int16_t)vd; e[3] = (int16_t)vq; e[4] = (int16_t)ia; e[5] = th;
		if(focTraceN<FOC_TRACE_N) focTraceN++;
	}
	lastIdRef = idRef; lastIqRef = iqRef;

	ipark1.Ds = vd*5676/v;    //Q15 modulation
	ipark1.Qs = vq*5676/v;
	ipark1.Theta = th;
	IPARK_MACRO1(&ipark1);
	pwm_gen.Mode = SEVENMODE;
	pwm_gen.Alpha = ipark1.Alpha;
	pwm_gen.Beta = ipark1.Beta;
	PWM_GEN_calc(&pwm_gen);
	Update_PWM(&pwm_gen);
}

void FocSetCurrent(int32_t iq){
	int32_t imax = IMAX();
	if(iq>imax) iq = imax; else if(iq<-imax) iq = -imax;
	focIqRef = iq;
	focVoltMode = 0;
}

void FocSetVoltage(int32_t permille){
	if(permille>1000) permille = 1000; else if(permille<-1000) permille = -1000;
	focVCmd = (int16_t)permille;
	focVoltMode = 1;
}

//speed loop: PI on the speed error against a first order reference (like SpeedCtrl), output q current
static int32_t spRef, spI;    //rpm << 8, mA << 8
static uint32_t spT;
static uint8_t spRun;

void FocSpeedReset(void){
	spI = 0;
	spRun = 0;
	spT = millis;
}

int32_t FocSpeedCtrl(int32_t sp, int32_t meas){
	int32_t dt = millis-spT;
	int32_t imax = IMAX()<<8;
	spT = millis;
	if(dt>20) dt = 20;
	if(sp>1000) sp = 1000; else if(sp<-1000) sp = -1000;
	if(meas>1000) meas = 1000; else if(meas<-1000) meas = -1000;
	if(!spRun){    //bumpless from the speed the wheel has
		spRef = meas<<8;
		spRun = 1;
	}
	int32_t d = ((sp<<8)-spRef)*dt/100;    //tref 100 ms
	spRef = d ? spRef+d : sp<<8;
	if(sp==0&&meas==0&&spRef==0){
		//stopped at a 0 rpm set point: the integrator decays (100 ms). Left as it was it pushed ~0.3 A
		//against static friction until the wheel broke loose, then swung +-50 rpm (the speed shows late)
		spI -= spI*dt/100;
	}
	int32_t e = (spRef>>8)-meas;
	int32_t kp = cfg(FOC_SPD_KP, 30), ki = cfg(FOC_SPD_KI, 100);    //mA per rpm, mA per rpm*s
	int32_t di = ki*e*256/1000*dt;
	int32_t u = kp*e*256+spI+di;
	if((u<imax||di<0)&&(u>-imax||di>0)){
		spI += di;
		if(spI>imax) spI = imax; else if(spI<-imax) spI = -imax;
	}
	u = kp*e*256+spI;
	if(u>imax) u = imax; else if(u<-imax) u = -imax;
	return u>>8;
}
