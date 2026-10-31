//Field oriented control on two measured phase currents.
//
//Current sensing (MM32SPIN05PFOP bench board): the chip's two internal op-amps amplify the low side
//current of phases B and C, outputs on PA2 / PA6 (no register setup: they run from reset). An output
//is only valid while the low sides conduct and saturates otherwise (gates off, high side on), so the
//adc samples at the pwm counter's zero (TIM1 update, TRGO: centre of the low side time), the zero current
//offsets are measured with the gates on and the zero vector applied, and the modulation is limited to
//94 % of the linear range (low side time at the sample >= ~3.4 us; the op-amps need 2.5). Measured gain: 19.4 counts/A
//(51.5 mA per count), offset ~1520 counts.
//Received UART traffic (the master transmitting on the RemoteUartBus) couples into the op-amp outputs:
//at 115200 ~5 samples per second read ~170 counts (~9 A) off on one channel, mostly for one period,
//sometimes for 4 in a row; nothing without traffic, a longer adc sample time does not help. A median of
//3 let the 4 period events through (false CURRENT_LIMIT_VIOLATION trips), and comparing with the last
//accepted sample locked onto a borderline glitch (then rejected the true values), and the median of the
//last 9 lagged the phase current's sine by 4 periods: with field weakening (6-8 A at 150 Hz, ~1.7 A per
//period) it rejected the true samples and fed the loop stale ones (I_q swung +-3 A, the wheel shook). A
//straight line through the last two samples ran away on one accepted outlier (made-up 20 A trips).
//Now the filter works on d / q (park()), which are nearly constant at any speed with the smooth tracker
//angle: a period more than 3 A off both of the last two accepted d / q is held at the last, for up to 4
//periods in a row, then taken; after an angle jump it starts again. The phase currents for the trip and
//the bus current come from the filtered d / q, the trip one period later (after the angle). The offset
//calibration (standing, zero vector) keeps a median of 9 per channel. A trip needs two periods in a row.
//
//Loop: every control period (every pwmDiv-th pwm period, ~4.39 kHz) in the adc isr. Park angle = the hall tracker's angle at the sample
//(hallAngleSample; HALL1.Angle has the output lead); PI per axis in mV with back emf feed
//forward on q and the dq decoupling (omega*L*i, L 0.46 mH: at 170 Hz electrical omega*L ~ 0.5 Ohm, as
//large as kp); on the voltage circle d first, q gets the rest, integration stops in saturation.
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

//modulation limits from the low side time the op-amps need either side of the sample (2.5 us): FOC 94 % at
//8192 (+-3.4 us; 4.39 kHz), sine's valid phase currents to 95.3 % (+-2.73 us). The low side half time is
//(1 - m) T / 4: at 4096 (8.8 kHz) 88 / 90.4 %, at 2048 (17.6 kHz) 76 / 81 % (FocInit())
static int32_t modMax = 30802, sampleMax = 31200;
#define MOD_MAX modMax
#define SAMPLE_MAX sampleMax
#define FW_MA_PER_RPM 8                 //field weakening d current limit per rpm (below)
#define INV_SQRT3_Q15 18919

static int32_t cfg(uint16_t v, int32_t def){ return cfgOrDefault(v, def); }    //0 and 65535 (an unset word) = default
#define GAIN()   cfg(IPHASE_GAIN, 515)    //0.1 mA per adc count
static __attribute__((noinline)) int32_t cfgMax(uint16_t v, int32_t def, int32_t max){ int32_t x = cfgOrDefault(v, def); return x>max ? max : x; }
#define IMAX_CFG() (cfgMax(FOC_IMAX, 100, 1000)*100) //mA, current_soft_max as set (<= 100 A)
#define IMAX()   CurrentImax()              //mA, the limit in use (below the pullback limit)
#define KP()     cfg(CUR_KP, 400)         //mV per A
#define KI()     cfgMax(CUR_KI, 300, 3000)    //V per A*s (32 bit integrator steps up to ~3700)

static volatile u32* adc1;    //result registers of the two current pins
static volatile u32* adc2;
static int32_t off1, off2;         //zero current, adc counts << 4
static int32_t calSum1, calSum2;
static uint8_t calN, offValid, armedPeriods, tripN;
#define GLITCH 60                  //adc counts (~3 A)
#define HIST 9
static int16_t hist1[HIST], hist2[HIST];    //raw samples per channel, ring
static uint8_t histPos;
static int32_t lastD, lastQ, prevD, prevQ;    //the last two accepted d / q currents, mA (glitch filter)
static int16_t lastThS;
static uint8_t dqOk, rejDq;
#define GLITCH_MA 3000                //mA
#define ANGLE_JUMP 2731               //15 deg el: the filter starts again
volatile uint16_t focGlitches;     //samples replaced by the median, diagnostic
static int32_t ia, ib, ic;         //mA, this period
static int32_t intD, intQ;         //integrators, uV
static int32_t fwId;               //field weakening d current, mA (<= 0)
volatile int32_t focIdNow;         //the d current reference applied, mA (electrical frame)

volatile int32_t focIdRef;
volatile int16_t focId, focIq;
volatile int16_t focIqF, focIdF;   //focIq, focId averaged over ~32 periods (7 ms), telemetry
static int32_t iqAcc, idAcc;
static uint8_t sampleOk;           //this period's phase currents are valid
static int32_t itotalPhase;        //the last bus current from them (sine above MOD_MAX keeps it)
volatile uint8_t focTrip;
static uint8_t focOver, overN;    //phase current above the soft limit (3/4 of the trip), 2 periods
volatile uint16_t focFoldbacks;    //periods the soft limit pulled the current back (diagnostic, no error)
static volatile int32_t focIqRef;  //mA, motor frame
static volatile int16_t focVCmd;   //permille, motor frame; voltage mode if nonzero
static volatile uint8_t focVoltMode;

static volatile uint8_t regenLim;    //braking current held back by the regeneration limit (this period)
static volatile int8_t qSat;    //q voltage on its limit: +1 towards positive focIqRef (motor frame), -1 negative

//Position hold (FOC position mode, near the target): the frame is not the hall angle but the target's
//electrical angle, with a d current (hold_current, default 4 A, at most 90 % of the current limit): the rotor is pulled there like a stepper,
//to a fraction of a hall step (1 deg mechanical = 15 deg electrical; the hall sensors cannot tell where
//the rotor is between two edges). The q axis is voltage driven (R * load current, below): a short
//circuit for the back emf, it damps the rotor (with the q current regulated the rotor rang on the
//magnetic spring). The angle moves to the
//target at <= HOLD_RATE per period. A hall state more than 90 deg el from the held angle means the rotor
//slipped (load above the hold torque): focHoldSlip, the position loop takes over again.
//A load (a weight) needs torque, not only the spring: the hold starts with the speed loop's integral as a
//q voltage feed forward (R * I, MOTOR_R: the short circuit damping stays), adds to it while the hall
//state shows the rotor >= 15 deg el (1 deg mechanical) behind the held angle, and gives it back to the
//speed loop's integrator when it ends. (With the old dead band the integrator decayed at the target: the
//load sagged until the next hall step, then the loop caught it again.)
#define HOLD_RATE 2                 //1/POS_FINE step per period: ~34 steps/s
volatile uint8_t focHold, focHoldSlip;
volatile int32_t focHoldPos;        //held position, 1/POS_FINE step
static volatile int32_t holdTarget;
static int32_t holdIq;              //load current, mA, electrical frame
static int32_t spRef, spI;    //speed loop (below): rpm << 8, mA << 8
static uint32_t spT;
static uint8_t spRun;
static SpeedDeriv spD;    //velocity D

static uint16_t holdA0;             //electrical angle of position 0 (mod 6 steps)
static int32_t holdI;               //smart hold (HOLD_IDLE): the d current now, mA
static uint32_t holdT;              //smart hold: ms of the last displacement (or the start)
static int16_t holdAngle(int32_t p){    //electrical angle of a position: one electrical turn = 6 steps
	int32_t m = p%(6*POS_FINE); if(m<0) m += 6*POS_FINE;
	return (int16_t)(holdA0-(uint32_t)(m*65536/(6*POS_FINE)));
}

void FocHoldStart(int32_t target){
	__disable_irq();
	int32_t m = iOdom%6; if(m<0) m += 6;    //(iOdom*POS_FINE wrapped after 2^23 steps: 2^32 is no multiple of 6)
	m *= POS_FINE;
	holdA0 = (uint16_t)(HALL1.CWAngleTab[HALL1.PreRaw]+(uint32_t)(m*65536/(6*POS_FINE)));    //centre of this sector
	focHoldPos = hallPosFine;
	holdTarget = target;
	holdIq = -spI/256;    //the speed loop's integral (the steady load), not its output: that was the
	                      //braking current of the approach (5.6 A into a standing wheel)
	focHoldSlip = 0;
	intD = intQ = 0;
	holdI = 1000000;    //the full hold current first (clamped below)
	holdT = millis;
	focHold = 1;
	__enable_irq();
}

void FocHoldTarget(int32_t target){
	holdTarget = target;
}

void FocHoldStop(void){
	if(!focHold) return;
	__disable_irq();
	focHold = 0;
	intD = intQ = 0;
	spI = -holdIq*256;    //the load current back to the speed loop (motor frame)
	spRun = 0;           //its reference from the measured speed
	__enable_irq();
}

//capture for tuning and fault analysis over SWD (make TRACE=1; 768 bytes of RAM), per pwm period: id,
//iq (mA), vd, vq (mV, electrical frame), ia (mA), angle. focTraceN: write 0xFF = one shot from the next
//reference step of >= 200 mA; 0xFE = ring buffer until a trip (then focTraceN = FOC_TRACE_N, the oldest
//entry at focTracePos)
#ifdef FOC_TRACE
#define FOC_TRACE_N 64
volatile int16_t focTrace[FOC_TRACE_N][6];
volatile uint8_t focTraceN = FOC_TRACE_N;
volatile uint8_t focTracePos;
volatile uint8_t focTraceDiv = 1;    //record every n-th period
static int32_t lastIdRef, lastIqRef;
static void trace(int32_t id, int32_t iq, int32_t vd, int32_t vq, int16_t th, int32_t idRef, int32_t iqRef){
	if(focTraceN==0xFF){
		int32_t a = iqRef-lastIqRef, b = idRef-lastIdRef;
		if(a>=200||a<=-200||b>=200||b<=-200) focTraceN = 0;
	}
	static uint8_t decim;    //focTraceDiv: record every n-th period (a longer window)
	uint8_t n = ++decim>=focTraceDiv ? focTraceN : 0xFD;    //0xFD: not this period
	if(n!=0xFD) decim = 0;
	if(n==0xFE){
		n = focTracePos;
		focTracePos = (uint8_t)((n+1)%FOC_TRACE_N);
	}
	if(n<FOC_TRACE_N){
		volatile int16_t* e = focTrace[n];
		e[0] = (int16_t)id; e[1] = (int16_t)iq; e[2] = (int16_t)vd; e[3] = (int16_t)vq;
		e[4] = (int16_t)ia; e[5] = th;
		if(focTraceN<FOC_TRACE_N) focTraceN++;
	}
	lastIdRef = idRef; lastIqRef = iqRef;
}
#define TRACE_FREEZE() do{ if(focTraceN==0xFE) focTraceN = FOC_TRACE_N; }while(0)    //freeze the ring at a trip
#else
#define trace(id, iq, vd, vq, th, idRef, iqRef) ((void)0)
#define TRACE_FREEZE() ((void)0)
#endif
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
	int32_t t = cfgMax(CUR_TRIP, 0, 2000)*100;    //<= 200 A: the phase currents are 32 bit safe to ~260 A (park())
	if(!t){ t = IMAX_CFG()*2; if(t<15000) t = 15000; }    //default: twice the limit, at least 15 A
	return t;
}

//the three phase current limits stay in order whatever the configuration: current_soft_max (the loops' limit)
//< pullback (above it the current loop pulls the current back, no error; park()) < current_hard_max (the
//trip: the current rose anyway, a loss of control)
int32_t CurrentPullback(void){    //mA
	int32_t h = CurrentHardMax(), p = cfgOrDefault(CUR_PULLBACK, 0)*100;
	if(!p) p = h*3/4;
	if(p>h*9/10) p = h*9/10;
	return p;
}
int32_t CurrentImax(void){    //mA
	int32_t i = IMAX_CFG(), c = CurrentPullback()*9/10;
	return (i<c ? i : c)*derate>>10;    //temperature derating (TEMP_DERATE)
}

uint8_t FocSensing(void){    //phase currents measured (FOC, or sine on a board with the current pins)
	return adc1&&adc2&&offValid&&COMMUTATION(DRIVEMODE)!=COMMUT_BLOCK;
}

uint8_t FocAvailable(void){
	return adcResultReg(IPHASEAPIN)&&adcResultReg(IPHASEBPIN);
}

void FocInit(void){
	modMax = 32768-446956L*36/(int32_t)TIM1->ARR;      //32768 * 4 * 3.41 us / T, T = ARR/36 us
	sampleMax = 32768-357826L*36/(int32_t)TIM1->ARR;   //2.73 us
	adc1 = adcResultReg(IPHASEAPIN);
	adc2 = adcResultReg(IPHASEBPIN);
	FocReset();
}

void FocReset(void){
	focHold = 0;
	intD = intQ = 0;
	fwId = 0;
	focIqRef = 0;
	focVCmd = 0;
	FocSpeedReset();
}

int32_t FocFieldWeakening(void){    //mA, <= 0
	return fwId;
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
	sampleOk = 0;
	uint8_t sine = COMMUTATION(DRIVEMODE)==COMMUT_SINE;
	if(armedPeriods<2){    //outputs saturated, or the first period after arming
		for(uint8_t k=0;k<HIST;k++){ hist1[k] = (int16_t)r1; hist2[k] = (int16_t)r2; }
		if(offValid) itotal = itotalPhase = 0;    //gates off: no bus current (the shunt reads noise)
		return;
	}
	if(armedPeriods<=2) dqOk = 0;    //the dq filter starts again (park())
	if(armedPeriods==2){ for(uint8_t k=0;k<HIST;k++){ hist1[k] = (int16_t)r1; hist2[k] = (int16_t)r2; } }
	if(!offValid){    //FocControl() applies the zero vector until the offsets are known
		hist1[histPos] = (int16_t)r1; hist2[histPos] = (int16_t)r2;
		histPos = (uint8_t)((histPos+1)%HIST);
		int32_t m1 = median9(hist1), m2 = median9(hist2);
		if(r1-m1>GLITCH||m1-r1>GLITCH) r1 = m1;
		if(r2-m2>GLITCH||m2-r2>GLITCH) r2 = m2;
		if(armedPeriods>=4&&realspeed==0&&(!sine||pwm==0)){    //sine: its zero vector at command 0
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
	//sine above that: the low side time is too short for a valid sample. (It was MOD_MAX: at the sine's
	//top speed (95 %) the bus current and I_d / I_q read the value from the acceleration, 2.5 A / 3.5 A
	//instead of 0.4 / 0.7.) itotal stays the shunt's, sampled in the active vectors (interrupt.c busv; it
	//was itotalPhase, frozen at its last valid value from ~10 turns/s with field advance up). Nor in a regen
	//short (bldc.c RegenShort): the low sides conduct for its duty only, at 4 % for ~1 us around the counter's
	//zero, the samples read the saturated amplifiers (F031: CURRENT_LIMIT_VIOLATION in every sine braking)
	if(sine&&(regenShort||pwm>SAMPLE_MAX||pwm<-SAMPLE_MAX)){
		dqOk = 0;
		return;
	}
	itotal = itotalPhase;    //from the previous period's filtered currents (park())
	sampleOk = 1;
}

//d and q current (electrical frame) of this period's sample, after HALLModuleCalc(): the tracker's angle
//at the sample (the output angle has a lead for the pwm)
static void park(int32_t* id, int32_t* iq, int16_t thS){
	Trig_Components sc = MCM_Trig_Functions(thS);
	//products on mA/4: 32 bit to ~260 A (in mA they overflowed from 65 A, a railed adc reads ~130 A)
	int32_t alpha = ia, beta = ((ib-ic)>>2)*INV_SQRT3_Q15>>13;
	int32_t d = ((alpha>>2)*sc.hCos>>13)+((beta>>2)*sc.hSin>>13);
	int32_t q = ((beta>>2)*sc.hCos>>13)-((alpha>>2)*sc.hSin>>13);
	int32_t trip = CurrentHardMax(), soft = CurrentPullback();    //soft: the loop pulls the current back (FocControl)
	int32_t pk = ia<0 ? -ia : ia, t2 = ib<0 ? -ib : ib, t3 = ic<0 ? -ic : ic;
	if(t2>pk) pk = t2;
	if(t3>pk) pk = t3;
	//glitch filter (see the top): a period more than 3 A off both of the last two accepted d / q (nearly
	//constant at any speed) is held at the last, for up to 4 periods; it starts again after an angle jump
	//(start, reversal, sector centre at low speed). Against the last accepted sample alone, a glitch just
	//under the threshold was taken and then the true samples were rejected as glitches for 4 periods: 1 ms of
	//a 2.7 A wrong q current, the loop drove the real one 1.5 A off, a bump at speed (trace, 8 turns/s, 2-4
	//per 10 s with bus traffic); the sample before it still takes them. (A median of the last 3 fixed that
	//but lagged the current loop's own fast changes: after a 31 deg tracker snap at 9 A of acceleration the
	//d current fell 2 A per period, was held at its old value as a glitch, the loop drove 12 A, a 30 A trip.)
	int16_t dth = (int16_t)(thS-lastThS-HALL1.MoveDir*HALL1.IncAngle);
	lastThS = thS;
	uint8_t restart = !dqOk||dth>ANGLE_JUMP||dth<-ANGLE_JUMP;
	uint8_t offLast = d-lastD>GLITCH_MA||lastD-d>GLITCH_MA||q-lastQ>GLITCH_MA||lastQ-q>GLITCH_MA;
	uint8_t offPrev = d-prevD>GLITCH_MA||prevD-d>GLITCH_MA||q-prevQ>GLITCH_MA||prevQ-q>GLITCH_MA;
	if(restart){
		dqOk = 1; rejDq = 0;
	}else if(offLast&&offPrev&&rejDq<4&&pk<soft){    //never above the soft limit: a real runaway stays visible
		d = lastD; q = lastQ; rejDq++; focGlitches++;
		alpha = ((d>>2)*sc.hCos>>13)-((q>>2)*sc.hSin>>13);    //phase currents back (inverse park, Clarke)
		beta = ((d>>2)*sc.hSin>>13)+((q>>2)*sc.hCos>>13);
		ia = alpha; ib = (-alpha+((beta>>2)*56756>>13))/2; ic = -ia-ib;    //56756 = sqrt(3) Q15
	}else{
		if(rejDq>=4) restart = 1;    //still off after 4: a real step, taken
		rejDq = 0;
	}
	if(restart){ prevD = lastD = d; prevQ = lastQ = q; }
	else if(!rejDq){ prevD = lastD; prevQ = lastQ; lastD = d; lastQ = q; }
	*id = d; *iq = q;
	//above the pullback limit two periods in a row: FocControl pulls the current back (no error); still rising
	//to the trip level (CurrentHardMax) means the loop lost control: the gates go off, CURRENT_LIMIT_VIOLATION
	pk = ia<0 ? -ia : ia; t2 = ib<0 ? -ib : ib; t3 = ic<0 ? -ic : ic;
	if(t2>pk) pk = t2;
	if(t3>pk) pk = t3;
	if(pk>soft){ if(overN<2) overN++; }else overN = 0;
	focOver = overN>=2;
	if(pk>trip){
		if(++tripN>=2){
			focTrip = 1;
			focTripI[0] = (int16_t)ia; focTripI[1] = (int16_t)ib; focTripI[2] = (int16_t)ic;
			TRACE_FREEZE();
		}
	}else{
		tripN = 0;
	}
	//bus current = sum of duty * phase current; the CCRs read back are the ones active in this period
	int32_t n = (int32_t)PWM_PERIOD;
	itotalPhase = ((n-(int32_t)TIM1->CCR1)*ia+(n-(int32_t)TIM1->CCR2)*ib+(n-(int32_t)TIM1->CCR3)*ic)/n/10;
	focId = (int16_t)*id;
	focIq = (int16_t)-*iq;
	iqAcc += focIq-(iqAcc>>5);
	focIqF = (int16_t)(iqAcc>>5);
	idAcc += focId-(idAcc>>5);
	focIdF = (int16_t)(idAcc>>5);
}

void FocObserve(void){    //sine: I_d / I_q for telemetry
	int32_t id, iq;
	if(sampleOk) park(&id, &iq, hallAngleSample);
}

static void zeroVector(void){
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_PERIOD/2;
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
	if(v>7000) v = 7000;                    //MOD_MAX*v in 32 bits (a glitch once overflowed it negative)
	int32_t vmax = MOD_MAX*v/5676;          //mV; phase amplitude Vbus/sqrt(3) at modulation 32768
	int16_t th = HALL1.Angle;
	int32_t id, iq;
	if(focHold){
		int32_t d = holdTarget-focHoldPos;
		focHoldPos += d>HOLD_RATE ? HOLD_RATE : (d<-HOLD_RATE ? -HOLD_RATE : d);
		th = holdAngle(focHoldPos);
		park(&id, &iq, th);
		int16_t dr = (int16_t)(th-(int16_t)HALL1.CWAngleTab[HALL1.PreRaw]);
		if(dr>16384||dr<-16384) focHoldSlip = 1;
		int32_t imax = IMAX();
		if(dr>8192&&holdIq<imax) holdIq++;    //rotor behind (lower angle): + q turns the angle up; 4.4 A/s
		else if(dr<-8192&&holdIq>-imax) holdIq--;
		int32_t ih = cfg(HOLD_CURRENT, 40)*100, ihm = IMAX()*9/10; if(ih>ihm) ih = ihm;    //hold_current
		//smart hold: the full current holds ~7 W for as long as the wheel stands. In the target's hall sector
		//(|dr| <= 30 deg el) for HOLD_IDLE_DELAY it falls to HOLD_IDLE (1 mA per control period, ~4.4 A/s);
		//pushed off, it rises at once with the displacement: HOLD_IDLE at half a step (30 deg el), the full
		//current at 1.5 steps (90 deg, where the hold counts as slipped and the position loop takes over).
		//A steady load is carried by holdIq (q) either way.
		int32_t idle = cfg(HOLD_IDLE, 0)*100;
		if(idle&&idle<ih){
			int32_t a = dr<0 ? -dr : dr, want = ih;
			if(a>5461){ want = idle+(ih-idle)*(a-5461)/(16384-5461); holdT = millis; }
			else if(millis-holdT>=(uint32_t)cfg(HOLD_IDLE_DELAY, 500)) want = idle;
			if(want>ih) want = ih;
			if(holdI>ih) holdI = ih;
			holdI = want>=holdI ? want : holdI-1;
			ih = holdI;
		}
		if(focOver){ ih = 0; intD = 0; focFoldbacks++; }    //soft limit (see park())
		int32_t kp = KP(), ed = ih-id;
		if(ed>20000) ed = 20000; else if(ed<-20000) ed = -20000;    //as below: 32 bit products
		intD += KI()*(((int32_t)CTRL_ARR+18)/36)/8*ed/125;
		int32_t vd = kp*ed/1000+intD/1000;
		if(vd>vmax){ intD -= (vd-vmax)*1000; vd = vmax; } else if(vd<-vmax){ intD -= (vd+vmax)*1000; vd = -vmax; }
		intQ = 0; fwId = 0; qSat = 0;
		focIdNow = ih;
		int32_t vq = holdIq*cfgMax(MOTOR_R, 300, 10000)/1000;    //(<= 10 Ohm: 32 bit)    //mV (cfg() read 65535 as 65 Ohm until it took 65535 as default)
		if(vq>vmax/2) vq = vmax/2; else if(vq<-vmax/2) vq = -vmax/2;
		ipark1.Ds = vd*5676/v; ipark1.Qs = vq*5676/v; ipark1.Theta = th;
		IPARK_MACRO1(&ipark1);
		pwm_gen.Mode = SEVENMODE; pwm_gen.Alpha = ipark1.Alpha; pwm_gen.Beta = ipark1.Beta;
		PWM_GEN_calc(&pwm_gen); Update_PWM(&pwm_gen);
		return;
	}
	park(&id, &iq, hallAngleSample);

	//references in the electrical frame: + q turns the angle up = negative realspeed
	//d current (field weakening, focIdRef) only while the angle is known: between hall edges the angle is
	//extrapolated at the last speed, and when the rotor slows it runs ahead (up to a sector, 60 deg). A
	//negative d current in a frame ahead of the rotor brakes it (|id| * sin(error)): it slowed further,
	//the error grew, the wheel stalled and rocked over a hall boundary (17-28 ms without an edge at
	//120 rpm with id -6 A; field weakening at speed shook the same way from 4 A on).
	int32_t imax = IMAX();
	int32_t idRef = hallAngleSure ? focIdRef+fwId : 0, iqRef;
	if(idRef>imax) idRef = imax; else if(idRef<-imax) idRef = -imax;
	focIdNow = idRef;
	int32_t iqmax = fwId ? MCM_Sqrt((imax/10)*(imax/10)-(idRef/10)*(idRef/10))*10 : imax;    //the total current stays within the limit (10 mA: 32 bit)
	if(focVoltMode){
		iqRef = focVCmd>0 ? -iqmax : iqmax;    //the voltage limit below sets the voltage; this the current limit
	}else{
		iqRef = -focIqRef;
		if(iqRef>iqmax) iqRef = iqmax; else if(iqRef<-iqmax) iqRef = -iqmax;
		//regeneration limit, every period on the bus voltage now: the braking current (electrical frame:
		//iqRef with the sign of realspeed) falls from the full limit at BAT_FULL to 0 at BAT_FULL + 1 V.
		//In the main loop on the 7 ms average it came late (900 rpm pumped a bench supply to 34 V). It was
		//on/off: the wheel coasted past a position target, the speed loop wound up meanwhile and fired
		//-13.5 A of braking once the bus fell back: the bus jumped to ~60 V and the board lost power
		//(F031 bench). regenLim stops the speed loop's integrator in the braking direction.
		//Above BAT_FULL the braking power is burnt in the motor first: negative d current (no torque, only
		//copper loss) up to the current limit at BAT_FULL + 0.5 V; only then the braking current falls, to 0
		//at + 1.5 V. A bench supply cannot sink and the bus capacitors take ~0.1 J per 3 V (a wheel at
		//3 turns/s carries ~2 J): with the braking cut the wheel coasted past position targets (F031: a half
		//turn overshot 9-16 steps, the bus at the limit, then CURRENT_LIMIT_VIOLATION on the way back).
		regenLim = 0;
		if(BAT_SET(BAT_FULL)&&vbat*10>(int32_t)BAT_FULL&&((iqRef>0&&realspeed>0)||(iqRef<0&&realspeed<0))){
			int32_t over = vbat*10-(int32_t)BAT_FULL;
			int32_t dump = imax*(over>500 ? 500 : over)/500;
			if(idRef>-dump) idRef = -dump;
			focIdNow = idRef;
			int32_t lim = MCM_Sqrt((imax/10)*(imax/10)-(idRef/10)*(idRef/10))*10;
			if(over>500) lim = lim*(1500-over)/1000;
			if(lim<0) lim = 0;
			if(iqRef>lim){ iqRef = lim; regenLim = 1; }
			else if(iqRef<-lim){ iqRef = -lim; regenLim = 1; }
		}
	}
	if(focOver){    //soft limit: no current asked this period, the integrators start again: kp drives it down
		idRef = iqRef = 0; intD = intQ = 0; fwId = 0;
		focFoldbacks++;
	}
	int32_t ed = idRef-id, eq = iqRef-iq;
	if(ed>20000) ed = 20000; else if(ed<-20000) ed = -20000;
	if(eq>20000) eq = 20000; else if(eq<-20000) eq = -20000;
	int32_t kp = KP(), kiTs8 = KI()*(((int32_t)CTRL_ARR+18)/36)/8;    //Ts in us: the control period; /8: kiTs*ed in 32 bit
	int32_t kv = cfg(SINE_KV, 242);
	//back emf feed forward and decoupling from realspeed (6 hall intervals), not fastspeed (the last one):
	//the hall intervals jitter with the phase current (+-3 % at no load, +-20 % with field weakening at
	//full current). Fed from the last interval, the jitter became +-2.8 V on q, the current followed and
	//jittered the next intervals: a 68 Hz oscillation of +-10 A that shook the wheel and tripped. The
	//average lags in a fast acceleration; the PI takes the difference.
	int32_t spdFF = realspeed;
	int32_t vff = -spdFF*5774/kv;    //back emf amplitude, mV: rpm/(sqrt(3)*kv/10)
	//omega*L in mOhm, signed like fastspeed (the electrical frame turns the other way): rpm * pole pairs
	//* 2 pi / 60 * 0.46 mH
	int32_t wL = spdFF*(int32_t)WINDINGS/2*(cfg(PHASE_L, 460)*10472/10000)/10000;    //phase_inductance, uH
	int32_t vdDec = wL*iqRef/1000, vqDec = -wL*idRef/1000;    //mV

	int32_t di = kiTs8*ed/125, diq = kiTs8*eq/125;
	intD += di; intQ += diq;
	int32_t vd = kp*ed/1000+intD/1000+vdDec;
	int32_t vq = kp*eq/1000+intQ/1000+vff+vqDec;
	//magnitude the current loops ask for, before the limit (field weakening below); not in voltage mode
	if(vd>30000) vd = 30000; else if(vd<-30000) vd = -30000;
	if(vq>30000) vq = 30000; else if(vq<-30000) vq = -30000;
	int32_t vmag = focVoltMode ? 0 : MCM_Sqrt(vd*vd+vq*vq);
	//voltage circle: d first, q gets the rest, integration stops in saturation (classic field weakening:
	//the d current must get its voltage). It shook from 4 A of d current until the hall angle lead
	//(9/8 periods), the stale current samples (glitch filter) and the noisy back emf feed forward were
	//fixed; a scaled vector (tried meanwhile) starved d: the d current stayed at -6 of -9 A.
	if(vd>vmax){ if(di>0) intD -= di; vd = vmax; }
	else if(vd<-vmax){ if(di<0) intD -= di; vd = -vmax; }
	int32_t vqmax = MCM_Sqrt(vmax*vmax-vd*vd);
	int32_t qhi = vqmax, qlo = -vqmax;
	if(focVoltMode){
		int32_t vc = -(int32_t)focVCmd*vmax/1000;    //electrical frame: the command is negated
		if(vc>0){ if(vc<qhi) qhi = vc; }else{ if(vc>qlo) qlo = vc; }
		//regeneration limit (as the current modes above): a q voltage on the far side of the back emf
		//brakes (the loop drives to qlo for a positive command, to qhi for a negative one); with the bus
		//above BAT_FULL both bounds stay on the motoring side of it (no braking current)
		if(BAT_SET(BAT_FULL)&&vbat*10>(int32_t)BAT_FULL){
			int32_t e = vff<-vqmax ? -vqmax : (vff>vqmax ? vqmax : vff);
			if(realspeed>0){ if(qhi>e) qhi = e; if(qlo>e) qlo = e; }
			else if(realspeed<0){ if(qhi<e) qhi = e; if(qlo<e) qlo = e; }
		}
	}
	if(vq>qhi){ if(diq>0) intQ -= diq; vq = qhi; qSat = -1; }    //electrical frame: q = -focIqRef
	else if(vq<qlo){ if(diq<0) intQ -= diq; vq = qlo; qSat = 1; }
	else qSat = 0;

	//field weakening: an integrator sets the d current (0 .. -FW_IMAX) so that the voltage the current
	//loops ask for stays at 95 % of the circle: they keep headroom and stay in control; the negative d
	//current lowers the voltage the back emf needs on q (omega*L*id, ~0.4 V per A at 550 rpm here).
	//1/64 mA per mV and period: ~35 ms. It was on/off (2 mA per period while q touched the circle):
	//that needed the loops saturated to work and kept them there; with a large d current the wheel
	//shook and the speed swung +-100 rpm.
	int32_t fwmax = cfgOrDefault(FW_IMAX, 0)*100;
	//at most 90 % of the current limit (the q current keeps >= 44 %). Bench, 24 V, current_soft_max
	//15 A, measured d current = reference: 520 rpm without, 626 rpm with 5 A, 745 with 9 A, 836 with
	//13 A (12.7 A in total, 4 A from the supply), 0.9-1.2 m/s^2 throughout. Each A of d current saves
	//omega*L (~0.37 V at 600 rpm). (An earlier "815 rpm with 6 A" was the median glitch filter feeding
	//stale currents: the real d current was larger and uncontrolled, ~180 W from the supply.)
	if(fwmax>imax*9/10) fwmax = imax*9/10;
	//and at most FW_MA_PER_RPM per rpm: a negative d current in a frame off the rotor pushes it further off,
	//and the hall frame follows the rotor better the faster it turns (more edges per second). Forced d
	//current, bench, 36 V (no field weakening needed): -4.5 A shook at 300 rpm (3.1 m/s^2), not at 450-750;
	//-6 A shook at 600 rpm (3.9 m/s^2, +-18 rpm), not at 750; 0 A nowhere (0.55-0.97, 300-800 rpm). With
	//9 A of field weakening at 24 V, 680 rpm shook at 2.8 m/s^2.
	{
		int32_t a = realspeed<0 ? -realspeed : realspeed;
		if(fwmax>a*FW_MA_PER_RPM) fwmax = a*FW_MA_PER_RPM;
	}
	if(fwmax&&vmag){
		fwId -= (vmag-vmax*95/100)/64;
		if(fwId<-fwmax) fwId = -fwmax; else if(fwId>0) fwId = 0;
	}else{
		fwId = 0;
	}

	trace(id, iq, vd, vq, th, idRef, iqRef);

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

void FocSpeedReset(void){
	spI = 0;
	spRun = 0;
	spT = millis;
}

int32_t FocSpeedCtrl(int32_t sp, int32_t meas, uint8_t filt){
	int32_t dt = millis-spT;
	int32_t imax = IMAX()<<8;
	spT = millis;
	if(dt>20) dt = 20;
	if(sp>1000) sp = 1000; else if(sp<-1000) sp = -1000;
	if(meas>1000) meas = 1000; else if(meas<-1000) meas = -1000;
	if(!spRun){    //bumpless from the speed the wheel has
		spRef = meas<<8;
		spRun = 1;
		spD.last = meas; spD.acc = 0;
	}
	int32_t d = ((sp<<8)-spRef)*dt/100;    //tref 100 ms (not in position mode: a lag inside its loop)
	spRef = filt ? (d ? spRef+d : sp<<8) : sp<<8;
	if(sp==0&&meas==0&&spRef==0){
		//stopped at a 0 rpm set point: the integrator decays (100 ms). Left as it was it pushed ~0.3 A
		//against static friction until the wheel broke loose, then swung +-50 rpm (the speed shows late).
		//Not above 1 A: that is a load (a slope, a weight), decayed it sagged and was caught again.
		if(spI<(1000<<8)&&spI>-(1000<<8)) spI -= spI*dt/100;
	}
	int32_t e = (spRef>>8)-meas;
	//mA per rpm, mA per rpm*s, default 30 / 100 (kp 50 limit-cycled at the stop, and the gain multiplies
	//the hall interval noise into torque). Position mode: 10/3 of that below 60 rpm, falling to 1x at 200
	//rpm. Near the target it has to follow the braking curve (at 30 mA/rpm 3.3 A of braking took ~110 rpm
	//of error: half a turn overshot 4-5 steps; 3.3x: 0-3, 0.21-0.34 s; 5 steps 0.05 s instead of 0.14-
	//0.18), and the hold takes over before the stop. At cruise speed 3.3x was unstable (+-10 A every
	//sample at 470 rpm, the bus pumped to 38 V, a trip).
	int32_t kp = cfgDef(FOC_SPD_KP, 30), ki = cfgDef(FOC_SPD_KI, 100);
	if(!filt){
		int32_t am = meas<0 ? -meas : meas, g = am<=60 ? 100 : (am>=200 ? 30 : 100-(am-60)/2);    //percent*3
		kp = kp*g/30; ki = ki*g/30;
	}
	if(kp>1000) kp = 1000;    //kp*e*256 in 32 bit (e <= 2000 rpm)
	if(ki>1000) ki = 1000;
	int32_t di = ki*e*256/1000*dt;
	int32_t u = kp*e*256+spI+di;
	//no integration into a current the voltage cannot drive (q voltage on its limit): at top speed the
	//integrator ran up to the current limit against an unreachable set point and took ~1.7 s to come
	//back when the set point dropped
	//nor into braking the regeneration limit holds back (braking: against the measured speed)
	uint8_t brakeHeld = regenLim&&((di>0&&meas<0)||(di<0&&meas>0));
	if((u<imax||di<0)&&(u>-imax||di>0)&&!(qSat>0&&di>0)&&!(qSat<0&&di<0)&&!brakeHeld){
		spI += di;
		if(spI>imax) spI = imax; else if(spI<-imax) spI = -imax;
	}
	u = kp*e*256+spI;
	int32_t kd = cfgDef(FOC_SPD_KD, 0);
	if(kd){    //D on the measured speed: 0.01 mA per rpm/s
		int32_t dd = kd*SpeedDerivStep(&spD, meas, dt)/100, im = imax>>8;
		if(dd>im) dd = im; else if(dd<-im) dd = -im;
		u -= dd<<8;
	}
	if(u>imax) u = imax; else if(u<-imax) u = -imax;
	return u>>8;
}
