//PinFinder calibrations: hall edge angles, pole count, phase resistance and inductance, the speed constants
//of block and sine commutation. Each sets the config words the main firmware uses (in RAM: save keeps them)
//and prints them under their config names.
#include "pf.h"
#include "main.h"

static const uint8_t hallSeq[8] = { 7, 0, 2, 1, 4, 5, 3, 7 };    //hall state -> place in the order of rising steps

static uint8_t hallsSet(void){
	if(HALLAPIN<PINCOUNT&&HALLBPIN<PINCOUNT&&HALLCPIN<PINCOUNT){
		pinMode(HALLAPIN, INPUT); pinMode(HALLBPIN, INPUT); pinMode(HALLCPIN, INPUT);
		return 1;
	}
	err("hall pins not set: detect hall first");
	return 0;
}

static int32_t isqrt(uint32_t v){
	uint32_t r = 0, b = 1u<<30;
	while(b>v) b >>= 2;
	while(b){
		if(v>=r+b){ v -= r+b; r = (r>>1)+b; } else r >>= 1;
		b >>= 2;
	}
	return (int32_t)r;
}

//least squares y = a x + b
static void fit(const float* x, const float* y, uint8_t n, float* a, float* b){
	float sx = 0, sy = 0, sxx = 0, sxy = 0;
	for(uint8_t i=0;i<n;i++){ sx += x[i]; sy += y[i]; sxx += x[i]*x[i]; sxy += x[i]*y[i]; }
	float d = (float)n*sxx-sx*sx;
	*a = d!=0.0f ? ((float)n*sxy-sx*sy)/d : 0.0f;
	*b = (sy-*a*sx)/(float)n;
}

//hall edges with direction (forward = the order of rising steps) between calls
static int32_t edgeCount;
static uint8_t edgePrev;
static void edgePoll(void){
	uint8_t h = HallState();
	if(h!=edgePrev&&hallSeq[h]<6&&hallSeq[edgePrev]<6){
		uint8_t d = (uint8_t)((hallSeq[h]+6-hallSeq[edgePrev])%6);
		if(d==1) edgeCount++; else if(d==5) edgeCount--;
	}
	edgePrev = h;
}

//---------------------------------------------------------------- hall edge angles
//The field turns 2 electrical turns forward and back in 1 deg steps (10 ms each); the rotor follows with
//some lag, so an edge is seen at a larger field angle forward and a smaller one backward: the edge is the
//mean of both (as hallcal.py did over SWD). HALL_ANG(h) = the edge where state h starts with rising angle.
void CalHall(int32_t mv){
	if(!hallsSet()) return;
	if(mv<=0) mv = 1000;
	int32_t sum[2][8], ref[8];
	uint8_t n[2][8];
	for(uint8_t h=0;h<8;h++){ sum[0][h] = sum[1][h] = 0; n[0][h] = n[1][h] = 0; ref[h] = -1; }
	note("wheel turns slowly both ways; keep it free");
	int32_t pos = 0;
	FieldAngle(0, mv);
	Wait(400);
	uint8_t prev = HallState(), stop = 0;
	for(uint8_t back=0;back<2&&!stop;back++){
		for(uint16_t i=0;i<720&&!stop;i++){
			pos += back ? -182 : 182;
			FieldAngle((uint16_t)pos, mv);
			Wait(10);
			uint8_t h = HallState();
			if(h!=prev&&hallSeq[h]<6&&hallSeq[prev]<6){
				//forward: state h starts here; backward: state prev started here
				uint8_t s = back ? prev : h;
				uint16_t a = (uint16_t)pos;
				if(ref[s]<0) ref[s] = a;
				sum[back][s] += (int16_t)(a-(uint16_t)ref[s]);
				n[back][s]++;
			}
			prev = h;
			if(i%90==89) progress(back*8+i/90+1, 16);
			stop = aborted();
		}
	}
	GatesOff();
	progressEnd();
	if(stop){ err("stopped"); return; }
	uint16_t ang[7];
	for(uint8_t h=1;h<=6;h++){
		if(!n[0][h]||!n[1][h]){ err("not all hall states seen both ways (more mV: calibrate hall <mV>)"); return; }
		int32_t f = sum[0][h]/n[0][h], b = sum[1][h]/n[1][h];
		ang[h] = (uint16_t)(ref[h]+(f+b)/2);
		if(!ang[h]||ang[h]==0xFFFF) ang[h] = 1;    //0 and 65535 mean unset
		char k[] = "hallang0";
		k[7] = (char)('0'+h);
		kvi(k, ang[h]);
		//against the built-in table (centres +-30 deg), and the lag forward-backward, in 0.1 deg el
		static const uint16_t def[7] = { 0, 38227, 60075, 49149, 16383, 27306, 5461 };    //centres - 30 deg
		kv("deg_x10"); outi(h); outc(' ');
		outi((int32_t)ang[h]*3600/65536); out(agent ? " offset " : " deg, off the built-in by ");
		outi((int32_t)(int16_t)(ang[h]-def[h])*3600/65536); out(agent ? " lag " : ", lag ");
		outi((f-b)*1800/65536); nl();
	}
	for(uint8_t h=1;h<=6;h++) HALL_ANG(h) = ang[h];
	ok();
}

//---------------------------------------------------------------- pole count
//The user turns the wheel one turn by hand: hall edges per turn = 3 x poles (6 per pole pair)
void CalPoles(uint32_t ms){
	if(!hallsSet()) return;
	if(!ms) ms = 60000;
	GatesOff();
	note("turn the wheel one full turn by hand; ends 3 s after it stops");
	edgeCount = 0;
	edgePrev = HallState();
	uint32_t t0 = millis, last = millis;
	int32_t seen = 0;
	while(millis-t0<ms){
		edgePoll();
		if(edgeCount!=seen){ seen = edgeCount; last = millis; }
		if((seen>=6||seen<=-6)&&millis-last>3000) break;
		if(aborted()){ err("stopped"); return; }
	}
	int32_t e = seen<0 ? -seen : seen;
	kvi("hall_edges", e);
	if(e<6){ err("the wheel did not turn"); return; }
	int32_t pp = (e+3)/6;
	WINDINGS = (uint16_t)(2*pp);
	kvi("windings", WINDINGS);
	if(e!=6*pp) warn("not whole pole pairs: turn exactly one turn, repeat");
	ok();
}

//---------------------------------------------------------------- phase currents
static int8_t chA, chB, sgn;
static uint8_t phA, phB;
static int32_t offA, offB, gain;

//offsets at zero current (all phases 50 %); 0: no phase current pins
static uint8_t curInit(void){
	chA = AdcChannel(IPHASEAPIN);
	chB = AdcChannel(IPHASEBPIN);
	if(chA<0||chB<0){ err("no phase current pins: detect current first"); return 0; }
	uint8_t pair = (uint8_t)(IPHASE_CFG&3);
	phA = pair==1 ? 1 : 0;
	phB = pair==0 ? 1 : 2;
	sgn = (IPHASE_CFG&4) ? 1 : -1;
	gain = (int32_t)cfgOrDefault(IPHASE_GAIN, 515);
	pinMode(IPHASEAPIN, INPUT_ADC);
	pinMode(IPHASEBPIN, INPUT_ADC);
	FieldAngle(0, 0);
	Wait(50);
	offA = offB = 0;
	for(uint8_t i=0;i<32;i++){ AdcScanTriggered(1u<<chA|1u<<chB); offA += AdcValue((uint8_t)chA); offB += AdcValue((uint8_t)chB); }
	return 1;
}

//phase currents in mA (into the motor), n samples averaged; returns the amplitude
static int32_t curRead(int32_t* i3, uint8_t n){
	int32_t a = 0, b = 0;
	for(uint8_t i=0;i<n;i++){ AdcScanTriggered(1u<<chA|1u<<chB); a += AdcValue((uint8_t)chA); b += AdcValue((uint8_t)chB); }
	int32_t ia = (a*32/n-offA)*sgn*gain/320, ib = (b*32/n-offB)*sgn*gain/320;    //x32 offsets, 0.1 mA per count
	i3[phA] = ia;
	i3[phB] = ib;
	i3[3-phA-phB] = -ia-ib;
	return isqrt((uint32_t)((i3[0]*i3[0]+i3[1]*i3[1]+i3[2]*i3[2])/3*2));
}

//---------------------------------------------------------------- resistance and inductance
//R: a static field at mv/2 and mv on the phase with a current pin (the rotor aligns, no motion), R = the
//voltage difference over the current difference (offsets drop out; MOSFETs included, as MOTOR_R). L: from
//0 to mv in one step, the current sampled every pwm period: the area between the final current and the
//rise is I x tau, L = R x tau (d axis).
void CalRL(int32_t mv){
	if(mv<=0) mv = 1000;
	if(!curInit()) return;
	uint16_t th = (uint16_t)(phA*21845);    //phase phA's direction: its current is the largest
	int32_t i3[3], cur[2];
	FieldAngle(th, mv);
	Wait(300);    //align
	for(uint8_t s=0;s<2;s++){
		FieldAngle(th, s ? mv : mv/2);
		Wait(80);
		cur[s] = curRead(i3, 32);
		kv("current_ma"); outi(cur[s]); out(agent ? " mv " : " mA at mV "); outi(s ? mv : mv/2); nl();
	}
	if(cur[1]-cur[0]<300){ GatesOff(); err("too little current (more mV: calibrate rl <mV>)"); return; }
	int32_t r = (mv-mv/2)*1000/(cur[1]-cur[0]);    //mOhm
	//inductance: step response of phase phA's current
	static int16_t smp[64];
	FieldAngle(th, 0);
	Wait(30);
	uint32_t m = 1u<<chA|1u<<chB;
	AdcScanTriggered(m);
	FieldAngle(th, mv);    //takes effect at the next update, with the next sample
	for(uint8_t i=0;i<64;i++){
		AdcScanTriggered(m);
		smp[i] = (int16_t)((AdcValue((uint8_t)chA)*32-offA)*sgn*gain/320);
	}
	GatesOff();
	int32_t fin = 0;
	for(uint8_t i=54;i<64;i++) fin += smp[i];
	fin /= 10;
	uint8_t t0 = 0;
	while(t0<20&&smp[t0+1]<fin/10) t0++;    //the last sample before the rise
	int32_t area = 0;    //trapezoids, x2
	for(uint8_t i=t0;i<63;i++) area += 2*fin-smp[i]-smp[i+1];
	int32_t per = ((int32_t)PWM_PERIOD+18)/36;    //us
	int32_t tau = fin>0 ? area*per/2/fin : 0;    //us
	int32_t l = r*tau/1000;    //uH
	kvi("tau_us", tau);
	MOTOR_R = (uint16_t)r;
	kvi("motorr", r);
	if(l>10&&l<20000){ PHASE_L = (uint16_t)l; kvi("phasel", l); }
	else{ err("no clean current rise: inductance not set"); return; }
	ok();
}

//---------------------------------------------------------------- speed constants
//rpm from the hall edges counted over ms while drive() runs the motor
static int32_t speedRun(void (*drive)(int32_t), int32_t level, uint32_t ms){
	edgePrev = HallState();
	uint32_t t0 = millis;
	while(millis-t0<1500){ drive(level); if(aborted()) return -999999; }    //accelerate
	edgeCount = 0;
	t0 = millis;
	while(millis-t0<ms){ drive(level); edgePoll(); }
	int32_t e = edgeCount<0 ? -edgeCount : edgeCount;
	return e*60000/(int32_t)ms/(3*(int32_t)WINDINGS);
}

static void driveBlock(int32_t permille){ RotateRun(permille); }

//sine-like drive for the kv: the field 90 deg ahead of the hall state's centre (calibrated edges, else the
//built-in table), complementary pwm: the q voltage averages 0.955 x mv over a 60 deg state (sin 30 / (pi/6))
static uint16_t centre[8];
static void driveField(int32_t mv){
	FieldAngle((uint16_t)(centre[HallState()]+16384), mv);
}

static void centres(void){
	static const uint16_t tab[8] = { 0, 43688, 0, 54610, 21844, 32767, 10922, 0 };
	uint8_t cal = 1;
	for(uint8_t h=1;h<=6;h++) if(HALL_ANG(h)==0||HALL_ANG(h)==0xFFFF) cal = 0;
	for(uint8_t h=0;h<8;h++){
		centre[h] = tab[h];
		if(!cal||h<1||h>6) continue;
		uint16_t lo = HALL_ANG(h), w = 0xFFFF;
		for(uint8_t j=1;j<=6;j++){ uint16_t d = (uint16_t)(HALL_ANG(j)-lo); if(j!=h&&d&&d<w) w = d; }
		centre[h] = (uint16_t)(lo+w/2);
	}
}

//block: rpm = kv/10 x Vbat x (duty - d0) at 3 duties (the main firmware's COM_SPEED model); sine: rpm per
//q voltage at 3 field voltages, kv = 5774 / (mV per rpm) (foc.c), d0 from the intercept
void CalKv(const char* which, int32_t top){
	if(!hallsSet()) return;
	if(WINDINGS<2||WINDINGS>200){ err("windings not set: calibrate poles first"); return; }
	uint8_t sine = which&&which[0]=='s';
	if(top<=0) top = sine ? 4000 : 500;
	int32_t vb = VbatMv();
	if(vb<10000){ err("no battery pin: detect vbat first"); return; }
	note("wheel spins at 3 speeds (keep it unloaded)");
	float x[3], y[3];
	if(sine) centres();
	RotateStop();
	for(uint8_t s=0;s<3;s++){
		int32_t lvl = top*(s+3)/5;    //60, 80, 100 %
		int32_t rpm = speedRun(sine ? driveField : driveBlock, lvl, 1500);
		if(rpm==-999999){ RotateStop(); GatesOff(); err("stopped"); return; }
		x[s] = sine ? (float)lvl*0.955f : (float)lvl/1000.0f;
		y[s] = (float)rpm;
		kv("rpm"); outi(rpm); out(sine ? (agent ? " mv " : " rpm at mV ") : (agent ? " permille " : " rpm at permille ")); outi(lvl); nl();
		progress(s+1, 3);
	}
	RotateStop();
	GatesOff();
	progressEnd();
	float a, b;
	fit(x, y, 3, &a, &b);
	if(a<=0.0f||y[0]<5.0f){ err("wheel did not turn at every level (calibrate kv block|sine <max>)"); return; }
	int32_t d0 = (int32_t)(-b/a*(sine ? 1.0f : 1000.0f));    //sine: mV, block: permille
	if(sine){
		int32_t kvs = (int32_t)(5774.0f*a);    //a = rpm per mV of q voltage
		int32_t d0p = d0*1732/vb;              //permille of the phase voltage at 100 %: vbat / sqrt(3)
		SINE_KV = (uint16_t)kvs;
		kvi("sinekv", kvs);
		if(d0p>0&&d0p<500){ SINE_D0 = (uint16_t)d0p; kvi("sined0", d0p); }
	}else{
		int32_t kvb = (int32_t)(a*10000.0f/(float)vb);    //0.1 rpm per V at 100 %
		SPEED_KV = (uint16_t)kvb;
		kvi("speedkv", kvb);
		if(d0>0&&d0<900){ SPEED_D0 = (uint16_t)d0; kvi("speedd0", d0); }
	}
	ok();
}
