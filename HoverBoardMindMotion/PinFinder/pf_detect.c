//PinFinder detections. Each one prints its results (config names) and ends with ok() or err(); the
//config words are changed in RAM only (save: the "save" command / the TUI's save question).
#include "pf.h"
#include "main.h"
#include "initialize.h"
#include "sim_eeprom.h"
#include "commutation.h"

uint8_t master;           //the board holds its own power (latch pin)
static uint8_t holdAll;   //no latch known: every free pin is pulled up (one of them may be the latch)

//free pins back to their idle state after a detection
static void releasePins(void){
	for(uint16_t p=0;p<PINCOUNT;p++){
		if(PinFree(p)) pinMode((uint8_t)p, holdAll ? INPUT_PULLUP : INPUT);
	}
}

void Tone(uint16_t pin, uint32_t ms, uint32_t hz){    //square wave on a pin (passive buzzer)
	if(pin>=PINCOUNT) return;
	pinMode((uint8_t)pin, OUTPUT);
	uint32_t half = 500000/hz;
	for(uint32_t t=millis;millis-t<ms;){
		digitalWrite((uint8_t)pin, 1);
		WaitUs(half);
		digitalWrite((uint8_t)pin, 0);
		WaitUs(half);
	}
}

//---------------------------------------------------------------- hall sensors
//The field is turned open loop in 12 directions per electrical turn (FieldVector: 30 deg el apart) and
//the rotor follows. Every free pin is read at each direction (majority over several turns): a hall pin
//is high for one block of ~6 of the 12. Which pin is A, B, C: the assignment for which the main firmware's
//block commutation (hall_to_pos, commutation.h) puts the field 60..120 deg ahead of the rotor at every
//direction (90 deg ideal). Some directions lie on a hall edge (bench motor: the block steps' own vectors),
//so the 6 block steps alone were ambiguous.
#define DIR_MS 100
#define TURNS 4
static const uint8_t hallToPos[8] = HALL_TO_POS_INIT;

//pins: candidate pins A, B, C; returns the number of directions at 90 deg lead, -1 if any is off 60..120
static int8_t leadScore(const uint16_t* bit, const uint16_t* sure, const uint8_t* pins, uint8_t invert){
	int8_t score = 0;
	for(uint8_t k=0;k<12;k++){
		if(!((sure[pins[0]]&sure[pins[1]]&sure[pins[2]])>>k&1)) continue;    //on an edge
		uint8_t h = (uint8_t)((bit[pins[0]]>>k&1)|(bit[pins[1]]>>k&1)<<1|(bit[pins[2]]>>k&1)<<2);
		if(invert) h = (uint8_t)(7-h);
		uint8_t st = hallToPos[h];
		if(!st) return -1;
		uint8_t lead = (uint8_t)((2*(st-1)+12-k)%12);    //step st's field is direction 2*(st-1)
		if(lead<2||lead>4) return -1;
		if(lead==3) score++;
	}
	return score;
}

void DetectHall(int32_t mv){
	uint16_t old[3] = { HALLAPIN, HALLBPIN, HALLCPIN };
	static uint8_t cnt[PINCOUNT][12];
	if(mv<=0) mv = 1000;
	HALLAPIN = HALLBPIN = HALLCPIN = PIN_NONE;
	for(uint16_t p=0;p<PINCOUNT;p++){
		if(PinFree(p)) pinMode((uint8_t)p, INPUT_PULLUP);    //open collector hall outputs
	}
	note("motor turns slowly; keep the wheel free, any key stops");
	uint8_t turns = 0, stop = 0;
	for(uint8_t n=0;n<16&&turns<TURNS&&!stop;n++){
		uint16_t pat[PINCOUNT];
		uint8_t moved = 0;
		if(!turns) for(uint16_t p=0;p<PINCOUNT;p++) for(uint8_t k=0;k<12;k++) cnt[p][k] = 0;
		for(uint16_t p=0;p<PINCOUNT;p++) pat[p] = 0;
		for(uint8_t k=0;k<12;k++){
			FieldVector(k, mv);
			Wait(DIR_MS);
			for(uint16_t p=0;p<PINCOUNT;p++){
				if(PinFree(p)&&digitalRead((uint8_t)p)) pat[p] |= (uint16_t)(1u<<k);
			}
		}
		for(uint16_t p=0;p<PINCOUNT;p++){    //moved: a pin with one block of 4..8 highs (2 edges)
			if(!PinFree(p)||!pat[p]||pat[p]==0xFFF) continue;
			uint8_t edges = 0, ones = 0;
			for(uint8_t k=0;k<12;k++){
				ones = (uint8_t)(ones+((pat[p]>>k)&1));
				edges = (uint8_t)(edges+(((pat[p]>>k)^(pat[p]>>((k+1)%12)))&1));
			}
			if(edges==2&&ones>=4&&ones<=8) moved = 1;
		}
		if(!moved){    //the rotor does not follow: more voltage, count again
			turns = 0;
			if(mv<3000) mv = mv*5/4;
		}else{
			for(uint16_t p=0;p<PINCOUNT;p++) for(uint8_t k=0;k<12;k++) cnt[p][k] = (uint8_t)(cnt[p][k]+((pat[p]>>k)&1));
			turns++;
		}
		progress(n+1, 16);
		stop = aborted();
	}
	GatesOff();
	progressEnd();
	releasePins();
	kvmv("voltage", mv);
	//hall candidates: pins high in 4..8 of the 12 directions, as one block
	static uint16_t bit[PINCOUNT], sure[PINCOUNT];    //per direction: majority level, all turns the same
	uint8_t cand[6], nc = 0;
	for(uint16_t p=0;p<PINCOUNT&&turns==TURNS;p++){
		if(!PinFree(p)) continue;
		uint8_t ones = 0, edges = 0, active = 0;
		bit[p] = sure[p] = 0;
		for(uint8_t k=0;k<12;k++){
			if(cnt[p][k]*2>TURNS) bit[p] |= (uint16_t)(1u<<k);
			if(cnt[p][k]==0||cnt[p][k]==TURNS) sure[p] |= (uint16_t)(1u<<k);
			if(cnt[p][k]) active = 1;
		}
		for(uint8_t k=0;k<12;k++){
			ones = (uint8_t)(ones+(bit[p]>>k&1));
			edges = (uint8_t)(edges+((bit[p]>>k^bit[p]>>((k+1)%12))&1));
		}
		if(!active||ones==12) continue;
		kv("counts"); outpin(p); outc(' ');
		for(uint8_t k=0;k<12;k++) outc((char)('0'+cnt[p][k]));
		nl();
		if(edges==2&&ones>=4&&ones<=8&&nc<6) cand[nc++] = (uint8_t)p;
	}
	//the assignment of 3 candidates to A, B, C with every direction at 60..120 deg lead, best at 90
	int8_t best = -1, inverted = 0;
	uint8_t pick[3] = { 0, 0, 0 };
	for(uint8_t i=0;i<nc;i++) for(uint8_t j=0;j<nc;j++) for(uint8_t m=0;m<nc;m++){
		if(i==j||j==m||i==m) continue;
		uint8_t pins[3] = { cand[i], cand[j], cand[m] };
		int8_t sc = leadScore(bit, sure, pins, 0);
		if(sc>best){ best = sc; pick[0] = pins[0]; pick[1] = pins[1]; pick[2] = pins[2]; }
		if(leadScore(bit, sure, pins, 1)>=0) inverted = 1;
	}
	if(stop||turns<TURNS||best<0){
		HALLAPIN = old[0]; HALLBPIN = old[1]; HALLCPIN = old[2];
		if(stop) err("stopped");
		else if(turns<TURNS) err("motor did not follow the field (more mV: detect hall <mV>)");
		else if(inverted) err("halls fit only inverted (180 deg off): not supported");
		else err(nc<3 ? "fewer than 3 hall pins found" : "no hall pin order fits the commutation");
		return;
	}
	HALLAPIN = pick[0]; HALLBPIN = pick[1]; HALLCPIN = pick[2];
	kvp("halla", HALLAPIN);
	kvp("hallb", HALLBPIN);
	kvp("hallc", HALLCPIN);
	kvi("lead90", best);    //directions at the ideal 90 deg lead (of up to 6 off the hall edges)
	//speed measurement: the main firmware captures the hall edges with TIM2 or TIM3 (initialize.c)
	int8_t tim[3] = { -1, -1, -1 };
	for(uint8_t i=0;i<TIMCOUNT;i++){
		for(uint8_t h=0;h<3;h++){
			if(halltims[i].io==pick[h]) tim[h] = (int8_t)halltims[i].tim2;
		}
	}
	if(tim[0]<0||tim[1]<0||tim[2]<0||tim[0]!=tim[1]||tim[1]!=tim[2]){
		warn("hall pins not on one timer: no speed measurement");
	}
	ok();
}

//---------------------------------------------------------------- battery voltage
void ShowAdc(void){
	int32_t vdd = VddMv();
	kvmv("vdd", vdd);
	for(uint8_t i=0;i<ADCCOUNT;i++){
		uint16_t p = adcs[i].io;
		if(!PinFree(p)&&p!=VBATPIN&&p!=ITOTALPIN&&p!=IPHASEAPIN&&p!=IPHASEBPIN) continue;
		kv(PinName(p));
		int32_t mv = PinMv(p);
		if(agent){ outi(mv); out(" vbat_mv="); outi(mv*(int32_t)VBAT_DIVIDER); }
		else{ outfix(mv, 3); out(" V  (x"); outi(VBAT_DIVIDER); out(" = "); outfix(mv*(int32_t)VBAT_DIVIDER/10, 2); out(" V)"); }
		if(p==VBATPIN) out(agent ? " vbat" : "  <- vbat");
		else if(p==ITOTALPIN) out(agent ? " itotal" : "  <- itotal");
		else if(p==IPHASEAPIN||p==IPHASEBPIN) out(agent ? " iphase" : "  <- phase current");
		nl();
	}
}

//the pin whose voltage times VBAT_DIVIDER is closest to expect_mv (or the only one between 20 and 45 V)
void DetectVbat(int32_t expect_mv){
	uint16_t best = PIN_NONE, inrange = 0;
	int32_t bestd = 0x7FFFFFFF;
	for(uint8_t i=0;i<ADCCOUNT;i++){
		uint16_t p = adcs[i].io;
		if(!PinFree(p)&&p!=VBATPIN) continue;
		int32_t v = PinMv(p)*(int32_t)VBAT_DIVIDER;
		if(expect_mv>0){
			int32_t d = v>expect_mv ? v-expect_mv : expect_mv-v;
			if(d<bestd){ bestd = d; best = p; }
		}else if(v>20000&&v<45000){
			best = p;
			inrange++;
		}
	}
	if((expect_mv<=0&&inrange!=1)||best==PIN_NONE||(expect_mv>0&&bestd>expect_mv/5)){
		ShowAdc();
		kvp("vbat_kept", VBATPIN);    //unchanged
		err(expect_mv>0 ? "no pin within 20 % of that voltage" :
		    inrange ? "several pins look like the battery: give the voltage (detect vbat 36.5)"
		            : "no pin reads 20..45 V: give the voltage (detect vbat 36.5)");
		return;
	}
	VBATPIN = best;
	kvp("vbat", VBATPIN);
	if(expect_mv>0){    //the divider from the measured voltage, 0.001 (the integer vbatdivider is 3 % steps)
		int32_t pmv = PinMv(best);
		if(pmv>0){ VBAT_GAIN = (uint16_t)(expect_mv*1000/pmv); kvi("vbatgain", VBAT_GAIN); }
	}
	kvmv("battery", VbatMv());
	ok();
}

//---------------------------------------------------------------- inputs the user operates
//A pin's state: 0 / 1 when something drives it (the same with pull-up and pull-down), 2 when floating
static uint8_t pinState(uint16_t p){
	pinMode((uint8_t)p, INPUT_PULLUP);
	WaitUs(30);
	uint8_t up = digitalRead((uint8_t)p);
	pinMode((uint8_t)p, INPUT_PULLDOWN);
	WaitUs(30);
	uint8_t down = digitalRead((uint8_t)p);
	pinMode((uint8_t)p, holdAll ? INPUT_PULLUP : INPUT);
	return up==down ? up : 2;
}

//waits for a free pin (or 'also') whose state changes and stays changed for 50 ms; pins that change by
//themselves in the first 0.5 s are ignored
static uint16_t waitChange(uint16_t also, uint32_t ms, uint8_t* from, uint8_t* to){
	uint8_t st[PINCOUNT], noisy[PINCOUNT];
	for(uint16_t p=0;p<PINCOUNT;p++){
		noisy[p] = 0;
		if(PinFree(p)||p==also) st[p] = pinState(p);
	}
	for(uint32_t t=millis;millis-t<500;){
		for(uint16_t p=0;p<PINCOUNT;p++){
			if((PinFree(p)||p==also)&&pinState(p)!=st[p]) noisy[p] = 1;
		}
	}
	note("now ...");
	for(uint32_t t=millis;millis-t<ms;){
		for(uint16_t p=0;p<PINCOUNT;p++){
			if(!(PinFree(p)||p==also)||noisy[p]) continue;
			uint8_t s = pinState(p);
			if(s==st[p]) continue;
			uint32_t t2 = millis;
			while(millis-t2<50&&pinState(p)==s);
			if(millis-t2<50) continue;    //a bounce
			*from = st[p];
			*to = s;
			return p;
		}
		if(aborted()) return PIN_NONE;
	}
	return PIN_NONE;
}

void DetectButton(uint32_t ms){
	uint8_t from, to;
	if(!ms) ms = 15000;
	note("press and hold the power button (any key cancels)");
	uint16_t p = waitChange(BUTTONPIN, ms, &from, &to);
	if(p==PIN_NONE){ err("no pin changed"); return; }
	BUTTONPIN = p;
	kvp("button", p);
	kvi("released", from);
	kvi("pressed", to);
	if(to!=1) warn("reads 0 when pressed: the firmware expects 1");
	ok();
}

//start with the charger disconnected: the pin that changes when it is connected, and how to read it
//(the main firmware: CHARGE_LEVEL, CHARGE_PULL)
//an input the user switches (charger, infrared foot sensor): the pin that changes, its level while active
//(1 low, 2 high) and its input mode (1 floating, 2 pull-up, 3 pull-down; states 0 / 1 driven, 2 floating:
//floating while active, it is pulled towards the other level). 0: no pin changed (the old pin stays)
static uint8_t userInput(uint16_t* pin, uint32_t ms, uint8_t* level, uint8_t* pull){
	uint8_t from, to;
	uint16_t keep = *pin;
	*pin = PIN_NONE;
	uint16_t p = waitChange(keep, ms ? ms : 30000, &from, &to);
	if(p==PIN_NONE){ *pin = keep; err("no pin changed"); return 0; }
	*level = (uint8_t)((to==2 ? !from : to) ? 2 : 1);
	*pull = to==2 ? (from ? 3 : 2) : (from==2 ? (to ? 3 : 2) : 1);
	*pin = p;
	return 1;
}

void DetectCharger(uint32_t ms){
	uint8_t level, pull;
	note("plug in the charger (start unplugged; any key cancels)");
	if(!userInput(&CHARGEPIN, ms, &level, &pull)) return;
	CHARGE_LEVEL = level;
	CHARGE_PULL = pull;
	kvp("charge", CHARGEPIN);
	kvi("chargelevel", level);
	kvi("chargepull", pull);
	ok();
}

//the two infrared foot sensors, one after the other (the first one is no longer free for the second)
void DetectIr(uint32_t ms){
	uint16_t mode = 0;
	for(uint8_t k=0;k<2;k++){
		uint8_t level, pull;
		note(k ? "now cover the other one" : "cover one infrared foot sensor (any key cancels)");
		if(!userInput(&IR_PIN(k), ms, &level, &pull)) return;
		mode = (uint16_t)(mode|(level|pull<<2)<<(4*k));
		kvp(k ? "ir2" : "ir1", IR_PIN(k));
	}
	IR_MODE = mode;
	kvi("irmode", mode);
	ok();
}

//one block step (s 1..6) or field direction (s 101..112 = k 0..11, FieldVector) held for ms, then the hall
//state and the currents (diagnostics)
void StepHold(uint8_t s, int32_t mv, uint32_t ms){
	if(s>=101&&s<=112) FieldVector((uint8_t)(s-101), mv>0 ? mv : 1000);
	else if(s>=1&&s<=6) BlockStep(s, DutyForMv(mv>0 ? mv : 2000));
	else{ err("step 1..6, or 101..112 = field direction 0..11"); return; }
	Wait(ms ? ms : 300);
	uint8_t h = HallState();
	//current at the end of the hold: averaged triggered samples of the shunt and phase current pins
	static const uint8_t which[3] = { 13, 16, 17 };    //itotal, iphasea, iphaseb
	static const char* const names[3] = { "itotal_counts", "iphasea_counts", "iphaseb_counts" };
	int32_t sum[3] = { 0, 0, 0 };
	uint32_t mask = 0;
	for(uint8_t i=0;i<3;i++){ int8_t ch = AdcChannel(pinstorage[which[i]]); if(ch>=0){ mask |= 1u<<ch; pinMode((uint8_t)pinstorage[which[i]], INPUT_ADC); } }
	for(uint8_t n=0;n<16&&mask;n++){
		AdcScanTriggered(mask);
		for(uint8_t i=0;i<3;i++){ int8_t ch = AdcChannel(pinstorage[which[i]]); if(ch>=0) sum[i] += AdcValue((uint8_t)ch); }
	}
	GatesOff();
	kvi("step", s);
	kvi("hall", h);
	for(uint8_t i=0;i<3;i++) if(AdcChannel(pinstorage[which[i]])>=0) kvi(names[i], sum[i]/16);
	ok();
}

//---------------------------------------------------------------- test rotation (hall commutation)
//hall state -> position in the sequence of rising steps (1,3,2,6,4,5); 7 = invalid state
static const uint8_t hallPos[8] = { 7, 0, 2, 1, 4, 5, 3, 7 };
static uint8_t rotPrev;
static int32_t rotFwd, rotBack, rotBad;

void RotateRun(int32_t permille){
	uint8_t h = HallState();
	uint16_t duty = (uint16_t)((permille<0 ? -permille : permille)*(int32_t)PWM_PERIOD/1000);
	if(h!=rotPrev&&rotPrev){
		uint8_t d = (uint8_t)((hallPos[h]+6-hallPos[rotPrev])%6);
		if(hallPos[h]>5||hallPos[rotPrev]>5) rotBad++;
		else if(d==1) rotFwd++;
		else if(d==5) rotBack++;
		else rotBad++;
	}
	rotPrev = h;
	if(!permille){ GatesOff(); return; }
	BlockStep(hallToPos[permille>0 ? h : 7-h], duty);    //the main firmware: hall_to_pos[hallpos(dir)]
}

void RotateStop(void){
	GatesOff();
	rotPrev = 0;
	rotFwd = rotBack = rotBad = 0;
}

void Rotate(int32_t permille, uint32_t ms){
	if(HALLAPIN>=PINCOUNT||HALLBPIN>=PINCOUNT||HALLCPIN>=PINCOUNT){ err("hall pins not set: detect hall first"); return; }
	if(permille>1000||permille<-1000){ err("permille -1000..1000"); return; }
	if(!ms) ms = 2000;
	pinMode(HALLAPIN, INPUT); pinMode(HALLBPIN, INPUT); pinMode(HALLCPIN, INPUT);
	RotateStop();
	uint32_t t = millis;
	while(millis-t<ms&&!aborted()) RotateRun(permille);
	int32_t f = rotFwd, b = rotBack, bad = rotBad;
	RotateStop();
	kvi("hall_steps_forward", f);
	kvi("hall_steps_backward", b);
	kvi("hall_errors", bad);
	if(VBATPIN<PINCOUNT) kvmv("battery", VbatMv());
	if(f+b<6) err("motor did not turn (more duty? hall pins?)");
	else if(bad*10>f+b) err("hall sequence errors");
	else ok();
}

//---------------------------------------------------------------- uart search (boot, no uart pins known)
//The user connects the board's uart TX and RX together: every tx/rx pin pair of one uart is tried until
//the test bytes come back; the pins are saved at once, then the board waits until the loop is removed.
static uint8_t loopback(UART_TypeDef* u, uint8_t c){
	while(u->CSR&UART_CSR_RXAVL) (void)u->RDR;
	u->TDR = c;
	for(uint32_t t=millis;millis-t<5;){
		if(u->CSR&UART_CSR_RXAVL) return (uint8_t)u->RDR==c;
	}
	return 0;
}

uint8_t UartFind(void){
	static const uint8_t test[] = { 0x11, 0x55, 0xAA, 0x12, 0xF0, 0x0F };
	for(uint8_t t=0;t<UARTCOUNT;t++){
		if(!uarts[t].tx||!PinFree(uarts[t].io)) continue;
		for(uint8_t r=0;r<UARTCOUNT;r++){
			if(uarts[r].tx||uarts[r].uart!=uarts[t].uart||!PinFree(uarts[r].io)) continue;
			for(uint8_t i=0;i<UARTCOUNT;i++){    //no other pin on a uart function
				if(PinFree(uarts[i].io)){ pinMode(uarts[i].io, INPUT); pinModeAF(uarts[i].io, GPIO_AF_7); }
			}
			UART_TypeDef* u = uarts[t].uart==2 ? UART2 : UART1;
			UARTX_Init(PF_BAUD, uarts[t].uart);
			pinModeAF(uarts[t].io, uarts[t].af);
			pinModeAF(uarts[r].io, uarts[r].af);
			pinMode(uarts[t].io, OUTPUT_AF);
			pinMode(uarts[r].io, INPUT_PULLUP);
			uint8_t good = 1;
			for(uint8_t i=0;i<sizeof(test)&&good;i++) good = loopback(u, test[i]);
			if(!good) continue;
			TXPIN = uarts[t].io;
			RXPIN = uarts[r].io;
			CfgSave();    //found: kept even if the board loses power before the menu
			while(loopback(u, '.')) Wait(200);    //until the loop is removed
			Wait(500);
			uartUsed = uarts[t].uart;
			return 1;
		}
	}
	return 0;
}

//---------------------------------------------------------------- power latch
void LatchHold(void){
	if(LATCHPIN<PINCOUNT){
		pinMode(LATCHPIN, OUTPUT);
		digitalWrite(LATCHPIN, 1);
		master = 1;
		holdAll = 0;
	}else{
		holdAll = 1;    //might be the latch: hold every free pin up (pull-up, not a hard high)
		master = 0;
		releasePins();
	}
}

static uint8_t debuggerAttached(void){
	return (*(volatile uint32_t*)0xE000EDF0&1u)!=0;    //DHCSR C_DEBUGEN (no CoreDebug in core_cm0.h)
}

//Releases the free pins one at a time; before each the pin is saved as LATCHPIN. If the board switches
//off, it was the latch: the next start holds it. If all pins pass, the board has no latch (slave board).
void LatchFind(void){
	if(debuggerAttached()) warn("debugger attached: if it powers the MCU, no latch found");
	note("release the button; the board switches off at the latch: switch it on again");
	Wait(2000);
	holdAll = 1;
	uint16_t keep = LATCHPIN;
	LATCHPIN = PIN_NONE;
	releasePins();
	if(keep<PINCOUNT){ pinMode(keep, INPUT_PULLUP); }
	for(uint16_t p=0;p<PINCOUNT;p++){
		if(!PinFree(p)||p==TXPIN||p==RXPIN) continue;
		LATCHPIN = p;
		CfgSave();
		LATCHPIN = PIN_NONE;
		kvp("trying", p);
		pinMode((uint8_t)p, INPUT_PULLDOWN);
		Wait(400);
		pinMode((uint8_t)p, INPUT_PULLUP);
		Wait(100);
	}
	LATCHPIN = PIN_NONE;
	CfgSave();
	holdAll = 0;
	master = 0;
	releasePins();
	kvp("latch", PIN_NONE);
	note("still powered: no latch (slave, or debugger supply)");
	ok();
}

void PowerOff(void){
	GatesOff();
	note("switching off");
	Wait(50);
	if(LATCHPIN<PINCOUNT){ digitalWrite(LATCHPIN, 0); pinMode(LATCHPIN, INPUT_PULLDOWN); }
	for(uint16_t p=0;p<PINCOUNT;p++){
		if(holdAll&&PinFree(p)) pinMode((uint8_t)p, INPUT_PULLDOWN);
	}
	Wait(1500);
	LatchHold();
	err("still powered (no latch, button held, or debugger supply)");
}

//---------------------------------------------------------------- info
void Info(void){
	static const char hex[] = "0123456789ABCDEF";
	kvs("pinfinder", PF_VERSION);
	kv("idcode");
	uint32_t id = *(volatile uint32_t*)0x40013400;    //DBGMCU_IDCODE
	for(int8_t i=28;i>=0;i-=4) outc(hex[(id>>i)&15]);
	nl();
	kv("uid");
	for(uint8_t i=0;i<12;i++){ uint8_t b = ((const volatile uint8_t*)0x1FFFF7E8)[i]; outc(hex[b>>4]); outc(hex[b&15]); }
	nl();
	kvs("board", master ? "master" : "slave");
	kvp("latch", LATCHPIN);
	kvi("debugger", debuggerAttached());
	kvmv("vdd", VddMv());
	if(VBATPIN<PINCOUNT) kvmv("battery", VbatMv());
	if(HALLAPIN<PINCOUNT&&HALLBPIN<PINCOUNT&&HALLCPIN<PINCOUNT) kvi("hall", HallState());
	kvp("tx", TXPIN);
	kvp("rx", RXPIN);
	kvi("baud", PF_BAUD);
	kvi("config_changed", CfgChanged());
}

//---------------------------------------------------------------- current sensing
//Phase currents: a static field (complementary PWM) in 12 directions; every free adc pin is sampled where
//the main firmware samples (low sides on). A phase current pin follows cos(theta - 120 X) of its phase X
//(theta = 210 + 30 k, FieldVector), rising or falling with current into the motor (iphasecfg bit 2).
//Bus current: one phase high, two low (the active vector) carries that phase's current through the shunt;
//sampled in the middle of the active vector at several amplitudes against the phase current pins.
static const int16_t cos12[12] = { 1000, 866, 500, 0, -500, -866, -1000, -866, -500, 0, 500, 866 };

static uint32_t freeAdcMask(void){
	uint32_t m = 0;
	for(uint8_t i=0;i<ADCCOUNT;i++){
		uint16_t p = adcs[i].io;
		if(PinFree(p)||p==ITOTALPIN||p==IPHASEAPIN||p==IPHASEBPIN){
			pinMode((uint8_t)p, INPUT_ADC);
			m |= 1u<<adcs[i].channel;
		}
	}
	return m;
}

static void sampleAvg(uint32_t mask, uint16_t at, int32_t* avg, uint16_t n){    //avg per channel (x n)
	for(uint8_t c=0;c<16;c++) avg[c] = 0;
	for(uint16_t i=0;i<n;i++){
		AdcScanAt(mask, at);
		for(uint8_t c=0;c<16;c++) if(mask>>c&1) avg[c] += AdcValue(c);
	}
}

void DetectCurrent(int32_t mv, int32_t shunt_uohm){
	if(mv<=0) mv = 1000;
	uint32_t mask = freeAdcMask();
	int32_t zero[16], v[16];
	static int16_t d[12][16];
	note("a few A of static current; the wheel may move a little");
	FieldVector(0, 0);    //all phases at 50 %: no current, low sides on at the sample
	Wait(50);
	sampleAvg(mask, 1, zero, 16);
	for(uint8_t k=0;k<12;k++){
		FieldVector(k, mv);
		Wait(60);
		sampleAvg(mask, 1, v, 16);
		for(uint8_t c=0;c<16;c++) d[k][c] = (int16_t)((v[c]-zero[c])/16);
		progress(k+1, 12);
	}
	FieldVector(0, 0);
	progressEnd();
	//per pin: the phase and sign with the largest correlation; amplitude = corr / 6000 counts
	uint16_t pin[3] = { PIN_NONE, PIN_NONE, PIN_NONE };
	int32_t amp[3] = { 0, 0, 0 };
	int8_t sign[3] = { 0, 0, 0 };
	for(uint8_t i=0;i<ADCCOUNT;i++){
		uint8_t c = (uint8_t)adcs[i].channel;
		if(!(mask>>c&1)) continue;
		int32_t best = 0, bx = 0, total = 0;
		for(uint8_t k=0;k<12;k++) total += (int32_t)d[k][c]*d[k][c];
		for(uint8_t x=0;x<3;x++){
			int32_t corr = 0;
			for(uint8_t k=0;k<12;k++) corr += d[k][c]*cos12[(7+k+12-4*x)%12];
			if((corr<0 ? -corr : corr)>(best<0 ? -best : best)){ best = corr; bx = x; }
		}
		int32_t a = (best<0 ? -best : best)/6000;
		//fit quality: the sinusoid's energy (6 a^2) against the total
		if(a<8||6*a*a*10<total*7) continue;
		kv("phase_current"); outpin(adcs[i].io); out(agent ? " phase=" : " phase "); outc((char)('A'+bx));
		out(agent ? " amplitude=" : " amplitude "); outi(a); out(best>0 ? " rises" : " falls"); nl();
		if(a>amp[bx]){ amp[bx] = a; pin[bx] = adcs[i].io; sign[bx] = (int8_t)(best>0 ? 1 : -1); }
	}
	uint8_t n = (pin[0]!=PIN_NONE)+(pin[1]!=PIN_NONE)+(pin[2]!=PIN_NONE);
	if(n<2){
		IPHASEAPIN = IPHASEBPIN = PIN_NONE;
		note("no phase current pins: no FOC, sine uses the bus current");
	}else{
		uint8_t a = pin[0]!=PIN_NONE ? 0 : 1, b = pin[2]!=PIN_NONE ? 2 : 1;    //pairs A,B / B,C / A,C
		if(a==0&&pin[1]!=PIN_NONE&&b==2&&amp[1]>amp[2]) b = 1;
		if(sign[a]!=sign[b]){ err("phase current pins of opposite sign (one polarity bit)"); return; }
		IPHASEAPIN = pin[a];
		IPHASEBPIN = pin[b];
		IPHASE_CFG = (uint16_t)((a==0&&b==1 ? 0 : a==1 ? 1 : 2)|(sign[a]>0 ? 4 : 0));
		kvp("iphasea", IPHASEAPIN);
		kvp("iphaseb", IPHASEBPIN);
		kvi("iphasecfg", IPHASE_CFG);
		//current amplitude = mv / phase resistance: the gain with motorr (default 300 mOhm), information only
		int32_t r = (int32_t)cfgOrDefault(MOTOR_R, 300);
		kvi("iphasegain_from_motorr", mv*10000/r/((amp[a]+amp[b])/2));    //0.1 mA per count
	}
	//bus current pin: phase X high, the others low (k = 4X-7: cos = 1 for X), sampled in the active vector
	if(IPHASEAPIN>=PINCOUNT){ ok(); return; }
	int32_t gain = (int32_t)cfgOrDefault(IPHASE_GAIN, 515);    //0.1 mA per count
	uint8_t pa = (uint8_t)(IPHASE_CFG&3), ph[2] = { (uint8_t)(pa==1 ? 1 : 0), (uint8_t)(pa==0 ? 1 : 2) };
	int8_t ca = AdcChannel(IPHASEAPIN), cb = AdcChannel(IPHASEBPIN);
	int8_t sg = (IPHASE_CFG&4) ? 1 : -1;
	uint8_t x = (uint8_t)(3-ph[0]-ph[1]);    //the phase without a pin is high: I = -(Ia + Ib)
	uint8_t kx = (uint8_t)((4*x+12-7)%12);
	//phase x's current from the phase current pins (I_x = -(I_a + I_b)), mA
	#define PHASE_X_MA() (-(((v[ca]-zero[ca])/16)*sg+((v[cb]-zero[cb])/16)*sg)*gain/10)
	//8 levels (0.75 .. 2.5 x mv), every candidate pin sampled in the middle of the active vector, one channel
	//per conversion (a scan converts its channels ~1.7 us apart, the window is ~+-7 us). The bus pin is the
	//one that rises steadily with the current (op-amp inputs read high-impedance noise there: flat or jumpy)
	static uint16_t avg[8][16];    //counts x 8, summed over the passes
	float xs[8];
	for(uint8_t s=0;s<8;s++){ xs[s] = 0; for(uint8_t c=0;c<16;c++) avg[s][c] = 0; }
	FieldVector(kx, mv*5/2);    //the rotor aligns first: a swinging rotor changed the current between the samples
	Wait(500);
	//3 passes: a 3 mOhm shunt is ~2.4 counts per A, the slope of one pass varied +-3 %
	for(uint8_t pass=0;pass<3;pass++){
		for(uint8_t s=0;s<8;s++){
			FieldVector(kx, mv*(s+3)/4);
			Wait(150);
			uint16_t at = (uint16_t)((fieldCcr[x]+fieldCcr[(x+1)%3])/2);    //middle of the active vector
			sampleAvg(mask, 1, v, 16);
			int32_t before = PHASE_X_MA();
			for(uint8_t c=0;c<16;c++){
				int32_t one[16];
				if(!(mask>>c&1)) continue;
				sampleAvg(1u<<c, at, one, 192);
				avg[s][c] = (uint16_t)(avg[s][c]+one[c]/72);
			}
			sampleAvg(mask, 1, v, 16);
			xs[s] += (float)(before+PHASE_X_MA())/6000.0f;    //A, before and after the bus samples
			progress(pass*8+s+1, 24);
		}
	}
	FieldVector(0, 0);
	GatesOff();
	progressEnd();
	kvi("bus_test_current_ma", (int32_t)(xs[7]*1000.0f));
	uint16_t best = PIN_NONE;
	float slope = 0, z = 0;
	for(uint8_t i=0;i<ADCCOUNT;i++){
		uint8_t c = (uint8_t)adcs[i].channel;
		uint16_t p = adcs[i].io;
		if(!(mask>>c&1)||p==IPHASEAPIN||p==IPHASEBPIN) continue;
		//least squares over the points above the adc's zero (a shunt's zero can be below it), rising steadily
		float sx = 0, sy = 0, sxx = 0, sxy = 0;
		uint8_t np = 0, steady = 1;
		for(uint8_t s=0;s<8;s++){
			float y = (float)avg[s][c]/8.0f;
			if(s&&avg[s][c]+4<avg[s-1][c]) steady = 0;    //falls by more than half a count
			if(y<3.0f) continue;
			sx += xs[s]; sy += y; sxx += xs[s]*xs[s]; sxy += xs[s]*y; np++;
		}
		float den = (float)np*sxx-sx*sx;
		if(!steady||np<3||den<0.01f) continue;
		float a = ((float)np*sxy-sx*sy)/den;
		if(a>0.5f&&(a>slope||p==ITOTALPIN)&&best!=ITOTALPIN){ slope = a; z = (sy-a*sx)/(float)np; best = p; }    //the configured pin first
	}
	if(best==PIN_NONE){ err("no bus current pin found"); return; }
	ITOTALPIN = best;
	kvp("itotal", best);
	int32_t div = (int32_t)(4096000.0f/(slope*(float)VddMv()));    //divider = A / V = 1 / (counts per A x VDD / 4096)
	kvi("counts_per_a_x100", (int32_t)(slope*100.0f));
	kvi("zero_counts_x100", (int32_t)(z*100.0f));
	if(shunt_uohm>0){
		//the shunt value is the reference: counts per A = R x 4096 / VDD; the phase currents read the measured
		//slope / that too high, so the phase gain scales by it
		float cTrue = (float)shunt_uohm*4.096f/(float)VddMv();    //uOhm * 4096 / mV / 1000
		int32_t g = (int32_t)((float)gain*slope/cTrue);    //(the zero is in counts: unchanged)
		div = (int32_t)(1000000.0f/(float)shunt_uohm);
		if(g>50&&g<5000){ IPHASE_GAIN = (uint16_t)g; kvi("iphasegain", g); }
	}
	if(div>0&&div<65535){ ITOTAL_DIVIDER = (uint16_t)div; kvi("itotaldivider", div); }
	ITOTAL_ZERO = (uint16_t)(z<0.0f ? (int32_t)(-z*100.0f) : 0);
	kvi("itotalzero", ITOTAL_ZERO);
	ok();
}

