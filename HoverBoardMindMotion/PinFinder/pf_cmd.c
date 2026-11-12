//The command core: one line, "<command> [args]", the same in agent mode and on the TUI's command line.
//Agent mode answers with "key=value" lines and a last line "ok" or "err <text>" (README "PinFinder").
#include "pf.h"
#include "main.h"
#include "sim_eeprom.h"
#include <string.h>


static int32_t num(const char* s, uint8_t* good){    //decimal or 0x hex, optional '-'
	int32_t v = 0, sign = 1, base = 10;
	*good = 0;
	if(!s) return 0;
	if(*s=='-'){ sign = -1; s++; }
	if(s[0]=='0'&&(s[1]=='x'||s[1]=='X')){ base = 16; s += 2; }
	if(!*s) return 0;
	for(;*s;s++){
		int32_t d = *s>='0'&&*s<='9' ? *s-'0' : (*s|32)>='a'&&(*s|32)<='f'&&base==16 ? (*s|32)-'a'+10 : -1;
		if(d<0) return 0;
		v = v*base+d;
	}
	*good = 1;
	return v*sign;
}

static uint32_t optnum(const char* s){    //optional positive argument, 0 = default
	uint8_t g;
	int32_t v = num(s, &g);
	return g&&v>0 ? (uint32_t)v : 0;
}

//a voltage in V ("24.8", "36") or mV ("24800"): with a decimal point or below 100 it is volts; 0 = none
static int32_t millivolts(const char* s){
	int32_t v = 0, frac = 0, scale = 1;
	uint8_t dot = 0;
	if(!s) return 0;
	for(;*s;s++){
		if(*s=='.'&&!dot){ dot = 1; continue; }
		if(*s<'0'||*s>'9') return 0;
		if(dot){ if(scale<1000){ frac = frac*10+(*s-'0'); scale *= 10; } }
		else v = v*10+(*s-'0');
		if(v>100000) return 0;
	}
	if(dot||v<100) return v*1000+frac*1000/scale;
	return v;
}

static void help(void){
	static const char* const h[] = {
		T("info | list | get <name|number> | set <name|number> <value|pin|none> | save | revert | defaults"),
		T("detect hall [mV] | vbat <V> (twice, 2 levels) | current [mV] [shunt mOhm] | imu | button [ms] | charger [ms] | ir [ms] | latch"),
		T("calibrate hall [mV] | poles [ms] | rl [mV] | kv block|sine [max] | imu"),
		T("adc | hall | pin <pin> high|low|pullup|pulldown|input|read|blink|tone [ms]"),
		T("step <1-6|101-112> [mV] [ms] | rotate <permille> [ms] | poweroff | tui | agent"),
	};
	for(uint8_t i=0;i<sizeof(h)/sizeof(h[0]);i++){
		if(agent) out("# ");
		out(h[i]);
		nl();
	}
}

static void setWord(const char* name, const char* val){
	int16_t idx = CfgIndex(name);
	if(idx<0){ err("unknown name"); return; }
	if(!val){ err("value missing"); return; }
	uint8_t g;
	int32_t v;
	uint16_t pin = PinByName(val);
	if(eq(val, "none")) v = PIN_NONE;
	else if(pin!=PIN_NONE) v = pin;
	else{
		v = num(val, &g);
		if(!g||v<0||v>65535){ err("value: 0..65535, a pin or none"); return; }
		if(CfgIsPin((uint8_t)idx)&&v>=PINCOUNT&&v!=PIN_NONE){ err("not a pin"); return; }
	}
	if(CfgIsPin((uint8_t)idx)&&v<PINCOUNT){    //one role per pin: the word that had it is cleared
		static const uint8_t ownWord[OWN_PINS] = { 0xFF, 0xFF, 14, 15, 10 };    //only tx, rx, latch may name their own pin
		for(uint8_t k=0;k<OWN_PINS;k++) if(ownPins[k]==v&&ownWord[k]!=idx){ err("reserved: SWD, uart, latch"); return; }
		for(int16_t i;(i=CfgPinWord((uint16_t)v, (uint8_t)idx))>=0;){ pinstorage[i] = PIN_NONE; CfgPrint((uint8_t)i); }
	}
	pinstorage[idx] = (uint16_t)v;
	if(&pinstorage[idx]==&INVERT_LOWSIDE) MotorInit();    //the gate polarity at once (TIM1_init reads it)
	if(&pinstorage[idx]==&LATCHPIN) LatchHold();    //a latch set by hand is driven at once (none: pull-ups again)
	CfgPrint((uint8_t)idx);
	ok();
}

static void pinCmd(const char* name, const char* mode, const char* arg){
	uint16_t p = name ? PinByName(name) : PIN_NONE;
	if(p==PIN_NONE){ err("pin name, e.g. PB9"); return; }
	int16_t w = CfgPinWord(p, 0xFF);    //its role (set keeps one per pin)
	if(!mode||eq(mode, "read")){    //any pin, its mode unchanged: level, the voltage on adc pins, what it is
		kvi(PinName(p), digitalRead((uint8_t)p));
		int32_t mv = PinMv(p);    //free and analog pins only: not on pins the board drives or reads digitally
		if(mv>=0) kvmv("pin", mv);
		if(w>=0) kvs("used_as", CfgName((uint8_t)w));
		ok();
		return;
	}
	if(!PinFree(p)&&!(w>=3&&w<=8)&&w!=11){    //in use, but not an LED, the buzzer, the charger pin
		err("pin in use: read only");
		return;
	}
	uint32_t ms = optnum(arg);
	if(eq(mode, "high")){ pinMode((uint8_t)p, OUTPUT); digitalWrite((uint8_t)p, 1); }
	else if(eq(mode, "low")){ pinMode((uint8_t)p, OUTPUT); digitalWrite((uint8_t)p, 0); }
	else if(eq(mode, "pullup")) pinMode((uint8_t)p, INPUT_PULLUP);
	else if(eq(mode, "pulldown")) pinMode((uint8_t)p, INPUT_PULLDOWN);
	else if(eq(mode, "input")) pinMode((uint8_t)p, INPUT);
	else if(eq(mode, "tone")) Tone(p, ms ? ms : 500, 2000);
	else if(eq(mode, "blink")){    //pull-up / pull-down, not driven: safe on a pin of unknown purpose
		for(uint32_t t=millis;millis-t<(ms ? ms : 2000)&&!aborted();){
			pinMode((uint8_t)p, (millis/150)&1 ? INPUT_PULLUP : INPUT_PULLDOWN);
		}
		PinRest(p);
	}
	else{ err("pin mode? (help)"); return; }
	ok();
}

void Command(char* line){
	char* argv[5];
	uint8_t argc = 0;
	for(char* s=line;*s&&argc<5;){
		while(*s==' ') s++;
		if(!*s) break;
		argv[argc++] = s;
		while(*s&&*s!=' ') s++;
		if(*s) *s++ = 0;
	}
	if(!argc) return;
	abortKey = 0;    //aborted() sets it: a stop by a key, not a timeout
	for(char* s=argv[0];*s;s++) if(*s>='A'&&*s<='Z') *s += 32;
	const char* c = argv[0];
	const char* a1 = argc>1 ? argv[1] : 0;
	const char* a2 = argc>2 ? argv[2] : 0;
	const char* a3 = argc>3 ? argv[3] : 0;
	if(agent){ out("> "); out(line); for(uint8_t i=1;i<argc;i++){ outc(' '); out(argv[i]); } nl(); }
	if(eq(c, "help")||eq(c, "?")){ help(); ok(); }
	else if(eq(c, "info")){ Info(); ok(); }
	else if(eq(c, "list")){ for(uint8_t i=0;i<128;i++) if(CfgName(i)) CfgPrint(i); ok(); }
	else if(eq(c, "get")){
		int16_t idx = a1 ? CfgIndex(a1) : -1;
		if(idx<0) err("unknown name"); else{ CfgPrint((uint8_t)idx); ok(); }
	}
	else if(eq(c, "set")) setWord(a1, a2);
	else if(eq(c, "save")){ HwCheck(); CfgSave(); note("saved to flash: the main firmware boots with it"); ok(); }
	else if(eq(c, "revert")){ if(ConfigLoad()) ConfigMigrate(); else CfgDefaults(); CfgMarkSaved(); ok(); }
	else if(eq(c, "defaults")){ CfgDefaults(); note("defaults in RAM (save keeps them)"); ok(); }
	else if(eq(c, "detect")||eq(c, "calibrate")){
		const char* w = a1 ? a1 : "";
		if(c[0]=='d'){
			if(eq(w, "hall")) DetectHall((int32_t)optnum(a2));
			else if(eq(w, "vbat")) DetectVbat(millivolts(a2));
			else if(eq(w, "current")) DetectCurrent((int32_t)optnum(a2), millivolts(a3));    //shunt mOhm -> uOhm
			else if(eq(w, "imu")) DetectImu();
			else if(eq(w, "button")) DetectButton(optnum(a2));
			else if(eq(w, "charger")) DetectCharger(optnum(a2));
			else if(eq(w, "ir")) DetectIr(optnum(a2));
			else if(eq(w, "latch")) LatchFind();
			else err("detect what? (help)");
		}else{
			if(eq(w, "hall")) CalHall((int32_t)optnum(a2));
			else if(eq(w, "poles")) CalPoles(optnum(a2));
			else if(eq(w, "rl")) CalRL((int32_t)optnum(a2));
			else if(eq(w, "kv")) CalKv(a2, (int32_t)optnum(a3));
			else if(eq(w, "imu")) CalImu();
			else err("calibrate what? (help)");
		}
	}
	else if(eq(c, "adc")){ ShowAdc(); ok(); }
	else if(eq(c, "hall")){
		kvi("hall", HallState());
		kvi("a", digitalRead(HALLAPIN)); kvi("b", digitalRead(HALLBPIN)); kvi("c", digitalRead(HALLCPIN));
		ok();
	}
	else if(eq(c, "pin")) pinCmd(a1, a2, a3);
	else if(eq(c, "step")) StepHold((uint8_t)optnum(a1), (int32_t)optnum(a2), optnum(a3));
	else if(eq(c, "rotate")){
		uint8_t g;
		int32_t pm = num(a1, &g);
		if(!g) err("rotate <permille> [ms]"); else Rotate(pm, optnum(a2));
	}
	else if(eq(c, "poweroff")) PowerOff();
	else if(eq(c, "tui")){ agent = 0; TuiStart(); }
	else if(eq(c, "agent")){ agent = 1; ok(); }
	else err("unknown command (help)");
}
