//The TUI: one screen with the pin map (unsaved changes in yellow), the board status and the menu. The
//detections print below it; a key afterwards draws the screen again. Without ANSI (the terminal did not
//answer the status request) the same text is printed without colours and screen clearing.
//Two prompts, told apart by their text: the menu ("press a key:") takes single keys, the command line
//("command>") a line of agent commands. Guided questions (battery voltage, shunt) are a command line with
//a hidden command in front of the answer.
#include "pf.h"
#include "main.h"
#include "sim_eeprom.h"
#include <string.h>

enum{ SCR_MENU, SCR_AFTER, SCR_LINE, SCR_LED, SCR_ROTATE };
static uint8_t screen;
uint8_t tuiLine;
static char line[48];
static uint8_t len, lineMin;  //line[0..lineMin): the hidden command of a guided question
static uint8_t lineEmptyRuns; //a guided question whose empty answer runs the command too
static uint16_t sel;          //LED screen: the pin being tried
static uint8_t ledLow;        //LED screen: the other free pins floating (else pulled up)
static uint32_t lastTick;
static int32_t rotPm;

static int8_t ledRole(uint16_t p){    //the LED / buzzer word (3-8) the pin is set in, -1: none
	for(uint8_t w=3;w<=8;w++) if(pinstorage[w]==p) return (int8_t)w;
	return -1;
}

static void ledLine(void){
	out("\r  trying ");
	color(1);
	out(PinName(sel));
	color(0);
	int8_t w = ledRole(sel);
	out(" = ");
	out(w<0 ? "none" : CfgName((uint8_t)w));
	out("        ");
}

static void rule(void){
	outc(' ');
	for(uint8_t i=0;i<72;i++) outc('-');
	nl();
}

static void heading(const char* s){
	nl();
	color(1);
	out(s);
	color(0);
	nl();
}

static void pad(uint8_t n, uint8_t w){ for(;n<w;n++) outc(' '); }

static void cell(uint8_t idx, uint8_t w){    //a config word's value, yellow while unsaved, padded to w
	if(CfgWordChanged(idx)) color(33);
	uint16_t v = pinstorage[idx];
	const char* s = CfgIsPin(idx) ? PinName(v) : (v==0xFFFF ? "-" : 0);
	if(s){ out(s); pad((uint8_t)strlen(s), w); }
	else{ outi(v); for(uint16_t k=1;k<w;k++,v/=10) if(v<10) outc(' '); }
	color(0);
}

static void mapCell(uint8_t idx){
	out(" ");
	const char* n = CfgName(idx);
	out(n);
	pad(zlen(n), 9);
	cell(idx, 8);
}

static void lineStart(const char* prompt, const char* pre, uint8_t emptyRuns){
	screen = SCR_LINE;
	tuiLine = 1;
	zcpy(line, pre);
	len = lineMin = (uint8_t)strlen(line);
	lineEmptyRuns = emptyRuns;
	nl();
	color(36);
	out(prompt);
	color(0);
}
static void commandLine(void){ lineStart(T("command> "), T(""), 0); }

static void after(void){    //an action printed its output: the next key redraws
	screen = SCR_AFTER;
	color(36);
	out("(any key: menu)");
	color(0);
}

//every named config word with its number, 3 columns; unsaved changes yellow. Then the command line
static void words(void){
	heading("config words: number, name, value (- = not set: the default)");
	uint8_t col = 0;
	for(uint8_t i=0;i<128;i++){
		const char* n = CfgName(i);
		if(!n) continue;
		out(i<10 ? "   " : (i<100 ? "  " : " "));
		outi(i);
		outc(' ');
		out(n);
		pad(zlen(n), 14);
		cell(i, 6);
		if(++col==3){ col = 0; nl(); }
	}
	if(col) nl();
	note("read: get <name|number>   write: set <name|number> <value|pin|none>   s / save: to flash");
	note("Enter on an empty line or Esc: back to the menu");
	commandLine();
}

static void draw(void){
	static const uint8_t map[5][4] = {
		{ 0, 3, 9, 12 }, { 1, 4, 10, 13 }, { 2, 5, 11, 16 }, { 14, 6, 8, 17 }, { 15, 7, 23, 38 },
	};
	screen = SCR_MENU;
	tuiLine = 0;
	if(ansi) out("\x1b[2J\x1b[H");
	else nl();
	color(1);
	out(" PinFinder " PF_VERSION);
	color(0);
	out(master ? "   master" : "   slave");
	out("   VDD ");
	outfix(VddMv()/10, 2);
	out(" V");
	if(VBATPIN<PINCOUNT){ out("   battery "); outfix(VbatMv()/10, 2); out(" V"); }
	if(CfgChanged()){ color(33); out("   unsaved (s = save)"); color(0); }
	nl();
	rule();
	for(uint8_t r=0;r<5;r++){
		for(uint8_t c=0;c<4;c++) mapCell(map[r][c]);
		nl();
	}
	rule();
	out(" detect     1 hall sensors  2 LEDs/buzzer  3 battery  4 power button  5 charger"); nl();
	out("            6 power latch  7 IMU  8 foot sensors  9 current sensing  a all"); nl();
	out(" calibrate  h hall angles  p poles  m R and L  k speed (kv)  g IMU mounting"); nl();
	out(" motor      t test run  o power off"); nl();
	out(" config     w words (read / write)  c command line  s save  r revert  i info  ? help"); nl();
	color(36);
	out("press a key: ");
	color(0);
}

static void ledNext(int8_t d){
	uint8_t rest = ledLow ? INPUT : INPUT_PULLUP;
	pinMode((uint8_t)sel, ledRole(sel)<0 ? rest : INPUT);    //set pins stay off: easy to see which LEDs are left
	for(uint8_t n=0;n<PINCOUNT;n++){
		sel = (uint16_t)((sel+PINCOUNT+d)%PINCOUNT);
		if(PinFree(sel)||ledRole(sel)>=0) break;    //free pins and the ones already set here (to change them)
	}
	ledLine();
}

static void ledStart(void){
	screen = SCR_LED;
	heading("LEDs and buzzer");
	out("The other free pins are "); out(ledLow ? "floating" : "pulled up (LEDs on)"); out(", the tried pin blinks."); nl();
	out(" w/s next/prev pin  r g b red/green/blue LED  u l upper/lower LED"); nl();
	out(" z buzzer  p passive buzzer  t tone  n none (clear)"); nl();
	out(" m other pins on/off   Enter back"); nl();
	for(uint16_t p=0;p<PINCOUNT;p++){
		if(PinFree(p)) pinMode((uint8_t)p, ledLow ? INPUT : INPUT_PULLUP);
	}
	if(!PinFree(sel)&&ledRole(sel)<0) ledNext(1); else ledLine();
}

static void ledKey(int c){
	static const char keys[] = "rgbulzn";
	static const uint8_t words[] = { 3, 4, 5, 6, 7, 8, 0 };
	if(c=='w'){ ledNext(1); return; }
	if(c=='s'){ ledNext(-1); return; }
	if(c=='m'){ ledLow = !ledLow; ledStart(); return; }
	if(c=='t'){ Tone(sel, 600, 2000); pinMode((uint8_t)sel, ledLow ? INPUT : INPUT_PULLUP); return; }
	if(c=='p'){ BUZZER_PASSIVE = 2; c = 'z'; }
	else if(c=='z') BUZZER_PASSIVE = 1;
	if(c=='\r'||c=='\n'){
		for(uint16_t p=0;p<PINCOUNT;p++) if(PinFree(p)) pinMode((uint8_t)p, INPUT);
		draw();
		return;
	}
	const char* k = strchr(keys, c);
	if(!k||!c) return;
	uint8_t w = words[k-keys];
	int8_t old = ledRole(sel);
	nl();
	if(old>=0&&old!=w){ pinstorage[old] = PIN_NONE; kvp(CfgName((uint8_t)old), PIN_NONE); }    //its old role
	if(!w){ ledLine(); return; }
	if(pinstorage[w]<PINCOUNT) pinMode((uint8_t)pinstorage[w], ledLow ? INPUT : INPUT_PULLUP);    //the old pin is free again
	pinstorage[w] = sel;
	kvp(CfgName(w), sel);
	if(w==8){ kvi("buzzerpassive", BUZZER_PASSIVE); HwCheck(); }
	ledNext(1);
}

static void rotateStart(void){
	if(HALLAPIN>=PINCOUNT||HALLBPIN>=PINCOUNT||HALLCPIN>=PINCOUNT){ err("hall pins not set: detect hall first"); after(); return; }
	screen = SCR_ROTATE;
	rotPm = 0;
	RotateStop();
	heading("test run (hall commutation)");
	out(" +/- duty 5 % steps (negative: back)  0 stop  Enter back"); nl();
}

static void rotateKey(int c){
	if(c=='+'||c=='=') rotPm += 50;
	else if(c=='-') rotPm -= 50;
	else if(c=='0') rotPm = 0;
	else if(c=='\r'||c=='\n'){ RotateStop(); draw(); return; }
	if(rotPm>1000) rotPm = 1000;
	if(rotPm<-1000) rotPm = -1000;
	out("\r  duty ");
	outfix(rotPm/10, 0);
	out(" %   hall ");
	outi(HallState());
	out("      ");
}

//every detection in a row; a key stops the running one (skipped), Esc the rest too. A summary of what
//passed, failed or was skipped at the end (a failed step used to scroll past), then the LED screen.
static void detectAll(void){
	static const char* const names[6] = { T("hall sensors"), T("current sensing"), T("battery voltage"),
		T("power button"), T("charger (any key skips)"), T("IMU") };
	uint8_t run = 0, good = 0, skip = 0, esc = 0;
	for(uint8_t k=0;k<6&&!esc;k++){
		if((k==3&&!master)||k==2) continue;    //battery: two voltage levels the user sets (menu 3)
		heading(names[k]);
		failed = 0;
		abortKey = 0;
		switch(k){
			case 0: DetectHall(0); break;
			case 1: DetectCurrent(0, 0); break;
			case 2: DetectVbat(0); break;
			case 3: DetectButton(0); break;
			case 4: DetectCharger(0); break;
			default: DetectImu(); break;
		}
		run |= (uint8_t)(1<<k);
		if(!failed) good |= (uint8_t)(1<<k);
		else if(abortKey) skip |= (uint8_t)(1<<k);    //stopped by a key
		esc = abortKey==0x1b;
	}
	heading("summary");
	for(uint8_t k=0;k<6;k++){
		if(!(run>>k&1)) continue;
		uint8_t g = good>>k&1, sk = skip>>k&1;
		color(g ? 32 : (sk ? 33 : 31));
		out(g ? "  ok      " : (sk ? "  skipped " : "  FAILED  "));
		color(0);
		out(names[k]); nl();
	}
	if(esc){ note("stopped (Esc)"); after(); return; }
	ledStart();
}

void TuiStart(void){
	ansi = 0;
	out("\x1b[5n\r     \r");    //device status request: an ANSI terminal answers ESC [ 0 n (others show "[5n": overwritten)
	for(uint32_t t=millis;millis-t<300;){
		int c = inbyte();
		if(c==0x1b) ansi = 1;
		if(ansi&&c=='n') break;
	}
	draw();
}

//menu keys that run one command: key, then the command ('\0' separated, in the same order)
static const char runKeys[] = "145678hpmgsri?";
static const char runCmds[] = T("detect hall\0detect button\0detect charger\0detect latch\0detect imu\0detect ir\0"
	"calibrate hall\0calibrate poles\0calibrate rl\0calibrate imu\0save\0revert\0info\0help");

static void lineKey(int c){
	if(c=='\r'||c=='\n'){
		line[len] = 0;
		nl();
		if((len==lineMin&&!lineEmptyRuns)||eq(line, "x")||eq(line, "exit")){ draw(); return; }
		uint8_t guided = lineMin!=0;
		Command(line);
		if(agent||screen!=SCR_LINE) return;    //"agent" / "tui" switched the front end
		if(guided){
			//battery: the first level is stored, the second one comes after the user changed the voltage
			if(vbLevelMv&&!failed){ lineStart(T("the new voltage, V: "), T("detect vbat "), 0); return; }
			after();
			return;
		}
		commandLine();
	}else if(c=='\b'||c==127){
		if(len>lineMin){ len--; out("\b \b"); }
	}else if(c==0x1b){
		draw();
	}else if(c>=' '&&len<sizeof(line)-1){
		line[len++] = (char)c;
		outc((char)c);
	}
}

void TuiKey(int c){
	switch(screen){
	case SCR_LINE: lineKey(c); return;
	case SCR_LED: ledKey(c); return;
	case SCR_ROTATE: rotateKey(c); return;
	case SCR_AFTER: draw(); return;
	default: break;
	}
	nl();
	const char* k = c ? strchr(runKeys, c) : 0;
	if(k){
		const char* s = runCmds;
		for(uint8_t i=0;i<k-runKeys;i++) s += strlen(s)+1;
		zcpy(line, s);
		heading(s);
		failed = 0;
		Command(line);
		if(c=='r'){ draw(); return; }
		if(c=='s'&&!failed) note("saved to flash: the main firmware boots with it");
		after();
		return;
	}
	switch(c){
	case '2': ledStart(); break;
	case '3':
		heading("battery voltage: two voltage levels find the pin");
		ShowAdc();
		failed = 0;
		lineStart(T("battery voltage you measure now, V (e.g. 36.5): "), T("detect vbat "), 0);
		break;
	case '9':
		heading("current sensing: a few A static current, the wheel may move");
		lineStart(T("shunt resistor mOhm if you can read it (R003 = 3), Enter if not: "), T("detect current 1000 "), 1);
		break;
	case 't': rotateStart(); break;
	case 'o': PowerOff(); after(); break;
	case 'a': detectAll(); break;
	case 'c':
		note("agent commands (? = list); get / set read and write config words; Enter on an empty line or Esc: menu");
		commandLine();
		break;
	case 'w': words(); break;
	case 'k':
		heading("speed constants (the wheel spins up fast: keep it free)"); CalKv("block", 0); CalKv("sine", 0); after(); break;
	default: draw(); break;
	}
}

void TuiPoll(void){
	if(screen==SCR_LED&&millis-lastTick>150){
		lastTick = millis;
		pinMode((uint8_t)sel, (millis/150)&1 ? INPUT_PULLUP : (ledLow ? INPUT_PULLDOWN : INPUT));
	}
	if(screen==SCR_ROTATE) RotateRun(rotPm);
}
