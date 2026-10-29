//The TUI: one screen with the pin map (unsaved changes in yellow), the board status and the menu. The
//detections print below it; a key afterwards draws the screen again. Without ANSI (the terminal did not
//answer the status request) the same text is printed without colours and screen clearing.
#include "pf.h"
#include "main.h"
#include "sim_eeprom.h"
#include <string.h>

enum{ SCR_MENU, SCR_AFTER, SCR_LINE, SCR_LED, SCR_ROTATE };
static uint8_t screen;
uint8_t tuiLine;
static char line[48];
static uint8_t len;
static uint16_t sel;          //LED screen: the pin being tried
static uint8_t ledLow;        //LED screen: the other free pins floating (else pulled up)
static uint32_t lastTick;
static int32_t rotPm;

static void ledLine(void){
	out("\r  trying ");
	color(1);
	out(PinName(sel));
	color(0);
	out("    ");
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

static void mapCell(uint8_t idx){
	out(" ");
	const char* n = CfgName(idx);
	out(n);
	for(uint8_t k=zlen(n);k<9;k++) outc(' ');
	if(CfgWordChanged(idx)) color(33);    //unsaved
	if(CfgIsPin(idx)){
		out(PinName(pinstorage[idx]));
		for(uint8_t k=(uint8_t)strlen(PinName(pinstorage[idx]));k<8;k++) outc(' ');
	}else{
		outi(pinstorage[idx]);
		out("     ");
	}
	color(0);
}

static void after(void);
static void words(void){    //the named config words that are not pins, 3 columns; unsaved changes yellow
	heading("config words (set <name> <value> on the command line c)");
	uint8_t col = 0;
	for(uint8_t i=0;i<128;i++){
		const char* n = CfgName(i);
		if(!n||CfgIsPin(i)) continue;
		out(" ");
		out(n);
		for(uint8_t k=zlen(n);k<14;k++) outc(' ');
		if(CfgWordChanged(i)) color(33);
		if(pinstorage[i]==0xFFFF) out("-     ");
		else{ outi(pinstorage[i]); for(uint16_t v=pinstorage[i],k=1;k<6;k++,v/=10) if(v<10) outc(' '); }
		color(0);
		if(++col==3){ col = 0; nl(); }
	}
	if(col) nl();
	after();
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
	if(CfgChanged()){ color(33); out("   unsaved changes (s)"); color(0); }
	nl();
	rule();
	for(uint8_t r=0;r<5;r++){
		for(uint8_t c=0;c<4;c++) mapCell(map[r][c]);
		nl();
	}
	rule();
	out(" 1 hall  2 LEDs/buzzer  3 battery  4 button  5 charger"); nl();
	out(" 6 latch  7 slave id  8 test motor  9 power off  a all"); nl();
	out(" calibrate: h hall  p poles  m R/L  k kv  d current  g imu"); nl();
	out(" c command  w words  s save  r revert  i info  ? help  @ agent"); nl();
	out("> ");
}

static void after(void){    //an action printed its output: the next key redraws
	screen = SCR_AFTER;
	color(36);
	out("(any key)");
	color(0);
}

static void lineStart(const char* pre){
	screen = SCR_LINE;
	tuiLine = 1;
	strcpy(line, pre);
	len = (uint8_t)strlen(pre);
	nl();
	out("pinfinder> ");
	out(line);
}

static void ledNext(int8_t d){
	if(ledLow) pinMode((uint8_t)sel, INPUT); else pinMode((uint8_t)sel, INPUT_PULLUP);
	for(uint8_t n=0;n<PINCOUNT;n++){
		sel = (uint16_t)((sel+PINCOUNT+d)%PINCOUNT);
		if(PinFree(sel)) break;
	}
	ledLine();
}

static void ledStart(void){
	screen = SCR_LED;
	heading("LEDs and buzzer");
	out("The other free pins are "); out(ledLow ? "floating" : "pulled up (LEDs on)"); out(", the tried pin blinks."); nl();
	out(" w/s next/prev pin  r g b red/green/blue LED  u l upper/lower LED"); nl();
	out(" z buzzer  p passive buzzer  t tone"); nl();
	out(" m other pins on/off   Enter back"); nl();
	for(uint16_t p=0;p<PINCOUNT;p++){
		if(PinFree(p)) pinMode((uint8_t)p, ledLow ? INPUT : INPUT_PULLUP);
	}
	if(!PinFree(sel)) ledNext(1); else ledLine();
}

static void ledKey(int c){
	static const char keys[] = "rgbulz";
	static const uint8_t words[] = { 3, 4, 5, 6, 7, 8 };
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
	if(pinstorage[w]<PINCOUNT) pinMode((uint8_t)pinstorage[w], ledLow ? INPUT : INPUT_PULLUP);    //the old one is free again
	pinstorage[w] = sel;
	pinMode((uint8_t)sel, INPUT);    //saved pins stay off: easy to see which LEDs are left
	nl();
	kvp(CfgName(w), sel);
	if(w==8){ kvi("buzzerpassive", BUZZER_PASSIVE); HwCheck(); }
	ledNext(1);
}

static void rotateStart(void){
	if(HALLAPIN>=PINCOUNT||HALLBPIN>=PINCOUNT||HALLCPIN>=PINCOUNT){ err("hall pins not set: detect hall first"); after(); return; }
	screen = SCR_ROTATE;
	rotPm = 0;
	RotateStop();
	heading("test motor (hall commutation)");
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
		if(k==3&&!master) continue;
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

void TuiKey(int c){
	switch(screen){
	case SCR_LINE:
		if(c=='\r'||c=='\n'){
			line[len] = 0;
			nl();
			if(!len){ draw(); return; }
			if(eq(line, "x")||eq(line, "exit")){ draw(); return; }
			Command(line);
			if(agent||screen!=SCR_LINE) return;    //"agent" / "tui" switched the front end
			lineStart("");
		}else if(c=='\b'||c==127){
			if(len){ len--; out("\b \b"); }
		}else if(c==0x1b){
			draw();
		}else if(c>=' '&&len<sizeof(line)-1){
			line[len++] = (char)c;
			outc((char)c);
		}
		return;
	case SCR_LED: ledKey(c); return;
	case SCR_ROTATE: rotateKey(c); return;
	case SCR_AFTER: draw(); return;
	default: break;
	}
	nl();
	switch(c){
	case '1': heading("hall sensors"); DetectHall(0); after(); break;
	case '2': ledStart(); break;
	case '3':
		heading("battery voltage");
		ShowAdc();
		out("Battery voltage you measure (36.5), or Enter: the pin at 20..45 V"); nl();
		lineStart("detect vbat ");
		break;
	case '4': heading("power button"); DetectButton(0); after(); break;
	case '5': heading("charger"); DetectCharger(0); after(); break;
	case '6': heading("power latch"); LatchFind(); after(); break;
	case '7': lineStart("set slaveid "); break;
	case '8': rotateStart(); break;
	case '9': PowerOff(); after(); break;
	case 'a': detectAll(); break;
	case 'c': lineStart(""); break;
	case 'w': words(); break;
	case 'h': heading("hall edge angles"); CalHall(0); after(); break;
	case 'p': heading("pole count"); CalPoles(0); after(); break;
	case 'm': heading("phase resistance and inductance"); CalRL(0); after(); break;
	case 'k': heading("speed constants"); CalKv("block", 0); CalKv("sine", 0); after(); break;
	case 'g': heading("IMU mounting"); CalImu(); after(); break;
	case 'd': lineStart("detect current 1000 "); out("  (+ shunt mOhm if known, e.g. 3)"); break;
	case 's': HwCheck(); CfgSave(); note("saved"); after(); break;
	case 'r': if(ConfigLoad()) ConfigMigrate(); else CfgDefaults(); CfgMarkSaved(); draw(); break;
	case 'i': heading("info"); Info(); after(); break;
	case '?': heading("commands (c)"); Command((char[]){"help"}); after(); break;
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
