//Serial output and result formatting. Output is blocking (at 115200 a full TUI screen takes ~0.2 s, the
//detections wait for the motor anyway); input is the circular DMA ring of uart.c.
#include "pf.h"
#include "uart.h"
#include "main.h"

uint8_t agent;
uint8_t failed;    //err() since the caller cleared it
int abortKey;      //the key that stopped the last operation (aborted())
uint8_t ansi;
static UART_TypeDef* U;

void IoInit(uint8_t uart){
	U = uart==2 ? UART2 : UART1;
}

static uint8_t mute;    //TUI: a debug line (kvd) is not printed, up to its nl()

void outc(char c){
	if(mute) return;
	uint32_t t = millis;
	while((U->CSR&UART_CSR_TXFULL)&&millis-t<5);
	U->TDR = (uint8_t)c;
}

//messages are byte pair encoded at build time (tools/pfstrings.py): a code byte stands for a pair of codes
//or characters. Codes: 0x80-0xFF (pairs 0-127), 0x0E-0x1A (128-140), 0x01-0x06 (141-146): control
//characters no message uses (plain strings, e.g. pin names and the echoed input, never contain them)
extern const uint8_t pfPairs[][2];
static const uint8_t* pairOf(uint8_t c){    //0: a plain character
	if(c>=0x80) return pfPairs[c-0x80];
	if(c>=14&&c<27) return pfPairs[c+114];    //0x0E-0x1A: 128-140
	if(c&&c<7) return pfPairs[c+140];
	return 0;
}

static char* zdst;    //zcpy: expand into memory instead of the uart
static void outz(uint8_t c){
	const uint8_t* p;
	while((p = pairOf(c))){
		outz(p[0]);
		c = p[1];
	}
	if(zdst) *zdst++ = (char)c;
	else outc((char)c);
}

void out(const char* s){
	while(*s) outz((uint8_t)*s++);
}

void zcpy(char* d, const char* s){
	zdst = d;
	out(s);
	*zdst = 0;
	zdst = 0;
}

static const char* zin;
static uint8_t zeq(uint8_t c){    //expands c against the input at zin
	const uint8_t* p = pairOf(c);
	if(p) return zeq(p[0])&&zeq(p[1]);
	return *zin++==c;
}

uint8_t eq(const char* in, const char* name){
	zin = in;
	while(*name) if(!zeq((uint8_t)*name++)) return 0;
	return !*zin;
}

static uint8_t zn(uint8_t c){
	const uint8_t* p = pairOf(c);
	return p ? (uint8_t)(zn(p[0])+zn(p[1])) : 1;
}

uint8_t zlen(const char* s){
	uint8_t n = 0;
	while(*s) n = (uint8_t)(n+zn((uint8_t)*s++));
	return n;
}

void outi(int32_t v){
	char b[12];
	uint8_t n = 0;
	uint32_t u = v<0 ? 0u-(uint32_t)v : (uint32_t)v;
	if(v<0) outc('-');
	do{ b[n++] = (char)('0'+u%10); u /= 10; }while(u);
	while(n) outc(b[--n]);
}

void outfix(int32_t v, uint8_t dec){
	int32_t p = 1;
	for(uint8_t i=0;i<dec;i++) p *= 10;
	if(v<0){ outc('-'); v = -v; }
	outi(v/p);
	if(!dec) return;
	outc('.');
	int32_t f = v%p;
	for(p/=10;p>1&&f<p;p/=10) outc('0');
	outi(f);
}

void outpin(uint16_t pin){
	out(pin<PINCOUNT ? PinName(pin) : "none");
}

void nl(void){
	out("\r\n");
	mute = 0;
}

void color(uint8_t c){
	if(agent||!ansi) return;
	out("\x1b[");
	outi(c);
	outc('m');
}

int inbyte(void){
	return SerialGetByte();
}

uint8_t aborted(void){
	int c = inbyte();
	if(c<0) return 0;
	abortKey = c;
	while(c>=0&&c!='\n'&&c!='\r'){    //the rest of an agent's "abort" line
		uint32_t t = millis;
		while((c = inbyte())<0&&millis-t<20);
	}
	return 1;
}

static uint8_t dots;    //TUI: progress dots on the current line (a result starts a new one)
static void endDots(void){    //the dots are a progress display: the result line takes their place
	if(dots) out(ansi ? "\r\x1b[K" : "\r\n");
	dots = 0;
}

void kv(const char* key){
	endDots();
	if(agent){
		out(key);
		outc('=');
	}else{
		out("  ");
		out(key);
		out(": ");
	}
}

void kvi(const char* key, int32_t v){ kv(key); outi(v); nl(); }
void kvd(const char* key){ endDots(); mute = !agent; kv(key); }
void kvdi(const char* key, int32_t v){ kvd(key); outi(v); nl(); }
void kvp(const char* key, uint16_t pin){ kv(key); outpin(pin); nl(); }
void kvs(const char* key, const char* s){ kv(key); out(s); nl(); }

void kvmv(const char* key, int32_t mv){
	if(agent){
		out(key);
		out("_mv=");
		outi(mv);
	}else{
		kv(key);
		outfix(mv/10, 2);
		out(" V");
	}
	nl();
}

void note(const char* s){
	endDots();
	if(agent) out("# ");
	else color(36);
	out(s);
	color(0);
	nl();
}

void warn(const char* s){
	endDots();
	if(agent){
		out("warn ");
	}else{
		color(33);
		out("\awarning: ");
	}
	out(s);
	color(0);
	nl();
}

void progress(int32_t n, int32_t total){
	if(agent){
		out("progress ");
		outi(n);
		outc('/');
		outi(total);
		nl();
	}else{
		outc('.');
		dots = 1;
	}
}

void progressEnd(void){
	endDots();
}

void ok(void){
	if(agent){
		out("ok");
		nl();
	}
}

void err(const char* s){
	failed = 1;
	endDots();
	if(agent){
		out("err ");
	}else{
		color(31);
		out("\aerror: ");
	}
	out(s);
	color(0);
	nl();
}
