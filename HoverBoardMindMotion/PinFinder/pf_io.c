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

void outc(char c){
	uint32_t t = millis;
	while((U->CSR&UART_CSR_TXFULL)&&millis-t<5);
	U->TDR = (uint8_t)c;
}

//messages are byte pair encoded at build time (tools/pfstrings.py): 0x80 + k stands for pfPairs[k]
extern const uint8_t pfPairs[][2];
static void outz(uint8_t c){
	while(c>=0x80){
		outz(pfPairs[c-0x80][0]);
		c = pfPairs[c-0x80][1];
	}
	outc((char)c);
}

void out(const char* s){
	while(*s) outz((uint8_t)*s++);
}

static const char* zin;
static uint8_t zeq(uint8_t c){    //expands c against the input at zin
	if(c>=0x80) return zeq(pfPairs[c-0x80][0])&&zeq(pfPairs[c-0x80][1]);
	return *zin++==c;
}

uint8_t eq(const char* in, const char* name){
	zin = in;
	while(*name) if(!zeq((uint8_t)*name++)) return 0;
	return !*zin;
}

static uint8_t zn(uint8_t c){
	return c>=0x80 ? (uint8_t)(zn(pfPairs[c-0x80][0])+zn(pfPairs[c-0x80][1])) : 1;
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

void kv(const char* key){
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
	if(agent) out("# ");
	else color(36);
	out(s);
	color(0);
	nl();
}

void warn(const char* s){
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
	}
}

void progressEnd(void){
	if(!agent) nl();
}

void ok(void){
	if(agent){
		out("ok");
		nl();
	}
}

void err(const char* s){
	failed = 1;
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
