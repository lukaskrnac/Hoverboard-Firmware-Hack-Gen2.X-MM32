//Single precision soft float in place of libgcc's: its Cortex-M0 build is the generic soft-fp C (add
//1.1 KB, multiply 0.7 KB, divide 0.6 KB, compares and conversions 1 KB); this one is ~0.8 KB. Floats
//are only used by the CANSimple protocol (physical units) and the input modes, in the main loop, never
//in the adc isr: speed does not matter here.
//
//IEEE 754 results with round to nearest even, as libgcc, except subnormal numbers (below 1.2e-38): read
//as 0 and results that small flushed to 0 (none of the firmware's values comes near). Infinities as
//IEEE; a NaN operand gives that NaN (quiet), an invalid operation (inf - inf, 0 * inf, 0 / 0) the default
//NaN 0x7FC00000, as libgcc's ARM soft-fp. Conversions to integer truncate and saturate like libgcc's:
//f2iz gives INT_MAX / INT_MIN (by the sign bit) at or above 2^31 and for NaN, f2uiz is f2iz of the value
//(x - 2^31 above 2^31, plus 2^31). Checked bit for bit against the host's IEEE arithmetic (DAZ / FTZ).
//
//All integer arithmetic: a float operation in here would call itself.
#include <stdint.h>
#include <string.h>

#define KEEP __attribute__((used, externally_visible))    //the calls appear after LTO: keep them
#define SIGN 0x80000000u
#define DEFAULT_NAN 0x7FC00000u

static uint32_t bits(float f){ uint32_t u; memcpy(&u, &f, 4); return u; }
static float flt(uint32_t u){ float f; memcpy(&f, &u, 4); return f; }
static uint32_t expo(uint32_t a){ return a<<1>>24; }
static uint32_t mant(uint32_t a){ return (a&0x7FFFFFu)|0x800000u; }    //with the implicit 1 (normal numbers)
static int isNaN(uint32_t a){ return (a<<1)>0xFF000000u; }

//m with its leading 1 at bit 31: the 24 bit significand, then 8 bits to round on (bit 0 also stands for
//any lower bits that were not zero); value = m * 2^(e-158)
static float roundPack(uint32_t s, int32_t e, uint32_t m){
	uint32_t r = m&0xFFu;
	m >>= 8;
	if(r>0x80u||(r==0x80u&&(m&1u))){
		if(++m==0x1000000u){ m >>= 1; e++; }
	}
	if(e>=255) return flt(s|0x7F800000u);
	if(e<=0) return flt(s);
	return flt(s|(uint32_t)e<<23|(m&0x7FFFFFu));
}

KEEP float __aeabi_fadd(float x, float y);
KEEP float __aeabi_fsub(float x, float y);
KEEP float __aeabi_frsub(float x, float y);
KEEP float __aeabi_fmul(float x, float y);
KEEP float __aeabi_fdiv(float x, float y);
KEEP int __aeabi_fcmpeq(float x, float y);
KEEP int __aeabi_fcmplt(float x, float y);
KEEP int __aeabi_fcmple(float x, float y);
KEEP int __aeabi_fcmpgt(float x, float y);
KEEP int __aeabi_fcmpge(float x, float y);
KEEP int __aeabi_fcmpun(float x, float y);
KEEP float __aeabi_i2f(int32_t i);
KEEP float __aeabi_ui2f(uint32_t u);
KEEP int32_t __aeabi_f2iz(float x);
KEEP uint32_t __aeabi_f2uiz(float x);

float __aeabi_fadd(float x, float y){
	uint32_t a = bits(x), b = bits(y);
	if((a<<1)<(b<<1)){ uint32_t t = a; a = b; b = t; }    //|a| >= |b| (a NaN sorts above infinity)
	uint32_t ea = expo(a), eb = expo(b);
	if(ea==255){
		if(a<<9) return flt(a|0x400000u);                 //NaN
		if(eb==255&&(a^b)>>31) return flt(DEFAULT_NAN);   //inf - inf
		return flt(a);
	}
	if(eb==0) return flt(ea ? a : a&b&SIGN);    //b is 0: a (both 0: -0 only for -0 + -0)
	uint32_t ma = mant(a)<<7, mb = mant(b)<<7, d = ea-eb;    //leading 1 at bit 30
	if(d>=31) mb = 1;    //only the sticky bit is left
	else if(d) mb = mb>>d|((mb<<(32-d))!=0);
	uint32_t m = (a^b)>>31 ? ma-mb : ma+mb;
	if(!m) return flt(0);    //x - x = +0
	int32_t e = (int32_t)ea+1;
	while(!(m>>31)){ m <<= 1; e--; }
	return roundPack(a&SIGN, e, m);
}

float __aeabi_fsub(float x, float y){ return __aeabi_fadd(x, flt(bits(y)^SIGN)); }
float __aeabi_frsub(float x, float y){ return __aeabi_fadd(y, flt(bits(x)^SIGN)); }

float __aeabi_fmul(float x, float y){
	uint32_t a = bits(x), b = bits(y), s = (a^b)&SIGN, ea = expo(a), eb = expo(b);
	if(isNaN(a)) return flt(a|0x400000u);
	if(isNaN(b)) return flt(b|0x400000u);
	if(ea==255||eb==255) return flt(ea==0||eb==0 ? DEFAULT_NAN : s|0x7F800000u);    //inf * 0 = NaN
	if(ea==0||eb==0) return flt(s);
	uint64_t p = (uint64_t)mant(a)*mant(b);    //2^46 .. 2^48
	uint32_t m = (uint32_t)(p>>16)|((p&0xFFFFu)!=0);
	int32_t e = (int32_t)(ea+eb)-126;
	if(!(m>>31)){ m <<= 1; e--; }
	return roundPack(s, e, m);
}

float __aeabi_fdiv(float x, float y){
	uint32_t a = bits(x), b = bits(y), s = (a^b)&SIGN, ea = expo(a), eb = expo(b);
	if(isNaN(a)) return flt(a|0x400000u);
	if(isNaN(b)) return flt(b|0x400000u);
	if(ea==255) return flt(eb==255 ? DEFAULT_NAN : s|0x7F800000u);    //inf / inf = NaN
	if(eb==255) return flt(s);
	if(eb==0) return flt(ea==0 ? DEFAULT_NAN : s|0x7F800000u);    //0 / 0 = NaN, x / 0 = inf
	if(ea==0) return flt(s);
	uint32_t ma = mant(a), mb = mant(b), q = 0;
	int32_t e = (int32_t)ea-(int32_t)eb+126;
	if(ma>=mb) e++; else ma <<= 1;    //ma / mb in [1, 2)
	for(uint8_t i=0;i<32;i++){    //restoring division, 32 quotient bits (the first is 1)
		q <<= 1;
		if(ma>=mb){ ma -= mb; q |= 1u; }
		ma <<= 1;
	}
	return roundPack(s, e, q|(ma!=0));
}

//-1 less, 0 equal, 1 greater, 2 unordered (NaN)
static int fcmp(float x, float y){
	uint32_t a = bits(x), b = bits(y);
	if(isNaN(a)||isNaN(b)) return 2;
	int32_t ka = a>>31 ? -(int32_t)(a&~SIGN) : (int32_t)a;    //sign and magnitude to ordered integers (-0 = 0)
	int32_t kb = b>>31 ? -(int32_t)(b&~SIGN) : (int32_t)b;
	return ka<kb ? -1 : ka>kb;
}
int __aeabi_fcmpeq(float x, float y){ return fcmp(x, y)==0; }
int __aeabi_fcmplt(float x, float y){ return fcmp(x, y)==-1; }
int __aeabi_fcmple(float x, float y){ int c = fcmp(x, y); return c==-1||c==0; }
int __aeabi_fcmpgt(float x, float y){ return fcmp(x, y)==1; }
int __aeabi_fcmpge(float x, float y){ int c = fcmp(x, y); return c==0||c==1; }
int __aeabi_fcmpun(float x, float y){ return fcmp(x, y)==2; }

static float fromU(uint32_t s, uint32_t u){
	if(!u) return flt(0);
	int32_t e = 158;
	while(!(u>>31)){ u <<= 1; e--; }
	return roundPack(s, e, u);
}
float __aeabi_ui2f(uint32_t u){ return fromU(0, u); }
float __aeabi_i2f(int32_t i){ return i<0 ? fromU(SIGN, -(uint32_t)i) : fromU(0, (uint32_t)i); }

int32_t __aeabi_f2iz(float x){
	uint32_t a = bits(x), e = expo(a);
	if(e<127) return 0;
	if(e>=158) return (int32_t)(0x7FFFFFFFu+(a>>31));    //|x| >= 2^31, inf, NaN: INT_MAX / INT_MIN
	uint32_t m = mant(a);
	m = e>=150 ? m<<(e-150) : m>>(150-e);
	return a>>31 ? -(int32_t)m : (int32_t)m;
}

uint32_t __aeabi_f2uiz(float x){
	uint32_t a = bits(x), e = expo(a);
	if(!(a>>31)&&e>=158&&!isNaN(a)) return e>=159 ? 0xFFFFFFFFu : mant(a)<<8;    //2^31 .. 2^32, above
	return (uint32_t)__aeabi_f2iz(x);
}
