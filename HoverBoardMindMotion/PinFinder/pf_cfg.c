//Config words by name (cfgnames.h, generated from tools/cfgfields.py), save / change tracking.
#include "pf.h"
#include "cfgnames.h"
#include "sim_eeprom.h"
#include <string.h>

//packed: index | pin << 7 per field, the names one after the other ('\0' separated); 8 bytes less per
//field than a table of {index, pin, char*}
static const uint8_t fieldIdx[CFG_FIELD_COUNT] = {
#define F(i, p, n) (uint8_t)((i)|(p)<<7),
	CFG_FIELDS
#undef F
};
static const char fieldNames[] =
#define F(i, p, n) n "\0"
	CFG_FIELDS
#undef F
	;
static const char* nameOf(uint8_t f){
	const char* s = fieldNames;
	while(f--) s += strlen(s)+1;
	return s;
}
static uint16_t saved[128];    //the record in flash (or the defaults): CfgChanged()

int16_t CfgIndex(const char* name){
	const char* n = fieldNames;
	for(uint8_t i=0;i<CFG_FIELD_COUNT;i++,n+=strlen(n)+1){
		if(eq(name, n)) return fieldIdx[i]&127;
	}
	//a number: the word index
	int32_t v = 0;
	if(!*name) return -1;
	for(const char* s=name;*s;s++){
		if(*s<'0'||*s>'9') return -1;
		v = v*10+(*s-'0');
		if(v>127) return -1;
	}
	return (int16_t)v;
}

static int16_t field(uint8_t idx){
	for(uint8_t i=0;i<CFG_FIELD_COUNT;i++){
		if((fieldIdx[i]&127)==idx) return i;
	}
	return -1;
}

const char* CfgName(uint8_t idx){
	int16_t f = field(idx);
	return f<0 ? 0 : nameOf((uint8_t)f);
}

uint8_t CfgIsPin(uint8_t idx){
	int16_t f = field(idx);
	return f>=0&&fieldIdx[f]>>7;
}

int16_t CfgPinWord(uint16_t pin, uint8_t skip){
	for(uint8_t f=0;f<CFG_FIELD_COUNT;f++){
		uint8_t i = fieldIdx[f]&127;
		if(fieldIdx[f]>>7&&i!=skip&&pinstorage[i]==pin) return i;
	}
	return -1;
}

void CfgPrint(uint8_t idx){
	const char* n = CfgName(idx);
	char b[8];
	if(!n){    //unnamed word: its index
		uint8_t k = 0;
		b[k++] = 'w';
		if(idx>=100) b[k++] = (char)('0'+idx/100);
		if(idx>=10) b[k++] = (char)('0'+idx/10%10);
		b[k++] = (char)('0'+idx%10);
		b[k] = 0;
		n = b;
	}
	kv(n);
	if(CfgIsPin(idx)) outpin(pinstorage[idx]);
	else outi(pinstorage[idx]);
	if(!agent&&pinstorage[idx]!=saved[idx]){
		color(33);
		out("  (changed, saved: ");
		if(CfgIsPin(idx)) outpin(saved[idx]);
		else outi(saved[idx]);
		outc(')');
		color(0);
	}
	nl();
}

uint8_t CfgWordChanged(uint8_t idx){
	return saved[idx]!=pinstorage[idx];
}

uint8_t CfgChanged(void){
	return memcmp(saved, pinstorage, sizeof(saved))!=0;
}

void CfgMarkSaved(void){
	memcpy(saved, pinstorage, sizeof(saved));
}

void CfgSave(void){
	pinstorage[MAGICNUM_POS] = MAGIC_NUMBER;
	pinstorage[EXT_MAGIC_POS] = EXT_MAGIC;
	SaveConfig();
	CfgMarkSaved();
}

void CfgDefaults(void){    //PINSTORAGE_DEFAULTS (pinout.h), built: only words 32-47 are a table
	static const uint16_t mid[16] = { PINSTORAGE_WORDS_32_47 };
	for(uint8_t i=0;i<128;i++) pinstorage[i] = i<32||i>=64 ? 0xFFFF : (i<48 ? mid[i-32] : 0);
	pinstorage[EXT_MAGIC_POS] = EXT_MAGIC;
	ConfigMigrate();
}
