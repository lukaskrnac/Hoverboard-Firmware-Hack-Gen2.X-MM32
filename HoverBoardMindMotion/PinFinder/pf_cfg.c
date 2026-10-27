//Config words by name (cfgnames.h, generated from tools/cfgfields.py), save / change tracking.
#include "pf.h"
#include "cfgnames.h"
#include "sim_eeprom.h"
#include <string.h>

static const struct{ uint8_t idx, pin; const char* name; } fields[CFG_FIELD_COUNT] = {
#define F(i, p, n) {i, p, n},
	CFG_FIELDS
#undef F
};
static uint16_t saved[128];    //the record in flash (or the defaults): CfgChanged()

int16_t CfgIndex(const char* name){
	for(uint8_t i=0;i<CFG_FIELD_COUNT;i++){
		if(!strcmp(name, fields[i].name)) return fields[i].idx;
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
		if(fields[i].idx==idx) return i;
	}
	return -1;
}

const char* CfgName(uint8_t idx){
	int16_t f = field(idx);
	return f<0 ? 0 : fields[f].name;
}

uint8_t CfgIsPin(uint8_t idx){
	int16_t f = field(idx);
	return f>=0&&fields[f].pin;
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

void CfgDefaults(void){
	static const uint16_t def[128] = PINSTORAGE_DEFAULTS;
	memcpy(pinstorage, def, sizeof(def));
	ConfigMigrate();
}
