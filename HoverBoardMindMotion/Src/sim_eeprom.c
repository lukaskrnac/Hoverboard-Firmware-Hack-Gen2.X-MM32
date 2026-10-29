//Board configuration in flash: one 1 KB page at 0x08007C00 (the linker script keeps it out of the image),
//8 slots of 128 bytes. A save writes a pair: the extension record (words 64-127, EXT_MAGIC at its word 32),
//then the main record (words 0-63, MAGIC_NUMBER at word 32) in the next slot; with fewer than two blank
//slots left the page is erased first (every 4th save; a power cut in that ~25 ms loses the config: the
//firmware then shows the no-config fault, PinFinder writes a new one). The newest main record is the slot
//before the first blank one (the last when the page is full), its extension the slot before it. Same in
//tools/mm32cfgstore.py. Until 2026-10-30 it was the vendor's sim-EEPROM over 2 KB at 0x08007800 (16 slots
//used round): code space now, mm32flash.py moves an old configuration into the new page.
#include "hal_conf.h"
#include "sim_eeprom.h"
#include "pinout.h"
#include "bldc.h"

#define CFG_BASE 0x08007C00u
#define CFG_SLOTS 8u
#define SLOT 128u
#define SLOT_ADDR(i) (CFG_BASE+(i)*SLOT)

static uint8_t slotBlank(uint32_t i){
	const uint32_t* p = (const uint32_t*)SLOT_ADDR(i);
	for(uint8_t k=0;k<SLOT/4;k++) if(p[k]!=0xFFFFFFFFu) return 0;
	return 1;
}

static uint32_t firstBlank(void){    //CFG_SLOTS: the page is full
	uint32_t i = 0;
	while(i<CFG_SLOTS&&!slotBlank(i)) i++;
	return i;
}

void SaveConfig(void){
	uint32_t s = firstBlank();
	FLASH_Unlock();
	if(s>CFG_SLOTS-2){ FLASH_ErasePage(CFG_BASE); s = 0; }
	const uint16_t* w = &pinstorage[64];    //the extension, then the main record
	for(uint32_t k=0;k<2;k++,s++,w=pinstorage){
		for(uint32_t i=0;i<SLOT/2;i++) FLASH_ProgramHalfWord(SLOT_ADDR(s)+2*i, w[i]);
	}
	FLASH_Lock();
}

//the newest record if its magic is valid (and the extension record before it, if it has one)
uint8_t ConfigLoad(void){
	uint32_t s = firstBlank();
	if(!s) return 0;
	const uint16_t* m = (const uint16_t*)SLOT_ADDR(s-1);
	if(m[MAGICNUM_POS]!=MAGIC_NUMBER) return 0;
	memcpy(pinstorage, m, SLOT);
	if(s>=2&&m[MAGICNUM_POS-SLOT/2]==EXT_MAGIC) memcpy(&pinstorage[64], m-SLOT/2, SLOT);
	return 1;
}

//a valid record with the pins the firmware cannot run without
uint8_t restorecfg(void){
	return ConfigLoad()&&HALLAPIN<PINCOUNT&&HALLBPIN<PINCOUNT&&HALLCPIN<PINCOUNT&&TXPIN<PINCOUNT&&RXPIN<PINCOUNT;
}

//older layouts (CFG_VERSION) to the current one; saved with the next config save
void ConfigMigrate(void){
	if(CFG_VERSION<1){    //old drive mode numbers (block 0/1, sine 2/3)
		if(DRIVEMODE==2) DRIVEMODE=SINE_VOLT;
		else if(DRIVEMODE==3) DRIVEMODE=SINE_SPEED;
		CFG_VERSION=1;
	}
	if(CFG_VERSION<2){    //baud was stored in baud, now in 100 baud (115200 did not fit the 16 bit word)
		BAUD = BAUD>=1200 ? BAUD/100 : 1152;
		CFG_VERSION=2;
	}
	if(CFG_VERSION<3){    //input mode words: PinFinder's "reserved" (35, 43 held 0 / 1), now default
		INPUT_MODE = VEL_RAMP_RATE = INPUT_FILTER_BW = TRAJ_VEL = TRAJ_ACCEL = TRAJ_DECEL = 0xFFFF;
		CFG_VERSION=3;
	}
}
