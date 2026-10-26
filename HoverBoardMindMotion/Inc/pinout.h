/*  _   _  _____     _______ ____    _   _    _    ____ _  __ __     ______  */
/* | | | |/ _ \ \   / / ____|  _ \  | | | |  / \  / ___| |/ / \ \   / /___ \ */
/* | |_| | | | \ \ / /|  _| | |_) | | |_| | / _ \| |   | ' /   \ \ / /  __) |*/
/* |  _  | |_| |\ V / | |___|  _ <  |  _  |/ ___ \ |___| . \    \ V /  / __/ */
/* |_| |_|\___/  \_/  |_____|_| \_\ |_| |_/_/   \_\____|_|\_\    \_/  |_____|*/
/*                                                                           */
//version 163 : fixed bug low battery voltage and drivemode cannot be changed



#ifndef PINOUT_H
#define PINOUT_H

#include "../Inc/hardware.h"
extern uint16_t pinstorage[64];    //board configuration, loaded from the sim-EEPROM (main.c)
	

#define HALLAPIN pinstorage[0]
#define HALLBPIN pinstorage[1]
#define HALLCPIN pinstorage[2]
#define LEDRPIN pinstorage[3]
#define LEDGPIN pinstorage[4]
#define LEDBPIN pinstorage[5]
#define LEDUPIN pinstorage[6]
#define LEDDPIN pinstorage[7]
#define BUZZERPIN pinstorage[8]
#define BUTTONPIN pinstorage[9]
#define LATCHPIN pinstorage[10]
#define VBATPIN pinstorage[12]
#define ITOTALPIN pinstorage[13]
#define TXPIN pinstorage[14]
#define RXPIN pinstorage[15]
#define IPHASEAPIN pinstorage[16]
#define IPHASEBPIN pinstorage[17]
#define OCPPIN pinstorage[23]
#define OCPREFPIN pinstorage[24]

#define VBAT_DIVIDER pinstorage[33]
#define ITOTAL_DIVIDER pinstorage[34]    //itotal = divider * adc voltage: 1/shunt ohm for a direct shunt (250 = 4 mOhm)
#define BAUD pinstorage[36]
#define PWM_RES pinstorage[37]
#define SLAVE_ID pinstorage[38]
#define WINDINGS pinstorage[39]
#define INVERT_LOWSIDE pinstorage[40]
#define SOFT_ILIMIT pinstorage[41]    //A, averaged bus current that switches the gates off, 0 = off
#define AWDG pinstorage[42]    //adc counts above the zero offset, hardware (adc watchdog) gate trip, 0 = off
#define DRIVEMODE pinstorage[44]
#define BAT_FULL pinstorage[45]
#define BAT_EMPTY pinstorage[46]
#define SERIAL_TIMEOUT pinstorage[47]
//COM_SPEED / SINE_SPEED controllers (calculation.c), 0 = built-in default
#define SPEED_KP pinstorage[48]    //0.1 permille duty per rpm error, default 50
#define SPEED_KI pinstorage[49]    //0.1 permille duty per rpm*s, default 100
#define SPEED_KV pinstorage[50]    //feed forward: 0.1 rpm per volt at 100% duty, default 395
#define SPEED_D0 pinstorage[51]    //feed forward: dead band in permille duty, default 185
#define SINE_KP pinstorage[52]     //SINE_SPEED: 0.1 permille modulation per rpm error, default 5
#define SINE_KI pinstorage[53]     //SINE_SPEED: 0.1 permille per rpm*s, default 85
#define SINE_KV pinstorage[54]     //SINE_SPEED feed forward: 0.1 rpm per volt at 100% modulation, default 242
#define SINE_D0 pinstorage[55]     //SINE_SPEED feed forward: dead band in permille, default 10


#define MAGICNUM_POS 32
#define MAGIC_NUMBER 0xDCAB


//independent watchdog, ~500 ms (main.c). A watchdog reset releases the power latch: the board switches off.
#define WATCHDOG

#endif
