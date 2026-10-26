/*  _   _  _____     _______ ____    _   _    _    ____ _  __ __     ______  */
/* | | | |/ _ \ \   / / ____|  _ \  | | | |  / \  / ___| |/ / \ \   / /___ \ */
/* | |_| | | | \ \ / /|  _| | |_) | | |_| | / _ \| |   | ' /   \ \ / /  __) |*/
/* |  _  | |_| |\ V / | |___|  _ <  |  _  |/ ___ \ |___| . \    \ V /  / __/ */
/* |_| |_|\___/  \_/  |_____|_| \_\ |_| |_/_/   \_\____|_|\_\    \_/  |_____|*/
/*                                                                           */
//version 163 : fixed bug low battery voltage and drivemode cannot be changed



#ifndef PINOUT_H
#define PINOUT_H

#include "hardware.h"
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
//18-22 are unused pin slots, used for numbers; 0 or 65535 (an unset pin slot) = default / off
#define POS_KP pinstorage[18]      //position: 0.1 rpm per hall step of error, default 20
#define POS_RPM pinstorage[19]     //position: max speed, rpm, default 100
#define MOTOR_R pinstorage[20]     //phase resistance incl. the mosfets, mOhm, default 300 (sine torque)
#define FW_IMAX pinstorage[21]     //FOC field weakening: max negative d current, 0.1 A; default off
#define FW_ADVANCE pinstorage[22]  //sine field weakening: max phase advance, degrees electrical; default off. No current
                                   //control: 15 deg ran the bench wheel at 914 instead of 586 rpm but drew 3.1 A instead of
                                   //0.66 A from the supply (~12 A phase current) unloaded; FOC field weakening is controlled
#define OCPPIN pinstorage[23]
#define ITOTAL_ZERO pinstorage[25] //shunt zero below adc 0 (the adc clips it), 0.01 counts; 0/65535 = 0 (bench board: 145)
#define OCPREFPIN pinstorage[24]

#define VBAT_DIVIDER pinstorage[33]
#define ITOTAL_DIVIDER pinstorage[34]    //itotal = divider * adc voltage: 1/shunt ohm for a direct shunt (250 = 4 mOhm; bench
                                         //board 419, measured against the op-amp phase current)
#define BAUD pinstorage[36]    //uart baud / 100 (config version 2; 1152 = 115200)
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
//FOC (foc.c), 0 = built-in default. The phase current pins are IPHASEAPIN/IPHASEBPIN.
#define IPHASE_GAIN pinstorage[56] //0.1 mA per adc count, default 515 (MM32SPIN05PFOP op-amps)
#define IPHASE_CFG pinstorage[57]  //bits 0-1: phases on the two pins 0 = A,B  1 = B,C  2 = A,C;
                                   //bit 2: adc rises with current into the motor (default: falls)
#define FOC_IMAX pinstorage[58]    //phase current limit, 0.1 A, default 100; trip at twice that
#define CUR_KP pinstorage[59]      //current loop, mV per A, default 400 (rings at 800, oscillates at 1600)
#define CUR_KI pinstorage[60]      //current loop, V per A*s, default 300 (zero at R/L: 0.3 Ohm, 0.46 mH)
#define FOC_SPD_KP pinstorage[61]  //FOC_SPEED, mA per rpm, default 30
#define FOC_SPD_KI pinstorage[62]  //FOC_SPEED, mA per rpm*s, default 100 (kp 50 limit-cycles at the stop)


//config layout version: 0 = before the drive mode encoding commutation*4 + control (SINE_VOLT was 2,
//SINE_SPEED 3); main.c migrates older layouts after loading
#define CFG_VERSION pinstorage[63]
#define CFG_VERSION_CURRENT 2    //2: BAUD in units of 100 baud

static inline int32_t cfgOrDefault(uint16_t v, int32_t def){ return (v==0||v==0xFFFF) ? def : v; }

#define MAGICNUM_POS 32
#define MAGIC_NUMBER 0xDCAB


//independent watchdog, ~500 ms (main.c). A watchdog reset releases the power latch: the board switches off.
#define WATCHDOG

#endif
