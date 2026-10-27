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
extern uint16_t pinstorage[128];    //board configuration, loaded from the sim-EEPROM (main.c)
//words 0-63: the record PinFinder writes; 64-127: the extension record (its own magic at 96, saved in the
//slot before the main record; missing: all 0xFFFF = defaults). 0/65535 = default unless noted.
#define EXT_MAGIC_POS 96
#define EXT_MAGIC 0xDCAD
#define HOLD_CURRENT pinstorage[64]      //FOC position hold current (stiffness), 0.1 A, default 40; <= 90 % of the limit
#define HOLD_ENABLE pinstorage[65]       //FOC position hold: 1 off, 2 on, default on
#define HOLD_WINDOW pinstorage[66]       //hold within this of the target, 0.01 hall step, default 150
#define POS_ACCEL pinstorage[67]         //FOC position approach accel / braking, 0.01 turns/s^2, default 2000
#define TORQUE_RAMP_RATE pinstorage[68]  //TORQUE_RAMP input mode, 0.01 Nm/s, default 1 (ODrive 0.01)
#define INERTIA pinstorage[69]           //torque feed forward of the trajectory accel, 0.0001 Nm/(turn/s^2), default 0
#define VEL_LIMIT_TOL pinstorage[70]     //overspeed error above vel_limit times this, 0.01, default 120
#define OVERSPEED_ERROR pinstorage[71]   //enable_overspeed_error: 1 off, 2 on, default off
#define TORQUE_VEL_LIMIT pinstorage[72]  //enable_torque_mode_vel_limit: 1 off, 2 on, default on
#define PHASE_L pinstorage[73]           //phase inductance, uH, default 460
//D and I terms, 0/65535 = 0 (off). D acts on the measured speed (no kick on a set point step), filtered 20 ms
#define POS_KI pinstorage[74]            //position I: 0.1 rpm per (hall step * s)
#define POS_KD pinstorage[75]            //position D: 0.01 rpm per rpm of speed error (trajectory - wheel)
#define FOC_SPD_KD pinstorage[76]        //FOC velocity D: 0.01 mA per rpm/s
#define SINE_KD pinstorage[77]           //sine velocity D: 0.01 permille modulation per rpm/s
#define SPEED_KD pinstorage[78]          //block velocity D: 0.01 permille duty per rpm/s
#define BUZZER_PASSIVE pinstorage[79]    //buzzer without its own oscillator (driven with 2.2 kHz): 1 off, 2 on, default off
#define CHARGE_LEVEL pinstorage[80]      //charger pin level while the charger is connected: 1 low, 2 high, default high
#define CHARGE_PULL pinstorage[81]       //charger pin input: 1 floating, 2 pull-up, 3 pull-down, default floating
#define CHARGE_LOCK pinstorage[82]       //no driving while the charger is connected: 1 off, 2 on, default off
#define IMU_SDA pinstorage[83]          //IMU I2C pins, address and id (PinFinder detect imu; imu.c reads it)
#define IMU_SCL pinstorage[84]
#define IMU_ADDR pinstorage[85]
#define IMU_ID pinstorage[86]
#define HALL_ANG(h) pinstorage[86+(h)]   //h 1..6 (words 87-92): angle (65536 per electrical turn) where hall state h
                                         //starts with rising angle (PinFinder calibrate hall); unset: +-30 deg
                                         //around the built-in sector centres
#define VBAT_GAIN pinstorage[93]         //battery divider in 0.001 (PinFinder detect vbat <V>); unset: VBAT_DIVIDER
static inline int cfgBool(uint16_t v, int def){ return v==2 ? 1 : (v==1 ? 0 : def); }


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
#define CHARGEPIN pinstorage[11]    //charger detect (CHARGE_LEVEL, CHARGE_PULL)
#define VBATPIN pinstorage[12]
#define ITOTALPIN pinstorage[13]
#define TXPIN pinstorage[14]
#define RXPIN pinstorage[15]
#define IPHASEAPIN pinstorage[16]
#define IPHASEBPIN pinstorage[17]
//18-22 are unused pin slots, used for numbers; 0 or 65535 (an unset pin slot) = default / off
#define POS_KP pinstorage[18]      //position: 0.1 rpm per hall step of error, default 20
#define POS_RPM pinstorage[19]     //velocity limit (vel_limit) in position and velocity modes, rpm, default 300
#define MOTOR_R pinstorage[20]     //phase resistance incl. the mosfets, mOhm, default 300 (sine torque)
#define FW_IMAX pinstorage[21]     //FOC field weakening: max negative d current, 0.1 A; default off
#define FW_ADVANCE pinstorage[22]  //sine field weakening: max phase advance, degrees electrical; default off. No current
                                   //control: 15 deg ran the bench wheel at 914 instead of 586 rpm but drew 3.1 A instead of
                                   //0.66 A from the supply (~12 A phase current) unloaded; FOC field weakening is controlled
//ODrive input modes (inputmode.c); 0/65535 = default
#define INPUT_MODE pinstorage[28]       //1 PASSTHROUGH (default), 2 VEL_RAMP, 3 POS_FILTER, 5 TRAP_TRAJ
#define VEL_RAMP_RATE pinstorage[29]    //VEL_RAMP: 0.01 turns/s^2, default 100 (ODrive's 1.0)
#define INPUT_FILTER_BW pinstorage[30]  //POS_FILTER bandwidth: 0.01 /s, default 200 (2.0)
#define TRAJ_VEL pinstorage[31]         //TRAP_TRAJ: velocity limit 0.01 turns/s, default 200 (2.0)
#define TRAJ_ACCEL pinstorage[35]       //TRAP_TRAJ: 0.01 turns/s^2, default 50 (0.5)
#define TRAJ_DECEL pinstorage[43]       //TRAP_TRAJ: 0.01 turns/s^2, default 50 (0.5)
#define CUR_TRIP pinstorage[27]      //phase current trip (current_hard_max), 0.1 A; 0/65535 = 2 x focimax, >= 15 A
#define TORQUE_CONST pinstorage[26]  //Nm per A x 1000 (CANSimple torque units); 0/65535 = 8.27 / sine kv
#define OCPPIN pinstorage[23]
#define ITOTAL_ZERO pinstorage[25] //shunt zero below adc 0 (the adc clips it), 0.01 counts; 0/65535 = 0 (bench board: 145)
#define OCPREFPIN pinstorage[24]

#define VBAT_DIVIDER pinstorage[33]
#define ITOTAL_DIVIDER pinstorage[34]    //itotal = divider * adc voltage: 1/shunt ohm for a direct shunt (333 = 3 mOhm: the
                                         //bench board's R003; PinFinder detect current <mV> <shunt mOhm>)
#define BAUD pinstorage[36]    //uart baud / 100 (config version 2; 1152 = 115200)
#define PWM_RES pinstorage[37]    //TIM1 period (centre aligned: 72 MHz / 2 / PWM_RES), read at boot; 8192 .. 16383
//the period the timer runs at: the code uses this, not PWM_RES (a RAM change applies at the next start)
#define PWM_PERIOD ((uint16_t)TIM1->ARR)
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
#define CFG_VERSION_CURRENT 3    //2: BAUD in units of 100 baud, 3: input mode words 28-31, 35, 43

static inline int32_t cfgOrDefault(uint16_t v, int32_t def){ return (v==0||v==0xFFFF) ? def : v; }

//the record a board without a saved configuration starts from (main firmware: does not boot; PinFinder:
//detects into it). Layout version 0 values (baud 19200 in baud): ConfigMigrate() brings it up to date.
#define PINSTORAGE_DEFAULTS {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, \
	0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, \
	0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDCAB, 31, 250, 0, 19200, 8192, 1, 30, 0, 10, 300, 1, 1, 42000, \
	32000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	[64 ... 95] = 0xFFFF, [EXT_MAGIC_POS] = EXT_MAGIC, [97 ... 127] = 0xFFFF}

#define MAGICNUM_POS 32
#define MAGIC_NUMBER 0xDCAB


//independent watchdog, ~500 ms (main.c). A watchdog reset releases the power latch: the board switches off.
#define WATCHDOG

#endif
