//PinFinder: finds a board's pins and writes the configuration the main firmware boots from.
//Two front ends on one command core (pf_cmd.c): the TUI for people (pf_tui.c) and a line protocol for
//scripts and agents (agent mode, see README "PinFinder"). Output goes through the kv*/note/ok/err
//functions, which print "key=value" lines in agent mode and readable lines in the TUI.
#ifndef PF_H
#define PF_H

#include "mcu.h"
#include "hardware.h"
#include "pinout.h"

#define PF_VERSION "2.0"
#define PF_BAUD 115200
#define PIN_NONE 0xFFFF

//pf_io.c: serial output (blocking) and input (the uart rx ring of uart.c)
extern uint8_t agent;    //1: agent protocol, 0: TUI
extern uint8_t ansi;     //TUI: the terminal answered the ANSI status request
void IoInit(uint8_t uart);
void outc(char c);
void out(const char* s);
void outi(int32_t v);
void outfix(int32_t v, uint8_t dec);    //v / 10^dec with dec decimals
void outpin(uint16_t pin);              //PA7, "none" when not set
void nl(void);
void color(uint8_t c);                  //TUI with ANSI: 0 reset, 31 red, 32 green, 33 yellow, 36 cyan, 1 bold
int inbyte(void);                       //-1: nothing received
uint8_t aborted(void);                  //a key / line arrived: stop the running operation (consumes it)
void kv(const char* key);               //starts a result line; the value follows, then nl()
void kvi(const char* key, int32_t v);
void kvp(const char* key, uint16_t pin);
void kvs(const char* key, const char* s);
void kvmv(const char* key, int32_t mv); //agent: key_mv=24100, TUI: key: 24.10 V
void note(const char* s);               //agent: "# s"
void warn(const char* s);
void progress(int32_t n, int32_t total);    //TUI: a dot per call
void progressEnd(void);                 //TUI: ends the dots' line
void ok(void);
void err(const char* s);

//pf_hw.c
void Wait(uint32_t ms);
void WaitUs(uint32_t us);
void AdcInit(void);
void AdcScan(uint32_t chmask);          //one conversion of each channel in the mask (bit = channel)
void AdcScanTriggered(uint32_t chmask);    //the same, started by TIM1 CC4 (low sides on)
void AdcScanAt(uint32_t chmask, uint16_t cnt);    //started at TIM1 counter cnt
uint16_t AdcValue(uint8_t ch);          //the last conversion of a channel
int8_t AdcChannel(uint16_t pin);        //-1: the pin has no adc channel
int32_t VddMv(void);                    //MCU supply from the internal 1.2 V reference
int32_t PinMv(uint16_t pin);            //voltage at an adc pin (VDD scaled)
int32_t VbatMv(void);                   //battery: VBATPIN x VBAT_DIVIDER, -1 without the pin
void MotorInit(void);
void GatesOff(void);
void BlockStep(uint8_t step, uint16_t duty);    //block commutation step 1..6 (bldc.c's table) at duty / PWM_PERIOD
extern uint16_t fieldCcr[3];
void FieldVector(uint8_t k, int32_t mv);
void FieldAngle(uint16_t th, int32_t mv);    //field at th (65536 per electrical turn, the main firmware's angle)
int32_t Sin16(uint16_t a);                   //Q15    //k = 0..11: field at k x 30 deg el (even k: step k/2+1), complementary
uint16_t DutyForMv(int32_t mv);         //duty for this average phase voltage at the measured (or assumed 42 V) battery
uint8_t PinFree(uint16_t pin);          //not SWD, not configured for anything (pins in words 0-17, 23, 24)
uint8_t HallState(void);                //C*4 + B*2 + A (the main firmware's hallpos(1))
const char* PinName(uint16_t pin);
uint16_t PinByName(const char* s);      //PIN_NONE: not a pin name
extern uint8_t uartUsed;                //1: UART1, 2: UART2

//pf_cfg.c: config words by name
int16_t CfgIndex(const char* name);     //-1: unknown
const char* CfgName(uint8_t idx);       //0: unnamed word
uint8_t CfgIsPin(uint8_t idx);
void CfgPrint(uint8_t idx);             //one result line
uint8_t CfgChanged(void);               //differs from the saved record
void CfgMarkSaved(void);
void CfgSave(void);
void CfgDefaults(void);

//pf_detect.c: the detections; each prints results and ends with ok() / err()
void DetectHall(int32_t mv);
void DetectVbat(int32_t expect_mv);
void DetectCurrent(int32_t mv, int32_t shunt_uohm);
void ScanWindow(uint8_t k, int32_t mv, uint16_t pin);        //phase current pins, then the bus current pin and its calibration
void ShowAdc(void);
void DetectButton(uint32_t ms);
void DetectCharger(uint32_t ms);
void DetectIr(uint32_t ms);
void DetectImu(void);                   //pf_imu.c
void CalImu(void);
//pf_calib.c
void CalHall(int32_t mv);
void CalPoles(uint32_t ms);
void CalRL(int32_t mv);
void CalKv(const char* which, int32_t top);
void StepHold(uint8_t s, int32_t mv, uint32_t ms);
void Rotate(int32_t permille, uint32_t ms);
void RotateRun(int32_t permille);       //one main loop pass of the TUI's test rotation
void RotateStop(void);
uint8_t UartFind(void);                 //boot without uart pins: loop-back search, 1 = found
void LatchHold(void);                   //boot: hold the power latch (or pull every free pin up to find it)
void LatchFind(void);
void PowerOff(void);
void Tone(uint16_t pin, uint32_t ms, uint32_t hz);    //square wave (passive buzzer test)
void Info(void);
extern uint8_t master;                  //board has a power latch

//pf_cmd.c
void Command(char* line);               //one command line, both modes

//pf_tui.c
void TuiStart(void);
void TuiKey(int c);
void TuiPoll(void);                     //main loop: running interactive screens
extern uint8_t tuiLine;                 //1: the TUI's command line has the keys

#endif
