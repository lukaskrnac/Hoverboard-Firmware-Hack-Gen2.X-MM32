//ODrive CANSimple protocol over the RemoteUartBus.
//
//The message ids, payload layouts and units are those of ODrive's CANSimple protocol (docs.odriverobotics
//.com, can-protocol); a CAN frame becomes a UART frame:
//
//    0xA5 | node id | cmd id (bit 7 = request, the CAN RTR bit) | length 0..8 | payload | CRC-16 (LE)
//
//CRC-16/XMODEM (as the v1 frames) over node id .. payload. Node 0x3F is broadcast: every board acts on
//it, none answers. Set_ messages are not answered; requests (bit 7) and RxSdo are, with the same framing
//and cmd id (bit 7 clear). Little endian, float32 in physical units: turns, turns/s, Nm, A, V.
//Supported: Get_Version, Heartbeat, Estop, Get_Error, RxSdo/TxSdo (cansimple_sdo.c), Set_Axis_State,
//Get_Encoder_Estimates, Set_Controller_Mode, Set_Input_Pos/Vel/Torque, Set_Limits, Get_Iq,
//Get_Temperature (NaN: no sensor), Reboot (action 1 save_configuration only: a reset releases the power
//latch), Get_Bus_Voltage_Current, Clear_Errors, Set_Absolute_Position, Set_Pos_Gain, Set_Vel_Gains,
//Get_Torques, Get_Powers. Extension: 0x1E Set_Input_Voltage (float, fraction of the maximum, for
//VOLTAGE_CONTROL; unused in CANSimple).
#include <string.h>
#include "cansimple.h"
#include "calculation.h"
#include "main.h"
#include "pinout.h"
#include "bldc.h"
#include "uart.h"
#include "foc.h"
#include "hallhandle.h"
#include "remoteUartBus.h"
#include "interrupt.h"

#define START 0xA5
#define BROADCAST 0x3F

enum{
	CMD_GET_VERSION = 0x00, CMD_HEARTBEAT = 0x01, CMD_ESTOP = 0x02, CMD_GET_ERROR = 0x03, CMD_RXSDO = 0x04,
	CMD_TXSDO = 0x05, CMD_SET_AXIS_STATE = 0x07, CMD_GET_ENCODER_ESTIMATES = 0x09,
	CMD_SET_CONTROLLER_MODE = 0x0B, CMD_SET_INPUT_POS = 0x0C, CMD_SET_INPUT_VEL = 0x0D,
	CMD_SET_INPUT_TORQUE = 0x0E, CMD_SET_LIMITS = 0x0F, CMD_GET_IQ = 0x14, CMD_GET_TEMPERATURE = 0x15,
	CMD_REBOOT = 0x16, CMD_GET_BUS_VOLTAGE_CURRENT = 0x17, CMD_CLEAR_ERRORS = 0x18,
	CMD_SET_ABSOLUTE_POSITION = 0x19, CMD_SET_POS_GAIN = 0x1A, CMD_SET_VEL_GAINS = 0x1B,
	CMD_GET_TORQUES = 0x1C, CMD_GET_POWERS = 0x1D, CMD_SET_INPUT_VOLTAGE = 0x1E,
};
enum{ CM_VOLTAGE = 0, CM_TORQUE = 1, CM_VELOCITY = 2, CM_POSITION = 3 };    //ODrive ControlMode

static int32_t posOffset;    //odometer at position 0 (Set_Absolute_Position), hall steps
static uint8_t inputMode = 1;    //ODrive InputMode, PASSTHROUGH

static float rdF(const uint8_t* p){ float f; memcpy(&f, p, 4); return f; }
static uint32_t rdU32(const uint8_t* p){ return p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static void wrF(uint8_t* p, float f){ memcpy(p, &f, 4); }
static void wrU32(uint8_t* p, uint32_t v){ p[0] = (uint8_t)v; p[1] = (uint8_t)(v>>8); p[2] = (uint8_t)(v>>16); p[3] = (uint8_t)(v>>24); }
static int32_t clampF(float f, int32_t lim){ return f>lim ? lim : (f<-lim ? -lim : (int32_t)(f<0 ? f-0.5f : f+0.5f)); }

int32_t StepsPerTurn(void){ return 3*(int32_t)WINDINGS; }    //hall edges per turn: 6 per pole pair
float TorqueConstant(void){    //Nm per A (amplitude of the phase current)
	int32_t kt = cfgOrDefault(TORQUE_CONST, 0);
	if(!kt){    //from the sine kv (0.1 rpm/V at 100 % modulation): 8.27 / kv, as ODrive's estimate
		int32_t kv = SINE_KV ? SINE_KV : 242;
		return 82.7f/(float)kv;
	}
	return (float)kt/1000.0f;
}

static uint16_t crc16(const uint8_t* p, int n){
	uint16_t crc = 0;
	while(n-->0){
		crc ^= (uint16_t)*p++<<8;
		for(uint8_t i=0;i<8;i++) crc = crc&0x8000 ? (uint16_t)(crc<<1^0x1021) : (uint16_t)(crc<<1);
	}
	return crc;
}

static void reply(uint8_t cmd, const uint8_t* data, uint8_t len){
	uint8_t f[14];
	f[0] = START; f[1] = (uint8_t)SLAVE_ID; f[2] = cmd; f[3] = len;
	memcpy(&f[4], data, len);
	uint16_t c = crc16(&f[1], 3+len);
	f[4+len] = (uint8_t)c; f[5+len] = (uint8_t)(c>>8);
	SerialSend(f, (u16)(6+len));
}

static void setControlMode(uint32_t cm){
	static const uint8_t map[4] = { CTRL_PWM, CTRL_TORQUE, CTRL_SPEED, CTRL_POSITION };
	if(cm>3) return;
	SetDriveMode((uint8_t)(COMMUTATION(DRIVEMODE)*4+map[cm]));    //RAM only; save_configuration stores it
}

static void inputArrived(void){    //a set point: commands come from this protocol, the watchdog is fed
	cmdSource = 2;
	RemoteFeedWatchdog();
}

//Get_ payloads, also used by the request handler
static uint8_t getMsg(uint8_t cmd, uint8_t* d){
	float kt = TorqueConstant();
	switch(cmd){
		case CMD_GET_VERSION:
			d[0] = 2; d[1] = 0; d[2] = 0; d[3] = 0;    //protocol 2, hardware unknown
			d[4] = FW_VERSION_MAJOR; d[5] = FW_VERSION_MINOR; d[6] = FW_VERSION_REVISION; d[7] = 1;    //unreleased
			return 8;
		case CMD_HEARTBEAT:
			wrU32(d, axisErrors|axisDisarm);
			d[4] = axisState; d[5] = 0; d[6] = posDone; d[7] = 0;
			return 8;
		case CMD_GET_ERROR:
			wrU32(d, axisErrors); wrU32(d+4, axisDisarm);
			return 8;
		case CMD_GET_ENCODER_ESTIMATES:
			wrF(d, (float)(iOdom-posOffset)/(float)StepsPerTurn());
			wrF(d+4, (float)realspeed/60.0f);
			return 8;
		case CMD_GET_IQ:{
			float set = __builtin_nanf(""), meas = __builtin_nanf("");
			if(COMMUTATION(DRIVEMODE)==COMMUT_FOC){ set = (float)FocIqSetpoint()/1000.0f; meas = (float)focIq/1000.0f; }
			else if(CONTROL(DRIVEMODE)==CTRL_TORQUE){ set = (float)speed/100.0f; }
			wrF(d, set); wrF(d+4, meas);
			return 8;
		}
		case CMD_GET_TEMPERATURE:
			wrF(d, __builtin_nanf("")); wrF(d+4, __builtin_nanf(""));
			return 8;
		case CMD_GET_BUS_VOLTAGE_CURRENT:
			wrF(d, (float)fvbat/100.0f); wrF(d+4, (float)BusCurrent()/100.0f);
			return 8;
		case CMD_GET_TORQUES:{
			uint8_t n = getMsg(CMD_GET_IQ, d);
			wrF(d, rdF(d)*kt); wrF(d+4, rdF(d+4)*kt);
			return n;
		}
		case CMD_GET_POWERS:{
			float pe = (float)fvbat/100.0f*(float)BusCurrent()/100.0f;
			float tq = COMMUTATION(DRIVEMODE)==COMMUT_FOC ? (float)focIq/1000.0f*kt : __builtin_nanf("");
			wrF(d, pe); wrF(d+4, tq*(float)realspeed*(6.2831853f/60.0f));
			return 8;
		}
	}
	return 0xFF;
}

static void handle(uint8_t node, uint8_t cmdByte, const uint8_t* d, uint8_t len){
	uint8_t cmd = cmdByte&0x1F, request = cmdByte&0x80;
	uint8_t answer = node!=BROADCAST;
	uint8_t out[8];
	if(request){
		uint8_t n = getMsg(cmd, out);
		if(n!=0xFF&&answer) reply(cmd, out, n);
		return;
	}
	switch(cmd){
		case CMD_ESTOP:
			axisState = AXIS_IDLE;
			axisDisarm |= ERR_ESTOP_REQUESTED;
			break;
		case CMD_RXSDO:
			if(len>=4){
				uint8_t n = SdoAccess(d, len, out);
				if(n&&answer) reply(CMD_TXSDO, out, n);
			}
			break;
		case CMD_SET_AXIS_STATE:
			if(len>=4){
				uint32_t st = rdU32(d);
				if(st==AXIS_IDLE) axisState = AXIS_IDLE;
				else if(st==AXIS_CLOSED_LOOP&&!axisDisarm){ axisState = AXIS_CLOSED_LOOP; RemoteFeedWatchdog(); }
			}
			break;
		case CMD_SET_CONTROLLER_MODE:
			if(len>=8){
				setControlMode(rdU32(d));
				inputMode = (uint8_t)rdU32(d+4);
			}
			break;
		case CMD_SET_INPUT_POS:
			if(len>=4){
				float steps = rdF(d)*(float)StepsPerTurn();
				posCmd = clampF(steps, 2000000000)+posOffset;
				inputArrived();
			}
			break;
		case CMD_SET_INPUT_VEL:
			if(len>=4){
				if(CONTROL(DRIVEMODE)==CTRL_SPEED) speed = clampF(rdF(d)*60.0f, 1000);    //rpm
				inputArrived();
			}
			break;
		case CMD_SET_INPUT_TORQUE:
			if(len>=4){
				if(CONTROL(DRIVEMODE)==CTRL_TORQUE) speed = clampF(rdF(d)/TorqueConstant()*100.0f, 30000);    //0.01 A
				inputArrived();
			}
			break;
		case CMD_SET_INPUT_VOLTAGE:
			if(len>=4){
				if(CONTROL(DRIVEMODE)==CTRL_PWM) speed = clampF(rdF(d)*1000.0f, 1000);    //permille
				inputArrived();
			}
			break;
		case CMD_SET_LIMITS:
			if(len>=8){
				float v = rdF(d), i = rdF(d+4);
				if(v>0) POS_RPM = (uint16_t)clampF(v*60.0f, 1000);    //position speed limit, rpm
				if(i>0) FOC_IMAX = (uint16_t)clampF(i*10.0f, 600);    //0.1 A
			}
			break;
		case CMD_REBOOT:
			if(len>=1&&d[0]==1) RemoteSaveConfig();    //save_configuration (reboot/erase/DFU: not supported)
			break;
		case CMD_CLEAR_ERRORS:
			axisDisarm = 0;
			FocClearTrip();
			break;
		case CMD_SET_ABSOLUTE_POSITION:
			if(len>=4){
				int32_t steps = clampF(rdF(d)*(float)StepsPerTurn(), 2000000000);
				posOffset = iOdom-steps;
				posCmd = iOdom;    //no jump: the target is where the wheel is
			}
			break;
		case CMD_SET_POS_GAIN:    //(turns/s) per turn -> 0.1 rpm per hall step
			if(len>=4) POS_KP = (uint16_t)clampF(rdF(d)*600.0f/(float)StepsPerTurn(), 65534);
			break;
		case CMD_SET_VEL_GAINS:{    //Nm/(turns/s), (Nm/s)/(turns/s) -> FOC speed loop mA/rpm, mA/(rpm*s)
			if(len>=8){
				float k = 1000.0f/60.0f/TorqueConstant();
				FOC_SPD_KP = (uint16_t)clampF(rdF(d)*k, 65534);
				FOC_SPD_KI = (uint16_t)clampF(rdF(d+4)*k, 65534);
			}
			break;
		}
	}
}

//byte by byte from the rx ring; returns 0 while not inside a CANSimple frame
static uint8_t buf[14];
static uint8_t pos;
uint8_t CanSimpleFeed(uint8_t c){
	if(pos==0){
		if(c!=START) return 0;
		buf[pos++] = c;
		return 1;
	}
	buf[pos++] = c;
	if(pos==4&&buf[3]>8){ pos = 0; return 1; }    //length out of range: resync
	if(pos<4||pos<6+buf[3]) return 1;
	uint8_t len = buf[3];
	pos = 0;
	uint16_t c16 = (uint16_t)(buf[4+len]|buf[5+len]<<8);
	if(c16!=crc16(&buf[1], 3+len)) return 1;
	if(buf[1]!=SLAVE_ID&&buf[1]!=BROADCAST) return 1;
	handle(buf[1], buf[2], &buf[4], len);
	return 1;
}

//RxSdo / TxSdo: parameter access by endpoint id (the names, types and units are in tools/endpoints.json,
//as ODrive's flat_endpoints.json). Request: opcode u8 (0 read, 1 write), endpoint u16, reserved u8,
//value (4 bytes). Answer: 0, endpoint u16, 0, value (read); a write is confirmed without a value.
//  0x000..0x03F  config words (pinstorage, u16 in a u32); in RAM, save_configuration stores them
//  0x100..       runtime values, float32 unless noted
//  0x200..       functions (write, no value)
enum{
	EP_VBUS = 0x100, EP_IBUS, EP_POS_ESTIMATE, EP_VEL_ESTIMATE, EP_IQ_MEASURED, EP_IQ_SETPOINT,
	EP_AXIS_STATE, EP_ACTIVE_ERRORS, EP_DISARM_REASON, EP_ODOMETER, EP_CONTROL_MODE, EP_INPUT_MODE,
	EP_COMMUTATION, EP_INPUT_POS, EP_INPUT_VEL, EP_INPUT_TORQUE, EP_INPUT_VOLTAGE, EP_TORQUE_CONSTANT,
	EP_ID_SETPOINT, EP_FAST_SPEED,
	EP_SAVE_CONFIGURATION = 0x200, EP_CLEAR_ERRORS,
};

static uint8_t controlModeOdrive(void){
	static const uint8_t map[4] = { CM_VOLTAGE, CM_VELOCITY, CM_TORQUE, CM_POSITION };    //by CTRL_
	return map[CONTROL(DRIVEMODE)];
}

uint8_t SdoAccess(const uint8_t* d, uint8_t len, uint8_t* out){
	uint8_t write = d[0]==1;
	uint16_t ep = (uint16_t)(d[1]|d[2]<<8);
	uint32_t v = len>=8 ? rdU32(d+4) : 0;
	float f = len>=8 ? rdF(d+4) : 0.0f;
	uint8_t frame[8];
	uint32_t r = 0;
	if(write&&len<8&&ep<EP_SAVE_CONFIGURATION) return 0;
	if(ep<64){
		if(write){
			if(ep==MAGICNUM_POS||ep==63) return 0;
			if(ep==44) SetDriveMode((uint8_t)v); else pinstorage[ep] = (uint16_t)v;
		}
		r = pinstorage[ep];
	}else{
		float kt = TorqueConstant();
		switch(ep){
			case EP_VBUS: wrF((uint8_t*)&r, (float)fvbat/100.0f); break;
			case EP_IBUS: wrF((uint8_t*)&r, (float)BusCurrent()/100.0f); break;
			case EP_POS_ESTIMATE:
				if(write){ int32_t steps = clampF(f*(float)StepsPerTurn(), 2000000000); posOffset = iOdom-steps; posCmd = iOdom; }
				wrF((uint8_t*)&r, (float)(iOdom-posOffset)/(float)StepsPerTurn()); break;
			case EP_VEL_ESTIMATE: wrF((uint8_t*)&r, (float)realspeed/60.0f); break;
			case EP_IQ_MEASURED: case EP_IQ_SETPOINT:{
				uint8_t q[8]; getMsg(CMD_GET_IQ, q);
				memcpy(&r, ep==EP_IQ_MEASURED ? q+4 : q, 4); break;
			}
			case EP_AXIS_STATE:
				if(write){ if(v==AXIS_IDLE) axisState = AXIS_IDLE; else if(v==AXIS_CLOSED_LOOP&&!axisDisarm) axisState = AXIS_CLOSED_LOOP; }
				r = axisState; break;
			case EP_ACTIVE_ERRORS: r = axisErrors; break;
			case EP_DISARM_REASON: r = axisDisarm; break;
			case EP_ODOMETER: r = (uint32_t)iOdom; break;
			case EP_CONTROL_MODE: if(write) setControlMode(v); r = controlModeOdrive(); break;
			case EP_INPUT_MODE: if(write) inputMode = (uint8_t)v; r = inputMode; break;
			case EP_COMMUTATION:
				if(write&&v<=COMMUT_FOC) SetDriveMode((uint8_t)(v*4+CONTROL(DRIVEMODE)));
				r = COMMUTATION(DRIVEMODE); break;
			case EP_INPUT_POS:
				if(write){ posCmd = clampF(f*(float)StepsPerTurn(), 2000000000)+posOffset; inputArrived(); }
				wrF((uint8_t*)&r, (float)(posCmd-posOffset)/(float)StepsPerTurn()); break;
			case EP_INPUT_VEL:
				if(write&&CONTROL(DRIVEMODE)==CTRL_SPEED){ speed = clampF(f*60.0f, 1000); inputArrived(); }
				wrF((uint8_t*)&r, CONTROL(DRIVEMODE)==CTRL_SPEED ? (float)speed/60.0f : 0.0f); break;
			case EP_INPUT_TORQUE:
				if(write&&CONTROL(DRIVEMODE)==CTRL_TORQUE){ speed = clampF(f/kt*100.0f, 30000); inputArrived(); }
				wrF((uint8_t*)&r, CONTROL(DRIVEMODE)==CTRL_TORQUE ? (float)speed/100.0f*kt : 0.0f); break;
			case EP_INPUT_VOLTAGE:
				if(write&&CONTROL(DRIVEMODE)==CTRL_PWM){ speed = clampF(f*1000.0f, 1000); inputArrived(); }
				wrF((uint8_t*)&r, CONTROL(DRIVEMODE)==CTRL_PWM ? (float)speed/1000.0f : 0.0f); break;
			case EP_TORQUE_CONSTANT: wrF((uint8_t*)&r, kt); break;
			case EP_ID_SETPOINT: if(write) focIdRef = (int32_t)v; r = (uint32_t)focIdRef; break;
			case EP_FAST_SPEED: r = (uint32_t)fastspeed; break;
			case EP_SAVE_CONFIGURATION: if(!write) return 0; RemoteSaveConfig(); break;
			case EP_CLEAR_ERRORS: if(!write) return 0; axisDisarm = 0; FocClearTrip(); break;
			default: return 0;
		}
	}
	frame[0] = 0; frame[1] = (uint8_t)ep; frame[2] = (uint8_t)(ep>>8); frame[3] = 0;
	if(write){ memcpy(out, frame, 4); return 4; }
	wrU32(frame+4, r);
	memcpy(out, frame, 8);
	return 8;
}
