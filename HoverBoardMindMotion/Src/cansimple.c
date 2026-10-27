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
//Get_Encoder_Estimates, Set_Controller_Mode, Set_Input_Pos/Vel/Torque, Set_Limits, Set_Traj_Vel_Limit,
//Set_Traj_Accel_Limits, Get_Iq,
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
#include "inputmode.h"
#include "imu.h"

#define START 0xA5
#define BROADCAST 0x3F

enum{
	CMD_GET_VERSION = 0x00, CMD_HEARTBEAT = 0x01, CMD_ESTOP = 0x02, CMD_GET_ERROR = 0x03, CMD_RXSDO = 0x04,
	CMD_TXSDO = 0x05, CMD_SET_AXIS_STATE = 0x07, CMD_GET_ENCODER_ESTIMATES = 0x09,
	CMD_SET_CONTROLLER_MODE = 0x0B, CMD_SET_INPUT_POS = 0x0C, CMD_SET_INPUT_VEL = 0x0D,
	CMD_SET_INPUT_TORQUE = 0x0E, CMD_SET_LIMITS = 0x0F, CMD_SET_TRAJ_VEL_LIMIT = 0x11,
	CMD_SET_TRAJ_ACCEL_LIMITS = 0x12, CMD_GET_IQ = 0x14, CMD_GET_TEMPERATURE = 0x15,
	CMD_REBOOT = 0x16, CMD_GET_BUS_VOLTAGE_CURRENT = 0x17, CMD_CLEAR_ERRORS = 0x18,
	CMD_SET_ABSOLUTE_POSITION = 0x19, CMD_SET_POS_GAIN = 0x1A, CMD_SET_VEL_GAINS = 0x1B,
	CMD_GET_TORQUES = 0x1C, CMD_GET_POWERS = 0x1D, CMD_SET_INPUT_VOLTAGE = 0x1E,
};
enum{ CM_VOLTAGE = 0, CM_TORQUE = 1, CM_VELOCITY = 2, CM_POSITION = 3 };    //ODrive ControlMode

//RxSdo / TxSdo: parameter access by endpoint id (the names, types and units are in tools/endpoints.json,
//as ODrive's flat_endpoints.json). Request: opcode u8 (0 read, 1 write), endpoint u16, reserved u8,
//value (4 bytes). Answer: 0, endpoint u16, 0, value (read); a write is confirmed without a value.
//  0x000..0x07F  config words (pinstorage, u16 in a u32; 64.. the extension record); save_configuration stores them
//  0x100..       runtime values, float32 unless noted
//  0x200..       functions (write, no value)
enum{
	EP_VBUS = 0x100, EP_IBUS, EP_POS_ESTIMATE, EP_VEL_ESTIMATE, EP_IQ_MEASURED, EP_IQ_SETPOINT,
	EP_AXIS_STATE, EP_ACTIVE_ERRORS, EP_DISARM_REASON, EP_ODOMETER, EP_CONTROL_MODE, EP_INPUT_MODE,
	EP_COMMUTATION, EP_INPUT_POS, EP_INPUT_VEL, EP_INPUT_TORQUE, EP_INPUT_VOLTAGE, EP_TORQUE_CONSTANT,
	EP_ID_SETPOINT, EP_FAST_SPEED, EP_CHARGER,
	//ODrive parameters (float, ODrive units), stored in the config words
	EP_VEL_LIMIT = 0x120, EP_POS_GAIN, EP_VEL_GAIN, EP_VEL_INTEGRATOR_GAIN, EP_CURRENT_SOFT_MAX,
	EP_CURRENT_HARD_MAX, EP_TORQUE_CONSTANT_CFG, EP_POLE_PAIRS, EP_PHASE_RESISTANCE, EP_WATCHDOG_TIMEOUT,
	EP_UNDERVOLTAGE, EP_OVERVOLTAGE, EP_NODE_ID, EP_UART_BAUDRATE,
	EP_VEL_RAMP_RATE, EP_INPUT_FILTER_BW, EP_TRAJ_VEL_LIMIT, EP_TRAJ_ACCEL_LIMIT, EP_TRAJ_DECEL_LIMIT,
	EP_FW_CURRENT, EP_FW_ADVANCE, EP_POS_SETPOINT, EP_VEL_SETPOINT, EP_ID_MEASURED,
	//extension record (config words 64..): scaled like VEL_RAMP_RATE.., then bools (uint8), bandwidth
	EP_HOLD_CURRENT = 0x138, EP_HOLD_WINDOW, EP_POS_ACCEL, EP_TORQUE_RAMP_RATE, EP_INERTIA, EP_VEL_LIMIT_TOL,
	EP_PHASE_L, EP_HOLD_ENABLE, EP_OVERSPEED_ERROR, EP_TORQUE_VEL_LIMIT, EP_CURRENT_BW,
	EP_POS_INTEGRATOR_GAIN, EP_POS_DERIVATIVE_GAIN, EP_VEL_DERIVATIVE_GAIN,
	//IMU (imu.c): state uint8, then float: accel m/s^2, gyro deg/s, temperature C
	EP_IMU_STATE = 0x150, EP_IMU_ACCEL_X, EP_IMU_ACCEL_Y, EP_IMU_ACCEL_Z, EP_IMU_GYRO_X, EP_IMU_GYRO_Y, EP_IMU_GYRO_Z,
	EP_IMU_TEMPERATURE,
	EP_SAVE_CONFIGURATION = 0x200, EP_CLEAR_ERRORS,
};

static int32_t posOffset;    //odometer at position 0 (Set_Absolute_Position), hall steps
static float posTurns(void){ return (float)(hallPosFine-posOffset*POS_FINE)/((float)StepsPerTurn()*POS_FINE); }
static uint8_t odriveParam(uint16_t ep, uint8_t write, float f, float* val);
static void setInputMode(uint32_t m){ if(InputModeSupported(m)) INPUT_MODE = (uint16_t)m; }

static float rdF(const uint8_t* p){ float f; memcpy(&f, p, 4); return f; }
static uint32_t rdU32(const uint8_t* p){ return p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static void wrF(uint8_t* p, float f){ memcpy(p, &f, 4); }
static void wrU32(uint8_t* p, uint32_t v){ p[0] = (uint8_t)v; p[1] = (uint8_t)(v>>8); p[2] = (uint8_t)(v>>16); p[3] = (uint8_t)(v>>24); }
static int32_t clampF(float f, int32_t lim){ return f>lim ? lim : (f<-lim ? -lim : (int32_t)(f<0 ? f-0.5f : f+0.5f)); }

//a config word from a float: 0..65534, rounded. Negative and NaN give 0: the cast wrapped them (-2.5 V
//written as the undervoltage level became 63.0 V, a low battery cut-off above the bus voltage)
static uint16_t toWord(float f){ return !(f>0) ? 0 : (uint16_t)clampF(f, 65534); }

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
			d[4] = gatesoff ? AXIS_IDLE : AXIS_CLOSED_LOOP;    //the state in effect (v1 frames arm with bit 64)
			d[5] = 0; d[6] = posDone; d[7] = charging;    //byte 7 (unused by ODrive): charger connected
			return 8;
		case CMD_GET_ERROR:
			wrU32(d, axisErrors); wrU32(d+4, axisDisarm);
			return 8;
		case CMD_GET_ENCODER_ESTIMATES:
			wrF(d, posTurns());
			wrF(d+4, (float)realspeed/60.0f);
			return 8;
		case CMD_GET_IQ:{
			float set = __builtin_nanf(""), meas = __builtin_nanf("");
			if(COMMUTATION(DRIVEMODE)==COMMUT_FOC) set = (float)FocIqSetpoint()/1000.0f;
			else if(CONTROL(DRIVEMODE)==CTRL_TORQUE) set = (float)speed/100.0f;
			if(FocSensing()) meas = (float)focIqF/1000.0f;    //FOC, and sine on phase current pins
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
			float tq = FocSensing() ? (float)focIqF/1000.0f*kt : __builtin_nanf("");
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
				//the axis state takes effect for CANSimple commands only (v1: state bit 64): a board that
				//had no set point yet (boot) ignored IDLE and kept driving
				uint32_t st = rdU32(d);
				cmdSource = 2;
				if(st==AXIS_IDLE) axisState = AXIS_IDLE;
				else if(st==AXIS_CLOSED_LOOP&&!axisDisarm){ axisState = AXIS_CLOSED_LOOP; RemoteFeedWatchdog(); }
			}
			break;
		case CMD_SET_CONTROLLER_MODE:
			if(len>=8){
				setControlMode(rdU32(d));
				setInputMode(rdU32(d+4));
			}
			break;
		case CMD_SET_INPUT_POS:
			if(len>=4){
				float steps = rdF(d)*(float)StepsPerTurn();
				posCmd = clampF(steps*POS_FINE, 2000000000)+posOffset*POS_FINE;
				posVelFf = len>=6 ? (int16_t)(d[4]|d[5]<<8)*6/100 : 0;    //0.001 turns/s -> rpm
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
		case CMD_SET_TRAJ_VEL_LIMIT:
			if(len>=4){ float x; odriveParam(EP_TRAJ_VEL_LIMIT, 1, rdF(d), &x); }
			break;
		case CMD_SET_TRAJ_ACCEL_LIMITS:
			if(len>=8){ float x; odriveParam(EP_TRAJ_ACCEL_LIMIT, 1, rdF(d), &x); odriveParam(EP_TRAJ_DECEL_LIMIT, 1, rdF(d+4), &x); }
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
				posCmd = hallPosFine;    //no jump: the target is where the wheel is
			}
			break;
		case CMD_SET_POS_GAIN:    //(turns/s) per turn -> 0.1 rpm per hall step
			if(len>=4) POS_KP = toWord(rdF(d)*600.0f/(float)StepsPerTurn());
			break;
		case CMD_SET_VEL_GAINS:{    //Nm/(turns/s), (Nm/s)/(turns/s) -> FOC speed loop mA/rpm, mA/(rpm*s)
			if(len>=8){
				float k = 1000.0f/60.0f/TorqueConstant();
				FOC_SPD_KP = toWord(rdF(d)*k);
				FOC_SPD_KI = toWord(rdF(d+4)*k);
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

static uint8_t controlModeOdrive(void){
	static const uint8_t map[4] = { CM_VOLTAGE, CM_VELOCITY, CM_TORQUE, CM_POSITION };    //by CTRL_
	return map[CONTROL(DRIVEMODE)];
}


//config words of EP_VEL_RAMP_RATE..EP_FW_ADVANCE and EP_HOLD_CURRENT..EP_PHASE_L in ODrive units:
//value = word / 10^e, 0/65535 = def / 10^e
static const float p10[] = { 1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 100000.0f, 1000000.0f };
static const struct{ uint8_t w, e; uint16_t def; } scaled[] = {
	{29, 2, 100}, {30, 2, 200}, {31, 2, 200}, {35, 2, 50}, {43, 2, 50},    //input modes
	{21, 1, 0}, {22, 0, 0},    //field weakening: FW_IMAX 0.1 A, FW_ADVANCE degrees; 0 = off
	{64, 1, 40}, {66, 2, 150}, {67, 2, 2000},    //hold_current A, hold_window steps, accel_limit turns/s^2
	{68, 2, 1}, {69, 4, 0}, {70, 2, 120}, {73, 6, 460},    //torque_ramp_rate, inertia, vel_limit_tolerance, phase_inductance H
};

//ODrive parameter <-> config word, value in ODrive units; returns 0 if ep is not one of them
static uint8_t odriveParam(uint16_t ep, uint8_t write, float f, float* val){
	float kt = TorqueConstant(), spt = (float)StepsPerTurn();
	int8_t k = ep>=EP_VEL_RAMP_RATE&&ep<=EP_FW_ADVANCE ? (int8_t)(ep-EP_VEL_RAMP_RATE) :
	           (ep>=EP_HOLD_CURRENT&&ep<=EP_PHASE_L ? (int8_t)(ep-EP_HOLD_CURRENT+7) : -1);
	if(k>=0){
		const uint8_t w = scaled[k].w; float sc = p10[scaled[k].e];
		if(write) pinstorage[w] = toWord(f*sc);
		*val = (float)cfgOrDefault(pinstorage[w], scaled[k].def)/sc; return 1;
	}
	if(ep>=EP_HOLD_ENABLE&&ep<=EP_TORQUE_VEL_LIMIT){    //bools: word 1 false, 2 true, 0/65535 default
		static const uint8_t bw[3] = { 65, 71, 72 }, bd[3] = { 1, 0, 1 };
		const uint8_t w = bw[ep-EP_HOLD_ENABLE];
		if(write) pinstorage[w] = f>0.5f ? 2 : 1;
		*val = (float)cfgBool(pinstorage[w], bd[ep-EP_HOLD_ENABLE]); return 1;
	}
	switch(ep){
		case EP_CURRENT_BW:{    //rad/s: current loop kp = L * bw (mV/A), ki = R * bw (V/(A s)), as ODrive
			float l = (float)cfgOrDefault(PHASE_L, 460);    //uH
			if(write){ CUR_KP = toWord(f*l/1000.0f); CUR_KI = toWord(f*(float)cfgOrDefault(MOTOR_R, 300)/1000.0f); }
			*val = (float)cfgOrDefault(CUR_KP, 400)*1000.0f/l; return 1;
		}
		case EP_VEL_LIMIT:    //turns/s <- rpm
			if(write) POS_RPM = toWord(f*60.0f);
			*val = (float)cfgOrDefault(POS_RPM, 300)/60.0f; return 1;
		case EP_POS_GAIN:    //(turns/s)/turn <- 0.1 rpm per hall step
			if(write) POS_KP = toWord(f*600.0f/spt);
			*val = (float)PosGain()*spt/600.0f; return 1;
		case EP_VEL_GAIN:    //Nm/(turns/s) <- FOC speed loop mA/rpm
			if(write) FOC_SPD_KP = toWord(f/(0.06f*kt));
			*val = (float)cfgOrDefault(FOC_SPD_KP, 30)*0.06f*kt; return 1;
		case EP_POS_INTEGRATOR_GAIN:    //((turns/s)/turn)/s <- 0.1 rpm per (hall step * s)
			if(write) POS_KI = toWord(f*600.0f/spt);
			*val = (float)cfgOrDefault(POS_KI, 0)*spt/600.0f; return 1;
		case EP_POS_DERIVATIVE_GAIN:    //(turns/s)/(turns/s) <- 0.01
			if(write) POS_KD = toWord(f*100.0f);
			*val = (float)cfgOrDefault(POS_KD, 0)/100.0f; return 1;
		case EP_VEL_DERIVATIVE_GAIN:    //Nm/(turns/s^2) <- 0.01 mA per rpm/s
			if(write) FOC_SPD_KD = toWord(f/(0.0006f*kt));
			*val = (float)cfgOrDefault(FOC_SPD_KD, 0)*0.0006f*kt; return 1;
		case EP_VEL_INTEGRATOR_GAIN:    //(Nm/s)/(turns/s) <- mA/(rpm*s)
			if(write) FOC_SPD_KI = toWord(f/(0.06f*kt));
			*val = (float)cfgOrDefault(FOC_SPD_KI, 100)*0.06f*kt; return 1;
		case EP_CURRENT_SOFT_MAX:    //A <- 0.1 A
			if(write) FOC_IMAX = toWord(f*10.0f);
			*val = (float)cfgOrDefault(FOC_IMAX, 100)/10.0f; return 1;
		case EP_CURRENT_HARD_MAX:    //A <- 0.1 A
			if(write) CUR_TRIP = toWord(f*10.0f);
			*val = (float)CurrentHardMax()/1000.0f; return 1;
		case EP_TORQUE_CONSTANT_CFG:    //Nm/A <- mNm/A
			if(write) TORQUE_CONST = toWord(f*1000.0f);
			*val = kt; return 1;
		case EP_POLE_PAIRS:
			if(write) WINDINGS = toWord(f*2.0f);
			*val = (float)WINDINGS/2.0f; return 1;
		case EP_PHASE_RESISTANCE:    //Ohm <- mOhm
			if(write) MOTOR_R = toWord(f*1000.0f);
			*val = (float)cfgOrDefault(MOTOR_R, 300)/1000.0f; return 1;
		case EP_WATCHDOG_TIMEOUT:    //s <- ms
			if(write) SERIAL_TIMEOUT = toWord(f*1000.0f);
			*val = (float)SERIAL_TIMEOUT/1000.0f; return 1;
		case EP_UNDERVOLTAGE:    //V <- mV (low battery cut-off; <= 20 V: off)
			if(write) BAT_EMPTY = toWord(f*1000.0f);
			*val = (float)BAT_EMPTY/1000.0f; return 1;
		case EP_OVERVOLTAGE:    //V: coast above battery full + 3 V
			if(write) BAT_FULL = toWord((f-3.0f)*1000.0f);
			*val = (float)BAT_FULL/1000.0f+3.0f; return 1;
		case EP_NODE_ID:
			if(write) SLAVE_ID = toWord(f);
			*val = (float)SLAVE_ID; return 1;
		case EP_UART_BAUDRATE:    //baud <- baud/100 (after a restart)
			if(write) BAUD = toWord(f/100.0f);
			*val = (float)BAUD*100.0f; return 1;
	}
	return 0;
}

uint8_t SdoAccess(const uint8_t* d, uint8_t len, uint8_t* out){
	uint8_t write = d[0]==1;
	uint16_t ep = (uint16_t)(d[1]|d[2]<<8);
	uint32_t v = len>=8 ? rdU32(d+4) : 0;
	float f = len>=8 ? rdF(d+4) : 0.0f;
	uint8_t frame[8];
	uint32_t r = 0;
	if(write&&len<8&&ep<EP_SAVE_CONFIGURATION) return 0;
	float pv;
	uint8_t isInt = ep==EP_POLE_PAIRS||ep==EP_NODE_ID||ep==EP_UART_BAUDRATE||(ep>=EP_HOLD_ENABLE&&ep<=EP_TORQUE_VEL_LIMIT);
	if(odriveParam(ep, write, isInt ? (float)v : f, &pv)){
		if(isInt) r = (uint32_t)pv; else wrF((uint8_t*)&r, pv);
	}else if(ep<128){
		if(write){
			if(ep==MAGICNUM_POS||ep==63||ep==EXT_MAGIC_POS) return 0;
			if(ep==44) SetDriveMode((uint8_t)v); else pinstorage[ep] = (uint16_t)v;
		}
		r = pinstorage[ep];
	}else{
		float kt = TorqueConstant();
		switch(ep){
			case EP_VBUS: wrF((uint8_t*)&r, (float)fvbat/100.0f); break;
			case EP_IBUS: wrF((uint8_t*)&r, (float)BusCurrent()/100.0f); break;
			case EP_POS_ESTIMATE:
				if(write){ int32_t steps = clampF(f*(float)StepsPerTurn(), 2000000000); posOffset = iOdom-steps; posCmd = hallPosFine; }
				wrF((uint8_t*)&r, posTurns()); break;
			case EP_VEL_ESTIMATE: wrF((uint8_t*)&r, (float)realspeed/60.0f); break;
			case EP_IQ_MEASURED: case EP_IQ_SETPOINT:{
				uint8_t q[8]; getMsg(CMD_GET_IQ, q);
				memcpy(&r, ep==EP_IQ_MEASURED ? q+4 : q, 4); break;
			}
			case EP_AXIS_STATE:
				if(write){ cmdSource = 2; if(v==AXIS_IDLE) axisState = AXIS_IDLE; else if(v==AXIS_CLOSED_LOOP&&!axisDisarm) axisState = AXIS_CLOSED_LOOP; }
				r = axisState; break;
			case EP_ACTIVE_ERRORS: r = axisErrors; break;
			case EP_DISARM_REASON: r = axisDisarm; break;
			case EP_ODOMETER: r = (uint32_t)iOdom; break;
			case EP_CONTROL_MODE: if(write) setControlMode(v); r = controlModeOdrive(); break;
			case EP_INPUT_MODE: if(write) setInputMode(v); r = InputMode(); break;
			case EP_POS_SETPOINT: wrF((uint8_t*)&r, (inPosSp/POS_FINE-(float)posOffset)/(float)StepsPerTurn()); break;
			case EP_VEL_SETPOINT: wrF((uint8_t*)&r, inVelSp/POS_FINE/(float)StepsPerTurn()); break;
			case EP_ID_MEASURED: wrF((uint8_t*)&r, FocSensing() ? (float)focIdF/1000.0f : __builtin_nanf("")); break;
			case EP_COMMUTATION:
				if(write&&v<=COMMUT_FOC) SetDriveMode((uint8_t)(v*4+CONTROL(DRIVEMODE)));
				r = COMMUTATION(DRIVEMODE); break;
			case EP_INPUT_POS:
				if(write){ posCmd = clampF(f*(float)StepsPerTurn()*POS_FINE, 2000000000)+posOffset*POS_FINE; inputArrived(); }
				wrF((uint8_t*)&r, (float)(posCmd-posOffset*POS_FINE)/((float)StepsPerTurn()*POS_FINE)); break;
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
			case EP_CHARGER: r = charging; break;
			case EP_IMU_STATE: r = imuState; break;
			case EP_SAVE_CONFIGURATION: if(!write) return 0; RemoteSaveConfig(); break;
			case EP_CLEAR_ERRORS: if(!write) return 0; axisDisarm = 0; FocClearTrip(); break;
			default:
				if(ep<EP_IMU_ACCEL_X||ep>EP_IMU_TEMPERATURE) return 0;
				wrF((uint8_t*)&r, ImuValue((uint8_t)(ep-EP_IMU_ACCEL_X)));
		}
	}
	frame[0] = 0; frame[1] = (uint8_t)ep; frame[2] = (uint8_t)(ep>>8); frame[3] = 0;
	if(write){ memcpy(out, frame, 4); return 4; }
	wrU32(frame+4, r);
	memcpy(out, frame, 8);
	return 8;
}
