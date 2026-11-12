#!/usr/bin/env python3
"""Writes endpoints.json (RxSdo/TxSdo endpoint ids, like ODrive's flat_endpoints.json) from the config
field table in cfgfields.py and the runtime endpoints of Src/cansimple.c. Run after changing either."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cfgfields import FIELDS

FW = "2.0.0"
RUNTIME = [  # id, name (ODrive's where it has one), type, access, unit/meaning  (= the EP_ enum in Src/cansimple.c)
    (0x100, "vbus_voltage", "float", "r", "V"),
    (0x101, "ibus", "float", "r", "A, bus current"),
    (0x102, "axis0.pos_estimate", "float", "rw", "turns; writing shifts the position (Set_Absolute_Position)"),
    (0x103, "axis0.vel_estimate", "float", "r", "turns/s"),
    (0x104, "axis0.motor.foc.Iq_measured", "float", "r", "A (FOC; NaN otherwise)"),
    (0x105, "axis0.motor.foc.Iq_setpoint", "float", "r", "A"),
    (0x106, "axis0.current_state", "uint8", "rw", "1 IDLE, 8 CLOSED_LOOP_CONTROL; writing = requested_state"),
    (0x107, "axis0.active_errors", "uint32", "r", "ODrive Error bits"),
    (0x108, "axis0.disarm_reason", "uint32", "r", "ODrive Error bits"),
    (0x109, "hoverboard.odometer", "int32", "r", "hall steps since boot"),
    (0x10A, "axis0.controller.config.control_mode", "uint8", "rw", "0 VOLTAGE, 1 TORQUE, 2 VELOCITY, 3 POSITION"),
    (0x10B, "axis0.controller.config.input_mode", "uint8", "rw", "1 PASSTHROUGH, 2 VEL_RAMP, 3 POS_FILTER, 5 TRAP_TRAJ, 6 TORQUE_RAMP"),
    (0x10C, "hoverboard.commutation", "uint8", "rw", "0 block, 1 sine, 2 FOC"),
    (0x10D, "axis0.controller.input_pos", "float", "rw", "turns"),
    (0x10E, "axis0.controller.input_vel", "float", "rw", "turns/s"),
    (0x10F, "axis0.controller.input_torque", "float", "rw", "Nm"),
    (0x110, "hoverboard.input_voltage", "float", "rw", "fraction of the maximum voltage, -1..1"),
    (0x111, "hoverboard.torque_constant_in_use", "float", "r", "Nm/A"),
    (0x112, "hoverboard.id_setpoint", "int32", "rw", "mA, FOC d current"),
    (0x113, "hoverboard.fast_speed", "int32", "r", "rpm, last hall interval"),
    (0x114, "hoverboard.charger_connected", "uint8", "r", "1 while the charger is connected (charge pin; also heartbeat byte 7 bit 0)"),
    (0x115, "hoverboard.foot_sensors", "uint8", "r", "bit 0 / 1: infrared foot sensor ir1 / ir2 covered (PinFinder detect ir); also heartbeat byte 7 bits 1-2"),
    (0x120, "axis0.controller.config.vel_limit", "float", "rw", "turns/s, position and velocity modes"),
    (0x121, "axis0.controller.config.pos_gain", "float", "rw", "(turns/s)/turn: position loop; FOC default 20, then the hold within 1.5 hall steps"),
    (0x122, "axis0.controller.config.vel_gain", "float", "rw", "Nm/(turns/s), FOC velocity loop; 0 = default (position mode 3.3x the velocity mode's)"),
    (0x123, "axis0.controller.config.vel_integrator_gain", "float", "rw", "(Nm/s)/(turns/s), FOC velocity loop"),
    (0x124, "axis0.config.motor.current_soft_max", "float", "rw", "A, FOC current limit"),
    (0x125, "axis0.config.motor.current_hard_max", "float", "rw", "A, phase current trip (CURRENT_LIMIT_VIOLATION)"),
    (0x126, "axis0.config.motor.torque_constant", "float", "rw", "Nm/A; 0 = 8.27 / kv"),
    (0x127, "axis0.config.motor.pole_pairs", "uint32", "rw", "after a restart"),
    (0x128, "axis0.config.motor.phase_resistance", "float", "rw", "Ohm incl. mosfets (sine torque model)"),
    (0x129, "axis0.config.watchdog_timeout", "float", "rw", "s without a set point -> 0 / hold position"),
    (0x12A, "config.dc_bus_undervoltage_trip_level", "float", "rw", "V, low battery cut-off; <= 20 V off"),
    (0x12B, "config.dc_bus_overvoltage_trip_level", "float", "rw", "V, coasts above (battery full + 3 V)"),
    (0x12C, "axis0.config.can.node_id", "uint32", "rw", "the bus id (slave id), after a restart"),
    (0x12D, "config.uart0_baudrate", "uint32", "rw", "baud, after a restart"),
    (0x12E, "axis0.controller.config.vel_ramp_rate", "float", "rw", "turns/s^2, VEL_RAMP; 0 = default 1"),
    (0x12F, "axis0.controller.config.input_filter_bandwidth", "float", "rw", "1/s, POS_FILTER; 0 = default 2"),
    (0x130, "axis0.trap_traj.config.vel_limit", "float", "rw", "turns/s, TRAP_TRAJ; 0 = default 2"),
    (0x131, "axis0.trap_traj.config.accel_limit", "float", "rw", "turns/s^2, TRAP_TRAJ; 0 = default 0.5"),
    (0x132, "axis0.trap_traj.config.decel_limit", "float", "rw", "turns/s^2, TRAP_TRAJ; 0 = default 0.5"),
    (0x133, "hoverboard.foc.field_weakening_current", "float", "rw", "A, max negative d current at the voltage limit (FOC); 0 = off, at most 90 % of current_soft_max"),
    (0x134, "hoverboard.sine.field_weakening_advance", "float", "rw", "degrees electrical, max phase advance beyond 100 % modulation (sine); 0 = off"),
    (0x135, "axis0.controller.pos_setpoint", "float", "r", "turns, after the input mode"),
    (0x136, "axis0.controller.vel_setpoint", "float", "r", "turns/s, after the input mode (position: feed forward)"),
    (0x137, "hoverboard.foc.Id_measured", "float", "r", "A, d current (FOC, sine on phase current pins), ~7 ms average"),
    (0x138, "hoverboard.position.hold_current", "float", "rw", "A, FOC position hold stiffness (d current at the target angle); default 4, at most 90 % of current_soft_max"),
    (0x139, "hoverboard.position.hold_window", "float", "rw", "hall steps: the hold takes over within this of the target (and below 25 rpm); default 1.5"),
    (0x13A, "hoverboard.position.accel_limit", "float", "rw", "turns/s^2, FOC position approach acceleration and braking curve; default 20"),
    (0x13B, "axis0.controller.config.torque_ramp_rate", "float", "rw", "Nm/s, TORQUE_RAMP input mode; default 0.01"),
    (0x13C, "axis0.controller.config.inertia", "float", "rw", "Nm/(turn/s^2): torque feed forward of the TRAP_TRAJ / POS_FILTER acceleration (FOC); default 0"),
    (0x13D, "axis0.controller.config.vel_limit_tolerance", "float", "rw", "overspeed error above vel_limit times this; default 1.2"),
    (0x13E, "axis0.config.motor.phase_inductance", "float", "rw", "H, FOC decoupling and current_control_bandwidth; default 0.00046"),
    (0x13F, "hoverboard.position.hold_enable", "uint8", "rw", "1 = hold at the target's electrical angle (FOC position), 0 = position loop only; default 1"),
    (0x140, "axis0.controller.config.enable_overspeed_error", "uint8", "rw", "1 = VELOCITY_LIMIT_VIOLATION (disarm) above vel_limit * vel_limit_tolerance; default 0"),
    (0x141, "axis0.controller.config.enable_torque_mode_vel_limit", "uint8", "rw", "1 = torque control limits the speed to vel_limit (torque falls with vel_gain); default 1"),
    (0x143, "hoverboard.controller.pos_integrator_gain", "float", "rw", "((turns/s)/turn)/s: position I, within 4 hall steps of the target; default 0 (off)"),
    (0x144, "hoverboard.controller.pos_derivative_gain", "float", "rw", "(turns/s)/(turns/s): position D on trajectory speed - wheel speed; default 0 (off)"),
    (0x145, "hoverboard.controller.vel_derivative_gain", "float", "rw", "Nm/(turns/s^2): FOC velocity D on the measured speed (20 ms filter); default 0 (off)"),
    (0x142, "axis0.config.motor.current_control_bandwidth", "float", "rw", "rad/s: sets the current loop gains kp = L*bw, ki = R*bw (reads back kp / L); default gains ~870"),
    (0x150, "hoverboard.imu.state", "uint8", "r", "0 no IMU configured (imusda/imuscl/imuaddr 0x68/0x69), 1 running, 2 no answer, 3 pins not I2C1 SDA/SCL (pcb)"),
    (0x151, "hoverboard.imu.accel_x", "float", "r", "m/s^2 (+-4 g), 100 Hz, board axes after calibrate imu (X forward, Y left, Z up; else the chip's); NaN unless running"),
    (0x152, "hoverboard.imu.accel_y", "float", "r", "m/s^2"),
    (0x153, "hoverboard.imu.accel_z", "float", "r", "m/s^2"),
    (0x154, "hoverboard.imu.gyro_x", "float", "r", "deg/s (+-1000), board axes, bias from the first 0.5 s standing still"),
    (0x155, "hoverboard.imu.gyro_y", "float", "r", "deg/s"),
    (0x156, "hoverboard.imu.gyro_z", "float", "r", "deg/s"),
    (0x157, "hoverboard.imu.temperature", "float", "r", "C (MPU-6050 scale)"),
    (0x158, "hoverboard.imu.pitch", "float", "r", "deg, front down positive, 0 = the level of calibrate imu (gyro + accelerometer, 0.5 s)"),
    (0x159, "hoverboard.imu.roll", "float", "r", "deg, right side down positive, 0 = the level of calibrate imu"),
    (0x200, "save_configuration", "function", "w", "store everything (when the wheel stands still)"),
    (0x201, "clear_errors", "function", "w", ""),
    (0x202, "hoverboard.power_off", "function", "w", "switches the board off (the power button's shutdown; boards with a power latch)"),
]

eps = {}
for i, (name, kind, meaning) in sorted(FIELDS.items()):
    eps["hoverboard.config." + name] = dict(id=i, type="uint16", access="r" if i in (32, 63, 96) else "rw", unit=meaning)
for i, name, typ, acc, unit in RUNTIME:
    eps[name] = dict(id=i, type=typ, access=acc, unit=unit)
out = dict(fw_version=FW, protocol=2, endpoints=eps)
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "endpoints.json")
with open(p, "w") as f:
    json.dump(out, f, indent=1)
print("wrote %s: %d endpoints" % (p, len(eps)))
