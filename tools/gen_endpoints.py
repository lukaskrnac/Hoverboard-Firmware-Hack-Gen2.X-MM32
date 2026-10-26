#!/usr/bin/env python3
"""Writes endpoints.json (RxSdo/TxSdo endpoint ids, like ODrive's flat_endpoints.json) from the config
field table in mm32config.py and the runtime endpoints of Src/cansimple.c. Run after changing either."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mm32config import FIELDS

FW = "2.0.0"
RUNTIME = [  # id, name, type, access, unit/meaning  (order = the EP_ enum in Src/cansimple.c)
    (0x100, "vbus_voltage", "float", "r", "V"),
    (0x101, "ibus", "float", "r", "A, bus current"),
    (0x102, "pos_estimate", "float", "rw", "turns; writing sets the position (Set_Absolute_Position)"),
    (0x103, "vel_estimate", "float", "r", "turns/s"),
    (0x104, "iq_measured", "float", "r", "A (FOC; NaN otherwise)"),
    (0x105, "iq_setpoint", "float", "r", "A"),
    (0x106, "axis_state", "uint8", "rw", "1 IDLE, 8 CLOSED_LOOP_CONTROL"),
    (0x107, "active_errors", "uint32", "r", "ODrive Error bits"),
    (0x108, "disarm_reason", "uint32", "r", "ODrive Error bits"),
    (0x109, "odometer", "int32", "r", "hall steps since boot"),
    (0x10A, "control_mode", "uint8", "rw", "0 VOLTAGE, 1 TORQUE, 2 VELOCITY, 3 POSITION"),
    (0x10B, "input_mode", "uint8", "rw", "1 PASSTHROUGH, 2 VEL_RAMP, 3 POS_FILTER, 5 TRAP_TRAJ"),
    (0x10C, "commutation", "uint8", "rw", "0 block, 1 sine, 2 FOC"),
    (0x10D, "input_pos", "float", "rw", "turns"),
    (0x10E, "input_vel", "float", "rw", "turns/s"),
    (0x10F, "input_torque", "float", "rw", "Nm"),
    (0x110, "input_voltage", "float", "rw", "fraction of the maximum voltage, -1..1"),
    (0x111, "torque_constant", "float", "r", "Nm/A in use"),
    (0x112, "id_setpoint", "int32", "rw", "mA, FOC d current"),
    (0x113, "fast_speed", "int32", "r", "rpm, last hall interval"),
    (0x200, "save_configuration", "function", "w", "store the config words (when the wheel stands still)"),
    (0x201, "clear_errors", "function", "w", ""),
]

eps = {}
for i, (name, kind, meaning) in sorted(FIELDS.items()):
    eps["config." + name] = dict(id=i, type="uint16", access="r" if i in (32, 63) else "rw", unit=meaning)
for i, name, typ, acc, unit in RUNTIME:
    eps[name] = dict(id=i, type=typ, access=acc, unit=unit)
out = dict(fw_version=FW, protocol=2, endpoints=eps)
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "endpoints.json")
with open(p, "w") as f:
    json.dump(out, f, indent=1)
print("wrote %s: %d endpoints" % (p, len(eps)))
