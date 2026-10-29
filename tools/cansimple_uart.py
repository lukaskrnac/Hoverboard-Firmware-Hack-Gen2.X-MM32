#!/usr/bin/env python3
"""ODrive CANSimple messages over the RemoteUartBus (host side, reference implementation).

Frame: 0xA5 | node | cmd (bit 7 = request) | length 0..8 | payload | CRC-16/XMODEM LE over node..payload.
Message ids, payloads and units as ODrive's CANSimple protocol; see PROTOCOL.md of the web console and
the firmware's Src/cansimple.c. Endpoint ids for RxSdo/TxSdo: tools/endpoints.json.

    bus = Bus("/dev/ttyUSB0", 115200)
    bus.set_controller_mode(1, CONTROL_MODE["VELOCITY_CONTROL"], INPUT_MODE["PASSTHROUGH"])
    bus.set_input_vel(1, 2.0)                  # turns/s
    pos, vel = bus.get_encoder_estimates(1)    # turns, turns/s
"""
import json, os, struct, time
import serial

START = 0xA5
BROADCAST = 0x3F
CMD = dict(GET_VERSION=0x00, HEARTBEAT=0x01, ESTOP=0x02, GET_ERROR=0x03, RXSDO=0x04, TXSDO=0x05,
           SET_AXIS_STATE=0x07, GET_ENCODER_ESTIMATES=0x09, SET_CONTROLLER_MODE=0x0B, SET_INPUT_POS=0x0C,
           SET_INPUT_VEL=0x0D, SET_INPUT_TORQUE=0x0E, SET_LIMITS=0x0F, SET_TRAJ_VEL_LIMIT=0x11,
           SET_TRAJ_ACCEL_LIMITS=0x12, GET_IQ=0x14, GET_TEMPERATURE=0x15,
           REBOOT=0x16, GET_BUS_VOLTAGE_CURRENT=0x17, CLEAR_ERRORS=0x18, SET_ABSOLUTE_POSITION=0x19,
           SET_POS_GAIN=0x1A, SET_VEL_GAINS=0x1B, GET_TORQUES=0x1C, GET_POWERS=0x1D, SET_INPUT_VOLTAGE=0x1E)
AXIS_STATE = dict(IDLE=1, CLOSED_LOOP_CONTROL=8)
CONTROL_MODE = dict(VOLTAGE_CONTROL=0, TORQUE_CONTROL=1, VELOCITY_CONTROL=2, POSITION_CONTROL=3)
INPUT_MODE = dict(INACTIVE=0, PASSTHROUGH=1, VEL_RAMP=2, POS_FILTER=3, TRAP_TRAJ=5, TORQUE_RAMP=6)
ERRORS = {0x100: "DC_BUS_OVER_VOLTAGE", 0x200: "DC_BUS_UNDER_VOLTAGE", 0x400: "DC_BUS_OVER_CURRENT",
          0x20: "DRV_FAULT", 0x800: "DC_BUS_OVER_REGEN_CURRENT", 0x1000: "CURRENT_LIMIT_VIOLATION", 0x4000: "INVERTER_OVER_TEMP", 0x8000: "VELOCITY_LIMIT_VIOLATION", 0x1000000: "WATCHDOG_TIMER_EXPIRED", 0x2000000: "ESTOP_REQUESTED", 0x4000000: "SPINOUT_DETECTED"}


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(node, cmd, payload=b"", request=False):
    body = bytes([node, cmd | (0x80 if request else 0), len(payload)]) + payload
    return bytes([START]) + body + struct.pack("<H", crc16(body))


def parse(buf):
    """-> (frames [(node, cmd, payload)], rest of buf)"""
    out = []
    i = 0
    while i < len(buf):
        if buf[i] != START:
            i += 1; continue
        if i + 4 > len(buf): break
        n = buf[i + 3]
        if n > 8:
            i += 1; continue
        if i + 6 + n > len(buf): break
        body = buf[i + 1:i + 4 + n]
        if struct.unpack("<H", buf[i + 4 + n:i + 6 + n])[0] == crc16(body):
            out.append((body[0], body[1] & 0x7F, bytes(body[3:])))
            i += 6 + n
        else:
            i += 1
    return out, buf[i:]


def load_endpoints():
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "endpoints.json")
    with open(p) as f:
        return json.load(f)


class Bus:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.05):
        self.s = serial.Serial(port, baud, timeout=0.002)
        self.timeout = timeout
        self.rx = b""
        try:
            self.endpoints = load_endpoints()["endpoints"]
        except OSError:
            self.endpoints = {}

    def send(self, node, cmd, payload=b""):
        self.s.write(frame(node, CMD[cmd] if isinstance(cmd, str) else cmd, payload))

    def request(self, node, cmd, payload=b"", rtr=True):
        c = CMD[cmd] if isinstance(cmd, str) else cmd
        want = CMD["TXSDO"] if c == CMD["RXSDO"] else c
        self.s.reset_input_buffer(); self.rx = b""
        self.s.write(frame(node, c, payload, request=rtr))
        t_end = time.time() + self.timeout
        while time.time() < t_end:
            self.rx += self.s.read(64)
            frames, self.rx = parse(self.rx)
            for n, cmd, data in frames:
                if n == node and cmd == want:
                    return data
        return None

    # --- requests ---
    def get_version(self, node):
        d = self.request(node, "GET_VERSION")
        return None if d is None else dict(protocol=d[0], hw="%d.%d.%d" % tuple(d[1:4]), fw="%d.%d.%d" % tuple(d[4:7]), unreleased=d[7])

    def heartbeat(self, node):
        d = self.request(node, "HEARTBEAT")
        if d is None: return None
        err, state, proc, done, charger = struct.unpack("<IBBBB", d[:8])
        return dict(errors=err, error_names=[v for k, v in ERRORS.items() if err & k], state=state, procedure_result=proc,
                    trajectory_done=done, charger_connected=charger & 1,    # byte 7: hoverboard extension
                    foot_sensors=(charger >> 1) & 3)

    def get_error(self, node):
        d = self.request(node, "GET_ERROR")
        return None if d is None else dict(zip(("active_errors", "disarm_reason"), struct.unpack("<II", d)))

    def _ff(self, node, cmd):
        d = self.request(node, cmd)
        return None if d is None else struct.unpack("<ff", d)

    def get_encoder_estimates(self, node): return self._ff(node, "GET_ENCODER_ESTIMATES")
    def get_iq(self, node): return self._ff(node, "GET_IQ")
    def get_bus_voltage_current(self, node): return self._ff(node, "GET_BUS_VOLTAGE_CURRENT")
    def get_torques(self, node): return self._ff(node, "GET_TORQUES")
    def get_powers(self, node): return self._ff(node, "GET_POWERS")

    # --- set ---
    def set_axis_state(self, node, state): self.send(node, "SET_AXIS_STATE", struct.pack("<I", state))
    def set_controller_mode(self, node, control, inp=1): self.send(node, "SET_CONTROLLER_MODE", struct.pack("<II", control, inp))
    def set_input_pos(self, node, pos, vel_ff=0.0, torque_ff=0.0):
        self.send(node, "SET_INPUT_POS", struct.pack("<fhh", pos, int(vel_ff * 1000), int(torque_ff * 1000)))
    def set_input_vel(self, node, vel, torque_ff=0.0): self.send(node, "SET_INPUT_VEL", struct.pack("<ff", vel, torque_ff))
    def set_input_torque(self, node, torque): self.send(node, "SET_INPUT_TORQUE", struct.pack("<f", torque))
    def set_input_voltage(self, node, fraction): self.send(node, "SET_INPUT_VOLTAGE", struct.pack("<f", fraction))
    def set_limits(self, node, vel_limit, current_limit): self.send(node, "SET_LIMITS", struct.pack("<ff", vel_limit, current_limit))
    def set_traj_vel_limit(self, node, v): self.send(node, "SET_TRAJ_VEL_LIMIT", struct.pack("<f", v))
    def set_traj_accel_limits(self, node, a, d): self.send(node, "SET_TRAJ_ACCEL_LIMITS", struct.pack("<ff", a, d))
    def set_absolute_position(self, node, pos): self.send(node, "SET_ABSOLUTE_POSITION", struct.pack("<f", pos))
    def set_pos_gain(self, node, g): self.send(node, "SET_POS_GAIN", struct.pack("<f", g))
    def set_vel_gains(self, node, g, gi): self.send(node, "SET_VEL_GAINS", struct.pack("<ff", g, gi))
    def clear_errors(self, node): self.send(node, "CLEAR_ERRORS", b"\x00")
    def estop(self, node): self.send(node, "ESTOP")
    def save_configuration(self, node): self.send(node, "REBOOT", b"\x01")

    # --- parameters ---
    def _ep(self, name_or_id):
        if isinstance(name_or_id, int): return name_or_id, "uint32"
        e = self.endpoints[name_or_id]
        return e["id"], e["type"]

    def read(self, name_or_id):
        ep, typ = self._ep(name_or_id)
        d = self.request(self.node, "RXSDO", struct.pack("<BHB", 0, ep, 0), rtr=False)
        if d is None or len(d) < 8: return None
        return struct.unpack("<f" if typ == "float" else ("<i" if typ.startswith("int") else "<I"), d[4:8])[0]

    def write(self, name_or_id, value):
        ep, typ = self._ep(name_or_id)
        v = struct.pack("<f", value) if typ == "float" else struct.pack("<i" if typ.startswith("int") else "<I", int(value))
        return self.request(self.node, "RXSDO", struct.pack("<BHB", 1, ep, 0) + v, rtr=False) is not None

    def call(self, name_or_id):
        ep, _ = self._ep(name_or_id)
        return self.request(self.node, "RXSDO", struct.pack("<BHB", 1, ep, 0), rtr=False) is not None

    node = 1
