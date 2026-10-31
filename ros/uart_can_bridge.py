#!/usr/bin/env python3
"""SocketCAN <-> CANSimple-over-UART bridge: makes hoverboard boards look like ODrives on a CAN bus.

The firmware speaks ODrive's CANSimple messages, but over a UART and only when polled (several boards share
the line, so they cannot send cyclically). ODrive's ROS 2 driver (odriverobotics/ros_odrive: odrive_node and
odrive_ros2_control) talks to SocketCAN and expects the cyclic messages an ODrive sends by itself. This bridge:

  - polls every board for the cyclic messages at fixed rates (Heartbeat, Get_Encoder_Estimates, Get_Iq,
    Get_Torques, Get_Bus_Voltage_Current, Get_Temperature, Get_Error) and puts the answers on the CAN
    interface as if the ODrive had sent them,
  - forwards every frame addressed to a bridged node from CAN to the UART (Set_ messages, Clear_Errors,
    Estop, RxSdo); RTR frames become requests and their answers (and TxSdo replies) go back on CAN.

Commands go before polls; one request is outstanding at a time (the boards answer on a shared line).
A board that stops answering stops producing frames, so the driver sees it as gone.

    sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up      # once per boot (or --setup-vcan)
    python3 ros/uart_can_bridge.py --port /dev/ttyUSB0 --nodes 0 1
    ros2 launch ... odrive_node with interface:=vcan0 node_id:=0

A real CAN interface (can0) works the same way: the boards then appear next to real ODrives.
Needs pyserial and Linux (AF_CAN sockets).
"""
import argparse, os, select, socket, struct, subprocess, sys, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from cansimple_uart import frame, parse  # noqa: E402  (same framing as the firmware)
import serial  # noqa: E402

CAN_RTR_FLAG = 0x40000000
CAN_FMT = "=IB3x8s"
CAN_SIZE = struct.calcsize(CAN_FMT)

HEARTBEAT, GET_ERROR, RXSDO, TXSDO = 0x01, 0x03, 0x04, 0x05
ENCODER, IQ, TEMP, BUS, TORQUES, POWERS = 0x09, 0x14, 0x15, 0x17, 0x1C, 0x1D
GET_CMDS = {0x00, HEARTBEAT, GET_ERROR, ENCODER, IQ, TEMP, BUS, TORQUES, POWERS}
# cyclic messages and default rates in Hz (ODrive: heartbeat 10 Hz, encoder 100 Hz; the rest off by
# default, ros_odrive asks for them to be enabled). Lower them with many boards on one 115200 baud line:
# one poll is ~20 bytes, ~1.7 ms.
RATES = dict(heartbeat=(HEARTBEAT, 10), encoder=(ENCODER, 50), iq=(IQ, 10), torques=(TORQUES, 10),
             bus=(BUS, 2), temp=(TEMP, 1), error=(GET_ERROR, 2))


def setup_vcan(name):
    if os.path.exists(f"/sys/class/net/{name}"):
        return
    for cmd in (["ip", "link", "add", "dev", name, "type", "vcan"], ["ip", "link", "set", name, "up"]):
        subprocess.run((["sudo"] if os.geteuid() else []) + cmd, check=True)


class Bridge:
    def __init__(self, a):
        self.a = a
        self.ser = serial.Serial(a.port, a.baud, timeout=0)
        try:  # FTDI adapters hold received bytes up to 16 ms by default; low latency sets 1 ms (no root needed)
            self.ser.set_low_latency_mode(True)
        except (AttributeError, OSError, ValueError):
            pass
        self.can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can.bind((a.can,))
        self.nodes = set(a.nodes)
        self.cmds = []           # (node, cmd, payload, rtr) from CAN, sent before polls
        self.rx = b""
        now = time.monotonic()
        self.polls = []          # [next_due, period, node, cmd]
        for node in a.nodes:
            for name, (cmd, default) in RATES.items():
                hz = getattr(a, name)
                if hz > 0:
                    self.polls.append([now, 1.0 / hz, node, cmd])
        self.alive = {n: 0.0 for n in a.nodes}
        self.stats = dict(polls=0, answers=0, timeouts=0, to_can=0, from_can=0)

    # CAN side
    def can_send(self, node, cmd, data):
        self.can.send(struct.pack(CAN_FMT, node << 5 | cmd, len(data), data.ljust(8, b"\0")))
        self.stats["to_can"] += 1

    def can_receive(self):
        while True:
            try:
                raw = self.can.recv(CAN_SIZE, socket.MSG_DONTWAIT)
            except BlockingIOError:
                return
            can_id, dlc, data = struct.unpack(CAN_FMT, raw)
            if can_id & 0x80000000:  # extended id: not CANSimple
                continue
            rtr = bool(can_id & CAN_RTR_FLAG)
            node, cmd = (can_id >> 5) & 0x3F, can_id & 0x1F
            if node not in self.nodes and node != 0x3F:
                continue
            if not rtr and cmd in GET_CMDS | {TXSDO}:
                continue  # a cyclic message or reply from someone else on the bus
            self.cmds.append((node, cmd, data[:dlc], rtr))
            self.stats["from_can"] += 1

    # UART side
    def transact(self, node, cmd, payload=b"", request=False):
        """Send one frame; wait for the answer when one is due. -> payload or None"""
        self.ser.write(frame(node, cmd, payload, request))
        answer_cmd = TXSDO if cmd == RXSDO else cmd
        if node == 0x3F or not (request or cmd == RXSDO):
            return None
        deadline = time.monotonic() + self.a.timeout
        while time.monotonic() < deadline:
            r, _, _ = select.select([self.ser], [], [], max(0.0, deadline - time.monotonic()))
            if r:
                self.rx += self.ser.read(256)
                frames, self.rx = parse(self.rx)
                for n, c, p in frames:
                    if n == node and c == answer_cmd:
                        return p
        self.stats["timeouts"] += 1
        return None

    def run(self):
        last_report = time.monotonic()
        while True:
            self.can_receive()
            if self.cmds:
                node, cmd, payload, rtr = self.cmds.pop(0)
                p = self.transact(node, cmd, payload, rtr)
                if p is not None:
                    self.can_send(node, TXSDO if cmd == RXSDO else cmd, p)
                continue
            self.polls.sort(key=lambda e: e[0])
            due = self.polls[0]
            wait = due[0] - time.monotonic()
            if wait > 0:
                select.select([self.can], [], [], wait)
                continue
            due[0] = max(due[0] + due[1], time.monotonic() - due[1])  # no catch-up bursts after a stall
            node, cmd = due[2], due[3]
            self.stats["polls"] += 1
            p = self.transact(node, cmd, request=True)
            if p is not None:
                self.stats["answers"] += 1
                if not self.alive[node]:
                    print(f"node {node}: answering", flush=True)
                self.alive[node] = time.monotonic()
                self.can_send(node, cmd, p)
            elif self.alive[node] and time.monotonic() - self.alive[node] > 1.0:
                print(f"node {node}: no answer for 1 s", flush=True)
                self.alive[node] = 0.0
            if self.a.verbose and time.monotonic() - last_report > 5:
                last_report = time.monotonic()
                print(" ".join(f"{k}={v}" for k, v in self.stats.items()), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--can", default="vcan0", help="SocketCAN interface (vcan0 or a real can0)")
    ap.add_argument("--nodes", type=int, nargs="+", default=[0], help="CANSimple node ids of the boards")
    ap.add_argument("--timeout", type=float, default=0.02, help="answer timeout, s")
    ap.add_argument("--setup-vcan", action="store_true", help="create the vcan interface (sudo) if missing")
    ap.add_argument("-v", "--verbose", action="store_true", help="print statistics every 5 s")
    for name, (_, hz) in RATES.items():
        ap.add_argument(f"--{name}", type=float, default=hz, metavar="HZ", help=f"poll rate, 0 = off (default {hz})")
    a = ap.parse_args()
    if a.setup_vcan:
        setup_vcan(a.can)
    print(f"bridge {a.port}@{a.baud} <-> {a.can}, nodes {a.nodes}", flush=True)
    try:
        Bridge(a).run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
