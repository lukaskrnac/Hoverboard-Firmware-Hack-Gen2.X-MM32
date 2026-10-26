#!/usr/bin/env python3
"""Read / change the board configuration (pinstorage[64]) of an MM32SPIN05 hoverboard over SWD.

    python3 tools/mm32config.py show
    python3 tools/mm32config.py set itotal=PA7 softocp=10        # appends a new config record
    python3 tools/mm32config.py set slaveid=2 --restart          # and start the firmware again

The record format and names are the ones PinFinder uses (its 'addresses'). Records are 128 bytes in
16 slots at 0x08007800; the active one is the slot before the first blank slot, and a new one is
written exactly like sim_eeprom.c does (erasing the other 1 KB page when one fills up).
The firmware reads the configuration only at boot: use --restart (soft start, board stays powered)
or power cycle.
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mm32flash import Mm32, PAGE                                     # noqa: E402
from pyocd.core.helpers import ConnectHelper                          # noqa: E402

CONFIG, SLOT, SLOTS = 0x08007800, 128, 16
MAGIC = 0xDCAB
PINS = ("PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PA11 PA12 PA13 PA14 PA15 PB0 PB1 PB2 PB3 PB4 PB5 PB6 PB7 "
        "PB8 PB9 PB10 PB11 PB12 PC13 PC14 PC15 PD0 PD1 PD2 PD3").split()       # hardware.c, MM32SPIN05
# index: (name, kind, meaning); kind 'pin' shows/accepts pin names, 65535 = not used
FIELDS = {
    0: ("halla", "pin", "hall sensor A"), 1: ("hallb", "pin", "hall sensor B"), 2: ("hallc", "pin", "hall sensor C"),
    3: ("ledr", "pin", "red LED"), 4: ("ledg", "pin", "green LED"), 5: ("ledb", "pin", "blue / orange LED"),
    6: ("ledu", "pin", "upper LED"), 7: ("ledd", "pin", "lower LED"), 8: ("buzzer", "pin", "buzzer"),
    9: ("button", "pin", "power button sense"), 10: ("latch", "pin", "power self-hold"), 11: ("charge", "pin", "charger detect"),
    12: ("vbat", "pin", "battery voltage adc"), 13: ("itotal", "pin", "bus current adc"),
    14: ("tx", "pin", "uart tx"), 15: ("rx", "pin", "uart rx"), 16: ("iphasea", "pin", "phase A current adc"),
    17: ("iphaseb", "pin", "phase B current adc"), 23: ("ocp", "pin", "TIM1 break input (hardware overcurrent)"),
    24: ("ocpref", "pin", "overcurrent comparator reference"),
    32: ("magicnum", "num", "0xDCAB = valid"), 33: ("vbatdivider", "num", "vbat = divider * Vadc"),
    34: ("itotaldivider", "num", "itotal = divider * Vadc (1/shunt ohm)"), 36: ("baud", "num", "uart baud"),
    37: ("pwmres", "num", "TIM1 period"), 38: ("slaveid", "num", "RemoteUartBus slave id"),
    39: ("windings", "num", "motor poles"), 40: ("invlowside", "num", "1 = low side gate drive inverted"),
    41: ("softocp", "num", "A, averaged bus current limit, 0 = off"), 42: ("hardocp", "num", "adc watchdog current limit, adc counts above zero"),
    44: ("drivemode", "num", "0 COM_VOLT, 1 COM_SPEED, 2 SINE_VOLT, 3 SINE_SPEED"),
    45: ("batfull", "num", "mV"), 46: ("batempty", "num", "mV, <= 20000 disables the low battery cut-off"),
    47: ("timeout", "num", "ms without a command until speed = 0"),
    48: ("speedkp", "num", "COM_SPEED: 0.1 permille duty per rpm, 0 = default 50"),
    49: ("speedki", "num", "COM_SPEED: 0.1 permille duty per rpm*s, 0 = default 100"),
    50: ("speedkv", "num", "COM_SPEED feed forward: 0.1 rpm/V at 100% duty, 0 = default 395"),
    51: ("speedd0", "num", "COM_SPEED feed forward: dead band permille duty, 0 = default 185"),
    52: ("sinekp", "num", "SINE_SPEED: 0.1 permille modulation per rpm, 0 = default 5"),
    53: ("sineki", "num", "SINE_SPEED: 0.1 permille per rpm*s, 0 = default 85"),
    54: ("sinekv", "num", "SINE_SPEED feed forward: 0.1 rpm/V at 100% modulation, 0 = default 242"),
    55: ("sined0", "num", "SINE_SPEED feed forward: dead band permille, 0 = default 10"),
}
BY_NAME = {v[0]: k for k, v in FIELDS.items()}


def read_page(t):
    return bytes(t.read_memory_block8(CONFIG, SLOT * SLOTS))


def active_slot(page):
    blank = [page[i * SLOT:(i + 1) * SLOT] == b"\xff" * SLOT for i in range(SLOTS)]
    if all(blank):
        return None, 0
    first_blank = blank.index(True) if True in blank else None
    if first_blank is None:
        return SLOTS - 1, None
    return (first_blank - 1) % SLOTS, first_blank


def show(words):
    for i, (name, kind, meaning) in sorted(FIELDS.items()):
        v = words[i]
        if kind == "pin":
            txt = "-" if v == 0xFFFF else (PINS[v] if v < len(PINS) else "invalid %d" % v)
        else:
            txt = "0x%04X" % v if i == 32 else str(v)
        print("  [%2d] %-14s %-8s %s" % (i, name, txt, meaning))


def parse_value(name, text):
    i = BY_NAME[name]
    if FIELDS[i][1] == "pin":
        if text in ("-", "none", "65535"):
            return i, 0xFFFF
        if text.upper() not in PINS:
            raise SystemExit("%s: unknown pin %s" % (name, text))
        return i, PINS.index(text.upper())
    v = int(text, 0)
    if not 0 <= v <= 0xFFFF:
        raise SystemExit("%s: %d out of range" % (name, v))
    return i, v


def write_slot(mcu, page, words):
    """Append a record the way FLASH_SIM_WriteEE() does."""
    _, slot = active_slot(page)
    if slot is None:                                     # no blank slot (sim_eeprom.c never leaves this)
        mcu.erase(CONFIG, CONFIG + 2 * PAGE)
        slot = 0
    if slot == SLOTS // 2 - 1:                           # last slot of the first page: erase the second
        mcu.erase(CONFIG + PAGE, CONFIG + 2 * PAGE)
    elif slot == SLOTS - 1:                              # last slot of the second page: erase the first
        mcu.erase(CONFIG, CONFIG + PAGE)
    mcu.program_at(CONFIG + slot * SLOT, struct.pack("<64H", *words))
    back = struct.unpack("<64H", bytes(mcu.t.read_memory_block8(CONFIG + slot * SLOT, SLOT)))
    if list(back) != list(words):
        raise SystemExit("verify of slot %d FAILED" % slot)
    return slot


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=("show", "set"))
    ap.add_argument("assign", nargs="*", help="name=value, e.g. itotal=PA7 softocp=10")
    ap.add_argument("--restart", action="store_true", help="soft start the firmware after writing")
    a = ap.parse_args()

    changes = {}
    for kv in a.assign:
        name, _, val = kv.partition("=")
        if name not in BY_NAME:
            raise SystemExit("unknown field %s, known: %s" % (name, " ".join(sorted(BY_NAME))))
        i, v = parse_value(name, val)
        changes[i] = v

    session = ConnectHelper.session_with_chosen_probe(
        target_override="cortex_m", options={"connect_mode": "attach", "resume_on_disconnect": True})
    session.open()
    mcu = Mm32(session.target)
    try:
        page = read_page(mcu.t)
        slot, _ = active_slot(page)
        if slot is None:
            raise SystemExit("no configuration stored (run PinFinder first)")
        words = list(struct.unpack("<64H", page[slot * SLOT:(slot + 1) * SLOT]))
        if words[32] != MAGIC:
            print("warning: active record (slot %d) has no valid magic number" % slot)
        if a.cmd == "show":
            print("active record: slot %d" % slot)
            show(words)
            return
        if not changes:
            raise SystemExit("nothing to set")
        new = words[:]
        for i, v in changes.items():
            print("  %-14s %s -> %s" % (FIELDS[i][0], words[i], v))
            new[i] = v
        if new == words:
            print("no change")
            return
        mcu.halt()
        slot = write_slot(mcu, page, new)
        print("written to slot %d" % slot)
        if a.restart:
            mcu.soft_start()
            print("firmware restarted")
        else:
            mcu.t.resume()
            print("takes effect at the next start (--restart or power cycle)")
    finally:
        try:
            session.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
