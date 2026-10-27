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
from cfgfields import PINS, FIELDS, BY_NAME                          # noqa: E402
from mm32flash import Mm32, PAGE                                     # noqa: E402
from pyocd.core.helpers import ConnectHelper                          # noqa: E402

CONFIG, SLOT, SLOTS = 0x08007800, 128, 16
MAGIC = 0xDCAB
EXT_MAGIC = 0xDCAD    # extension record (words 64-127, magic at its word 32 = index 96): the slot before the main one


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


def records(page):
    """(slot, 128 words): the active main record and the extension record before it (or defaults)"""
    slot, _ = active_slot(page)
    if slot is None:
        return None, None
    base = list(struct.unpack("<64H", page[slot * SLOT:(slot + 1) * SLOT]))
    p = (slot - 1) % SLOTS
    ext = list(struct.unpack("<64H", page[p * SLOT:(p + 1) * SLOT]))
    if ext[32] != EXT_MAGIC:
        ext = [0xFFFF] * 64; ext[32] = EXT_MAGIC
    return slot, base + ext


def show(words):
    for i, (name, kind, meaning) in sorted(FIELDS.items()):
        v = words[i]
        if kind == "pin":
            txt = "-" if v == 0xFFFF else (PINS[v] if v < len(PINS) else "invalid %d" % v)
        else:
            txt = "0x%04X" % v if i in (32, 96) else str(v)
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
    ap.add_argument("--probe", default=os.environ.get("MM32_PROBE"), metavar="UID",
                    help="probe unique id when more than one is connected (or MM32_PROBE); pyocd list shows them")
    a = ap.parse_args()

    changes = {}
    for kv in a.assign:
        name, _, val = kv.partition("=")
        if name not in BY_NAME:
            raise SystemExit("unknown field %s, known: %s" % (name, " ".join(sorted(BY_NAME))))
        i, v = parse_value(name, val)
        changes[i] = v

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=a.probe, target_override="cortex_m", options={"connect_mode": "attach", "resume_on_disconnect": True})
    session.open()
    mcu = Mm32(session.target)
    try:
        page = read_page(mcu.t)
        slot, words = records(page)
        if slot is None:
            raise SystemExit("no configuration stored (run PinFinder first)")
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
        write_slot(mcu, page, new[64:])                      # the extension first, as the firmware does
        slot = write_slot(mcu, read_page(mcu.t), new[:64])
        print("written to slots %d (extension) and %d" % ((slot - 1) % SLOTS, slot))
        if a.restart:
            mcu.soft_start()
            print("firmware restarted")
        else:
            mcu.resume()
            print("takes effect at the next start (--restart or power cycle)")
    finally:
        try:
            session.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
