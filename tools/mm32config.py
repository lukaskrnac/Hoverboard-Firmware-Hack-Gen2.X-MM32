#!/usr/bin/env python3
"""Read / change the board configuration (pinstorage[64]) of an MM32SPIN05 hoverboard over SWD.

    python3 tools/mm32config.py show
    python3 tools/mm32config.py set itotal=PA7 softocp=10        # appends a new config record
    python3 tools/mm32config.py set slaveid=2 --restart          # and start the firmware again

The record format and names are the ones PinFinder uses (its 'addresses'). The storage (one 1 KB page
at 0x08007C00, a save = extension + main record) is in mm32cfgstore.py, written as sim_eeprom.c does.
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

import mm32cfgstore                                                   # noqa: E402
MAGIC = mm32cfgstore.MAGIC


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
        words, layout = mm32cfgstore.read(mcu.t)
        if words is None:
            raise SystemExit("no configuration stored (run PinFinder first)")
        if words[32] != MAGIC:
            print("warning: the newest record has no valid magic number")
        if a.cmd == "show":
            print("layout: %s" % ("1 KB page at 0x08007C00" if layout == "new" else "old (2 KB at 0x08007800): flash a current image to move it"))
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
        if layout != "new":
            raise SystemExit("the board has the old config layout: flash a current image first (mm32flash.py moves it)")
        mcu.halt()
        slot = mm32cfgstore.write(mcu, new)                  # the extension first, as the firmware does
        print("written to slots %d (extension) and %d" % (slot - 1, slot))
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
