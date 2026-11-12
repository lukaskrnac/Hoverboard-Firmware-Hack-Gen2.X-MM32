#!/usr/bin/env python3
"""Talk to PinFinder in agent mode (non-interactive, for scripts and agents).

    python3 tools/pinfinder.py info
    python3 tools/pinfinder.py detect hall
    python3 tools/pinfinder.py --json list
    python3 tools/pinfinder.py "set slaveid 2" save          # several commands in a row

Each command is sent as "@<command>"; PinFinder echoes "> <command>", then prints "key=value" lines,
"# notes", "warn ..." and "progress n/m" lines and ends with "ok" or "err <text>". Exit status 1 if a
command ended with err. Needs pyserial. PinFinder talks 115200 8N1 on the configured TX/RX pins.
"""
import argparse
import json
import sys
import time

import serial


class PinFinder:
    def __init__(self, port, baud=115200, verbose=False):
        self.s = serial.Serial(port, baud, timeout=0.1)
        self.verbose = verbose

    def close(self):
        self.s.close()

    def _line(self, deadline):
        buf = b""
        while time.time() < deadline:
            b = self.s.read(1)
            if not b:
                continue
            if b == b"\n":
                return buf.decode(errors="replace").rstrip("\r")
            buf += b
        raise TimeoutError("no answer from PinFinder")

    def cmd(self, command, timeout=60.0):
        """Runs one command. Returns (ok, results, lines): results maps key -> value (repeated keys -> list)."""
        self.s.reset_input_buffer()
        self.s.write(b"@" + command.encode() + b"\n")
        deadline = time.time() + timeout
        while True:    # skip the banner and anything before our echo
            ln = self._line(deadline)
            if self.verbose:
                print(ln, file=sys.stderr)
            if ln.startswith("> ") and ln[2:].split() == command.split():
                break
        results, lines = {}, []
        while True:
            ln = self._line(deadline)
            if self.verbose:
                print(ln, file=sys.stderr)
            lines.append(ln)
            if ln == "ok" or ln.startswith("err"):
                return ln == "ok", results, lines
            if "=" in ln and not ln.startswith(("#", "warn", "progress")):
                k, v = ln.split("=", 1)
                if k in results:
                    results[k] = (results[k] if isinstance(results[k], list) else [results[k]]) + [v]
                else:
                    results[k] = v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("commands", nargs="+", help="PinFinder commands (quote commands with arguments)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=90.0, help="seconds per command")
    ap.add_argument("--json", action="store_true", help="one JSON object per command")
    ap.add_argument("-v", "--verbose", action="store_true", help="raw lines to stderr")
    a = ap.parse_args()
    pf = PinFinder(a.port, a.baud, a.verbose)
    failed = False
    for c in a.commands:
        good, res, lines = pf.cmd(c, a.timeout)
        failed |= not good
        if a.json:
            print(json.dumps({"command": c, "ok": good, "results": res, "lines": lines}))
        else:
            for ln in lines:
                print(ln)
    pf.close()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
