#!/usr/bin/env python3
"""MM32 Flasher - a small window around tools/mm32flash.py (flash MM32SPIN05/06 over SWD, pyOCD).

Keep this file in tools/ next to mm32flash.py and mm32cfgstore.py.
Run:  python tools/mm32flash_gui.py     (or packed into an .exe, see build_exe.bat)
"""
import contextlib
import io
import logging
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import mm32flash


class QueueWriter(io.TextIOBase):
    def __init__(self, q):
        self.q = q

    def write(self, s):
        if s:
            self.q.put(s)
        return len(s)

    def flush(self):
        pass


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MM32 Flasher")
        self.geometry("640x520")
        self.minsize(520, 420)
        self.q = queue.Queue()
        self.busy = False
        self.probes = {}

        pad = {"padx": 8, "pady": 4}
        frm = ttk.Frame(self)
        frm.pack(fill="x", **pad)
        frm.columnconfigure(1, weight=1)

        ttk.Label(frm, text="Firmware (.bin):").grid(row=0, column=0, sticky="w")
        self.path = tk.StringVar()
        ttk.Entry(frm, textvariable=self.path).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(frm, text="Browse...", command=self.pick).grid(row=0, column=2)

        ttk.Label(frm, text="Probe:").grid(row=1, column=0, sticky="w")
        self.probe = tk.StringVar(value="auto")
        self.probe_cb = ttk.Combobox(frm, textvariable=self.probe, state="readonly",
                                     values=["auto"])
        self.probe_cb.grid(row=1, column=1, sticky="ew", padx=6)
        ttk.Button(frm, text="Refresh", command=self.refresh_probes).grid(row=1, column=2)

        opt = ttk.LabelFrame(self, text="Options")
        opt.pack(fill="x", **pad)
        self.erase_cfg = tk.BooleanVar(value=False)
        self.backup = tk.BooleanVar(value=True)
        ttk.Checkbutton(opt, text="Back up the 32 KB flash before writing (backup_*.bin next to the firmware)",
                        variable=self.backup).pack(anchor="w", padx=8, pady=2)
        ttk.Checkbutton(opt, text="Also erase the board configuration (--erase-config)",
                        variable=self.erase_cfg).pack(anchor="w", padx=8, pady=2)
        self.under_reset = tk.BooleanVar(value=False)
        ttk.Checkbutton(opt, text="Unlock: connect under reset (nRST must be wired)",
                        variable=self.under_reset).pack(anchor="w", padx=8, pady=2)
        row = ttk.Frame(opt)
        row.pack(anchor="w", padx=8, pady=2)
        ttk.Label(row, text="Start after write:").pack(side="left")
        self.start = tk.StringVar(value="soft")
        ttk.Combobox(row, textvariable=self.start, state="readonly", width=8,
                     values=["soft", "reset", "none"]).pack(side="left", padx=6)
        ttk.Label(row, text="SWD Hz:").pack(side="left", padx=(16, 0))
        self.freq = tk.StringVar(value="1000000")
        ttk.Entry(row, textvariable=self.freq, width=10).pack(side="left", padx=6)

        bar = ttk.Frame(self)
        bar.pack(pady=6)
        self.btn = ttk.Button(bar, text="Flash", command=self.go)
        self.btn.pack(side="left", padx=6)
        self.btn_unlock = ttk.Button(bar, text="Unlock chip (full chip erase)", command=self.unlock)
        self.btn_unlock.pack(side="left", padx=6)

        self.log = tk.Text(self, height=14, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True, **pad)

        self.after(100, self.pump)
        self.after(300, self.refresh_probes)
        self.say("Pick a .bin, connect SWD (GND, SWDIO, SWCLK) and click Flash.\n")

    def say(self, s):
        self.log.configure(state="normal")
        self.log.insert("end", s)
        self.log.see("end")
        self.log.configure(state="disabled")

    def pump(self):
        try:
            while True:
                item = self.q.get_nowait()
                if item is None:
                    self.done()
                else:
                    self.say(item)
        except queue.Empty:
            pass
        self.after(100, self.pump)

    def pick(self):
        p = filedialog.askopenfilename(title="Firmware",
                                       filetypes=[("Binary", "*.bin"), ("All files", "*.*")])
        if p:
            self.path.set(p)

    def refresh_probes(self):
        try:
            from pyocd.core.helpers import ConnectHelper
            found = ConnectHelper.get_all_connected_probes(blocking=False, print_wait_message=False)
        except Exception as e:
            self.say("Could not list probes: %s\n" % e)
            return
        self.probes = {"%s [%s]" % (p.description, p.unique_id): p.unique_id for p in found}
        self.probe_cb.configure(values=["auto"] + list(self.probes))
        if self.probe.get() not in self.probes:
            self.probe.set("auto")
        self.say("Probes found: %d\n" % len(self.probes))

    def go(self):
        if self.busy:
            return
        fw = self.path.get().strip()
        if not os.path.isfile(fw):
            messagebox.showerror("MM32 Flasher", "Pick an existing .bin file.")
            return
        try:
            freq = int(self.freq.get())
        except ValueError:
            messagebox.showerror("MM32 Flasher", "SWD frequency must be a number.")
            return
        argv = ["mm32flash", fw, "--start", self.start.get(), "--frequency", str(freq)]
        if self.erase_cfg.get():
            if not messagebox.askyesno("MM32 Flasher", "Really erase the board configuration too?"):
                return
            argv.append("--erase-config")
        if self.backup.get():
            bk = os.path.join(os.path.dirname(fw), "backup_%s.bin" % time.strftime("%Y%m%d_%H%M%S"))
            argv += ["--backup", bk]
        uid = self.probes.get(self.probe.get())
        if uid:
            argv += ["--probe", uid]
        def job():
            old_argv = sys.argv
            sys.argv = argv
            try:
                mm32flash.main()
            finally:
                sys.argv = old_argv

        self.start_job(job)

    def unlock(self):
        """Full chip erase of a read-protected chip: the same as the installation guide's
        `pyocd erase -t mm32spin05pf --chip` (the chip definition comes from a CMSIS pack that
        is downloaded once, internet needed the first time). Erases everything, config included."""
        if self.busy:
            return
        try:
            freq = int(self.freq.get())
        except ValueError:
            messagebox.showerror("MM32 Flasher", "SWD frequency must be a number.")
            return
        if not messagebox.askyesno(
                "MM32 Flasher",
                "Full chip erase wipes the WHOLE flash, including the original firmware and the board configuration.\n"
                "The original firmware of a locked chip cannot be backed up.\n\nContinue?"):
            return
        uid = self.probes.get(self.probe.get())
        under_reset = self.under_reset.get()

        def job():
            from pyocd.__main__ import PyOCDTool
            target = "mm32spin05pf"

            def run(args):
                print(">> pyocd " + " ".join(args))
                rc = PyOCDTool().run(args)
                if rc not in (None, 0):
                    raise SystemExit("pyocd %s exited with code %s" % (args[0], rc))

            listed = io.StringIO()
            with contextlib.redirect_stdout(listed):
                PyOCDTool().run(["list", "--targets"])
            if target not in listed.getvalue().lower():
                print("Downloading the chip definition %s (internet needed)..." % target)
                run(["pack", "update"])
                run(["pack", "install", target])
            args = ["erase", "-t", target, "--chip", "-f", str(freq)]
            if uid:
                args += ["-u", uid]
            if under_reset:
                args += ["-M", "under-reset"]
            run(args)
            print("Chip erased. You can now flash it with the Flash button.")

        self.start_job(job)

    def start_job(self, job):
        self.busy = True
        self.btn.configure(state="disabled")
        self.btn_unlock.configure(state="disabled")
        self.say("\n--- start ---\n")
        threading.Thread(target=self.work, args=(job,), daemon=True).start()

    def work(self, job):
        w = QueueWriter(self.q)
        handler = logging.StreamHandler(w)
        root = logging.getLogger()
        root.addHandler(handler)
        root.setLevel(logging.WARNING)
        ok = False
        try:
            with contextlib.redirect_stdout(w), contextlib.redirect_stderr(w):
                try:
                    job()
                    ok = True
                except SystemExit as e:
                    if e.code in (None, 0):
                        ok = True
                    else:
                        self.q.put("ERROR: %s\n" % e.code)
                except Exception as e:
                    self.q.put("ERROR: %s: %s\n" % (type(e).__name__, e))
        finally:
            root.removeHandler(handler)
        self.q.put("--- DONE ---\n" if ok else "--- FAILED ---\n")
        self.q.put(None)                                    # sentinel: the job is over

    def done(self):
        self.busy = False
        self.btn.configure(state="normal")
        self.btn_unlock.configure(state="normal")


if __name__ == "__main__":
    App().mainloop()
