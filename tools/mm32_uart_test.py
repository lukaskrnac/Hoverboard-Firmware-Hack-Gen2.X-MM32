"""Jednoduchý PC tester MM32 cez CANSimple over UART."""
import math
import struct
import sys
import time
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

from cansimple_uart import AXIS_STATE, CONTROL_MODE, INPUT_MODE, Bus, load_endpoints

IDLE = AXIS_STATE["IDLE"]
ARMED = AXIS_STATE["CLOSED_LOOP_CONTROL"]
VELOCITY = CONTROL_MODE["VELOCITY_CONTROL"]
RAMP = INPUT_MODE["VEL_RAMP"]
MAX_RPM = 300
COMM_ERRORS = (OSError, ValueError, struct.error)


class Tester(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MM32 UART tester")
        self.resizable(False, False)
        self.bus = None
        self.armed = False
        self.arm_pending = False
        self.target = 0.0
        self.next_poll = 0.0
        self.port = tk.StringVar()
        self.baud = tk.StringVar(value="115200")
        self.node = tk.StringVar(value="1")
        self.rpm = tk.StringVar(value="30")
        self.status = tk.StringVar(value="Vyber COM port a klikni na Pripojiť.")
        self.telemetry = tk.StringVar(value="Otáčky: —   |   Batéria: —   |   Prúd batérie: —")
        self.board_state = tk.StringVar(value="Stav dosky: —")
        self.command = tk.StringVar(value="Povel: —")

        frame = ttk.Frame(self, padding=12)
        frame.pack(fill="both")
        self.port_box = ttk.Combobox(frame, textvariable=self.port, state="readonly", width=11)
        ttk.Label(frame, text="COM port").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, text="Baud").grid(row=0, column=1, sticky="w")
        ttk.Label(frame, text="ID dosky").grid(row=0, column=2, sticky="w")
        self.port_box.grid(row=1, column=0, sticky="ew")
        baud_box = ttk.Entry(frame, textvariable=self.baud, width=10)
        baud_box.grid(row=1, column=1, sticky="ew", padx=4)
        node_box = ttk.Entry(frame, textvariable=self.node, width=6)
        node_box.grid(row=1, column=2, sticky="ew")
        refresh = ttk.Button(frame, text="Obnoviť", command=self.refresh)
        refresh.grid(row=1, column=3, padx=4)
        self.connection_inputs = (self.port_box, baud_box, node_box, refresh)
        self.connect_button = ttk.Button(frame, text="Pripojiť", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=4, sticky="ew")

        ttk.Label(frame, textvariable=self.telemetry).grid(row=2, columnspan=5, sticky="w", pady=(14, 3))
        ttk.Label(frame, textvariable=self.board_state, wraplength=470).grid(row=3, columnspan=5, sticky="w")
        ttk.Separator(frame).grid(row=4, columnspan=5, sticky="ew", pady=8)

        self.arm_button = ttk.Button(frame, text="Povoliť pohon", command=self.arm)
        self.arm_button.grid(row=5, columnspan=2, sticky="ew")
        ttk.Label(frame, text="Cieľ [ot./min]").grid(row=5, column=2, padx=4)
        ttk.Entry(frame, textvariable=self.rpm, width=9).grid(row=5, column=3)
        self.send_button = ttk.Button(frame, text="Nastaviť", command=self.set_speed)
        self.send_button.grid(row=5, column=4, sticky="ew")
        self.zero_button = ttk.Button(frame, text="Brzdiť na 0", command=self.zero)
        self.zero_button.grid(row=6, columnspan=2, sticky="ew", pady=5)
        self.stop_button = ttk.Button(frame, text="STOP / IDLE", command=self.stop)
        self.stop_button.grid(row=6, column=2, columnspan=2, sticky="ew", padx=4)
        self.clear_button = ttk.Button(frame, text="Vymazať chyby", command=self.clear_errors)
        self.clear_button.grid(row=6, column=4, sticky="ew")
        ttk.Label(frame, textvariable=self.command).grid(row=7, columnspan=5, sticky="w")
        ttk.Label(frame, textvariable=self.status, wraplength=470, justify="left").grid(
            row=8, columnspan=5, sticky="w", pady=(8, 0)
        )

        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.bind("<Escape>", lambda event: self.stop() if self.bus else None)
        self.refresh()
        self.update_buttons()
        self.after(100, self.tick)

    def refresh(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_box.configure(values=ports)
        if self.port.get() not in ports:
            self.port.set(ports[0] if ports else "")

    def update_buttons(self):
        connected = self.bus is not None
        for widget in self.connection_inputs:
            widget.state(["disabled"] if connected else ["!disabled"])
        for widget, enabled in (
            (self.arm_button, connected and not self.armed),
            (self.send_button, self.armed and not self.arm_pending),
            (self.zero_button, self.armed and not self.arm_pending),
            (self.stop_button, connected),
            (self.clear_button, connected and not self.armed),
        ):
            widget.state(["!disabled"] if enabled else ["disabled"])
        self.connect_button.configure(text="Odpojiť" if connected else "Pripojiť")
        self.command.set(f"Povel: {self.target:g} ot./min" if self.armed else "Povel: —")

    def toggle_connection(self):
        if self.bus:
            self.disconnect("Odpojené.")
            return
        try:
            node, baud = int(self.node.get()), int(self.baud.get())
            if not self.port.get() or not 0 <= node <= 62 or baud <= 0:
                raise ValueError("Zadaj COM port, baud > 0 a ID dosky 0 až 62.")
            self.bus = Bus(self.port.get(), baud)
            self.bus.s.write_timeout = 0.05
            self.bus.node = node
            version = self.bus.get_version(node)
            if not version or version["protocol"] != 2:
                raise ValueError("Doska neodpovedala protokolom CANSimple v2. Skontroluj ID, baud a vodiče.")
            self.read_state()
            self.next_poll = 0.0
            self.status.set(f"Pripojené. Firmware {version['fw']}. Pohon povoľ ručne.")
        except COMM_ERRORS + (IndexError,) as exc:
            self.disconnect(str(exc))
        self.update_buttons()

    def read_state(self):
        state = self.bus.heartbeat(self.bus.node)
        if state is None:
            raise OSError("Doska neodpovedá.")
        label = {IDLE: "IDLE", ARMED: "povolený"}.get(state["state"], str(state["state"]))
        errors = ", ".join(state["error_names"])
        if state["errors"]:
            label += f" · 0x{state['errors']:08X}" + (f" ({errors})" if errors else "")
        self.board_state.set("Stav dosky: " + label)
        return state

    def arm(self):
        bus = self.bus
        try:
            state = self.read_state()
            if state["state"] != IDLE or state["errors"]:
                self.status.set("Povolenie vyžaduje stav IDLE bez chýb.")
                return
            bus.set_controller_mode(bus.node, VELOCITY, RAMP)
            if (bus.read("axis0.controller.config.control_mode") != VELOCITY
                    or bus.read("axis0.controller.config.input_mode") != RAMP):
                raise OSError("Doska nepotvrdila režim regulácie otáčok.")
            bus.set_input_vel(bus.node, 0)
            self.target = 0.0
            self.armed = True
            self.arm_pending = True
            bus.set_axis_state(bus.node, ARMED)
            # Nasledujúci heartbeat potvrdí účinok povelu po regulačnom cykle dosky.
            self.next_poll = time.monotonic() + 0.05
            self.status.set("Odoslané povolenie pohonu s cieľom 0 ot./min.")
        except COMM_ERRORS as exc:
            self.disconnect(str(exc))
        self.update_buttons()

    def set_speed(self):
        try:
            target = float(self.rpm.get().replace(",", "."))
            if not math.isfinite(target) or abs(target) > MAX_RPM:
                raise ValueError()
        except ValueError:
            self.status.set(f"Zadaj otáčky od −{MAX_RPM} do +{MAX_RPM}.")
            return
        self.target = target
        self.update_buttons()

    def zero(self):
        self.rpm.set("0")
        self.set_speed()

    def send_idle(self):
        self.armed = False
        self.arm_pending = False
        self.target = 0.0
        self.bus.set_axis_state(self.bus.node, IDLE)
        self.bus.set_input_vel(self.bus.node, 0)

    def stop(self):
        try:
            self.send_idle()
            self.status.set("Odoslané STOP / IDLE. Na ďalší rozbeh povoľ pohon znova.")
        except OSError as exc:
            self.disconnect(f"STOP sa nepodarilo odoslať: {exc}")
        self.update_buttons()

    def clear_errors(self):
        try:
            if self.read_state()["state"] != IDLE:
                self.status.set("Chyby môžeš vymazať iba v IDLE.")
                return
            self.bus.clear_errors(self.bus.node)
            self.next_poll = 0.0
            self.status.set("Odoslaná požiadavka na vymazanie chýb.")
        except COMM_ERRORS as exc:
            self.disconnect(str(exc))

    def tick(self):
        bus = self.bus
        if bus:
            try:
                now = time.monotonic()
                if now >= self.next_poll:
                    state = self.read_state()
                    if self.armed and (state["state"] != ARMED or state["errors"]):
                        self.stop()
                        if self.bus:
                            self.status.set("Doska hlási chybu alebo nepovolila pohon. Poslané IDLE.")
                    elif self.arm_pending:
                        self.arm_pending = False
                        self.status.set("Pohon povolený. Zadaj skúšobné otáčky.")
                        self.update_buttons()
                    if self.bus:
                        enc = bus.get_encoder_estimates(bus.node)
                        power = bus.get_bus_voltage_current(bus.node)
                        rpm = "—" if enc is None else f"{enc[1] * 60:.1f} ot./min"
                        volts = "—" if power is None else f"{power[0]:.2f} V"
                        amps = "—" if power is None else f"{power[1]:.2f} A"
                        self.telemetry.set(f"Otáčky: {rpm}   |   Batéria: {volts}   |   Prúd batérie: {amps}")
                    self.next_poll = now + 0.4
                if self.armed:
                    bus.set_input_vel(bus.node, self.target / 60)
            except COMM_ERRORS as exc:
                self.disconnect(str(exc))
        self.after(100, self.tick)

    def disconnect(self, reason):
        if self.bus:
            if self.armed:
                try:
                    self.send_idle()
                except OSError:
                    reason += " STOP sa nepodarilo odoslať."
            self.bus.s.close()
        self.bus = None
        self.armed = False
        self.arm_pending = False
        self.target = 0.0
        self.board_state.set("Stav dosky: —")
        self.telemetry.set("Otáčky: —   |   Batéria: —   |   Prúd batérie: —")
        self.status.set(reason)
        self.update_buttons()

    def close_window(self):
        self.disconnect("Odpojené.")
        self.destroy()


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        try:
            endpoints = load_endpoints()["endpoints"]
            if "axis0.controller.config.control_mode" not in endpoints:
                raise ValueError("Missing velocity-control endpoint")
            app = Tester()
            app.withdraw()
            app.update_idletasks()
            app.destroy()
        except Exception:
            raise SystemExit(1)
    else:
        Tester().mainloop()
