#!/usr/bin/env python3
"""Honda datalog GUI simulator (Linux-focused).

Visual simulator with real-time sliders for:
- RPM, TPS, MAP, AFR, IAT, ECT

Protocol compatibility:
- 0x10 -> 0xCD (HTS handshake)
- 0x20 -> 52-byte packet
"""

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass
import sys

if os.name != "nt":
    import pty
    import select
else:
    pty = None
    select = None

try:
    import tkinter as tk
    from tkinter import ttk
except ModuleNotFoundError as e:
    print("Missing dependency: tkinter", file=sys.stderr)
    print("Install it on Ubuntu/Debian with:", file=sys.stderr)
    print("  sudo apt-get update && sudo apt-get install -y python3-tk", file=sys.stderr)
    raise SystemExit(2) from e

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except Exception:
    serial = None
    list_ports = None

PACKET_SIZE = 52


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def temp_c_to_raw(target_c: float) -> int:
    # Inverse fit compatible with docs/main.rs conversion.
    def raw_to_c(raw: int) -> float:
        x = raw / 51.0
        f = (
            (0.1423 * x**6)
            - (2.4938 * x**5)
            + (17.837 * x**4)
            - (68.698 * x**3)
            + (154.69 * x**2)
            - (232.75 * x)
            + 284.24
        )
        return (f - 32.0) * 5.0 / 9.0

    best_raw = 0
    best_err = 1e9
    for raw in range(256):
        err = abs(raw_to_c(raw) - target_c)
        if err < best_err:
            best_err = err
            best_raw = raw
    return best_raw


def mbar_to_raw(map_mbar: float) -> int:
    volts = (map_mbar + 29.9) / 365.9
    raw = round(volts * 255.0 / 5.0)
    return int(clamp(raw, 0, 255))


def tps_to_raw(tps_pct: float) -> int:
    raw = round(tps_pct * 2.04 + 25.0)
    return int(clamp(raw, 0, 255))


def afr_to_wb_raw(afr: float) -> int:
    lam = afr / 14.7
    wb_volt = (lam - 0.71) * (5.0 / (1.3 - 0.71))
    raw = round(clamp(wb_volt, 0.0, 5.0) * 255.0 / 5.0)
    return int(clamp(raw, 0, 255))


def rpm_to_div(rpm: float) -> int:
    if rpm <= 1.0:
        return 65535
    div = round(1_851_562.0 / rpm)
    return int(clamp(div, 1, 65535))


def ign_deg_to_raw(deg: float) -> int:
    raw = round((deg + 6.0) / 0.25)
    return int(clamp(raw, 0, 255))


def battery_v_to_raw(v: float) -> int:
    raw = round(v * 270.0 / 26.0)
    return int(clamp(raw, 0, 255))


def iacv_duty_to_raw(duty: float) -> int:
    raw = round((duty + 100.0) * 327.68)
    return int(clamp(raw, 0, 65535))


def fract(x: float) -> float:
    return x - math.floor(x)


def hash01(n: float) -> float:
    return fract(math.sin(n * 12.9898 + 78.233) * 43758.5453)


def smoothstep01(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def smooth_noise(now_s: float, hz: float, seed: float) -> float:
    x = now_s * hz + seed * 19.19
    i = math.floor(x)
    f = x - i
    a = hash01(i + seed * 101.0)
    b = hash01(i + 1.0 + seed * 101.0)
    t = smoothstep01(f)
    return ((a + (b - a) * t) * 2.0) - 1.0


@dataclass
class SimValues:
    rpm: float = 1200.0
    tps: float = 8.0
    map_mbar: float = 450.0
    afr: float = 14.5
    iat_c: float = 30.0
    ect_c: float = 85.0
    batt_v: float = 13.8
    vtec: bool = False
    auto_profile: bool = False
    auto_accel_sec: float = 10.0
    auto_afr_start: float = 11.0
    auto_afr_end: float = 16.0
    auto_map_start: float = 320.0
    auto_map_end: float = 1650.0
    auto_manual_mix: float = 0.35
    auto_variation: bool = True
    variation_level: float = 0.5


class SharedState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.values = SimValues()

    def set_values(self, v: SimValues) -> None:
        with self._lock:
            self.values = v

    def snapshot(self) -> SimValues:
        with self._lock:
            return SimValues(**self.values.__dict__)


def with_variation(v: SimValues, now_s: float) -> SimValues:
    if not v.auto_variation:
        return v

    k = clamp(v.variation_level, 0.0, 1.0)
    out = SimValues(**v.__dict__)

    out.rpm = clamp(
        out.rpm
        + (40.0 + 220.0 * k) * smooth_noise(now_s, 0.35, 11.0)
        + (15.0 + 95.0 * k) * smooth_noise(now_s, 1.6, 12.0),
        800.0,
        9200.0,
    )
    out.tps = clamp(
        out.tps
        + (0.6 + 3.5 * k) * smooth_noise(now_s, 0.5, 21.0)
        + (0.2 + 1.4 * k) * smooth_noise(now_s, 2.2, 22.0),
        0.0,
        100.0,
    )
    out.map_mbar = clamp(
        out.map_mbar
        + (8.0 + 95.0 * k) * smooth_noise(now_s, 0.45, 31.0)
        + (2.0 + 35.0 * k) * smooth_noise(now_s, 1.9, 32.0)
        + (out.tps - 50.0) * 0.15,
        -70.0,
        1790.0,
    )
    out.afr = clamp(
        out.afr
        + (0.02 + 0.35 * k) * smooth_noise(now_s, 0.4, 41.0)
        + (0.01 + 0.12 * k) * smooth_noise(now_s, 1.7, 42.0),
        10.0,
        18.0,
    )

    # Transient mini-events to avoid repetitive traces.
    lift_evt = clamp((smooth_noise(now_s, 0.13, 90.0) - 0.78) / 0.22, 0.0, 1.0) * k
    out.tps = clamp(out.tps - 10.0 * lift_evt, 0.0, 100.0)
    out.map_mbar = clamp(out.map_mbar - 120.0 * lift_evt, -70.0, 1790.0)
    out.afr = clamp(out.afr + 0.5 * lift_evt, 10.0, 18.0)

    enr_evt = clamp((smooth_noise(now_s, 0.17, 95.0) - 0.80) / 0.20, 0.0, 1.0) * k
    out.map_mbar = clamp(out.map_mbar + 90.0 * enr_evt, -70.0, 1790.0)
    out.afr = clamp(out.afr - 0.45 * enr_evt, 10.0, 18.0)

    out.iat_c = clamp(out.iat_c + 1.5 * k * smooth_noise(now_s, 0.1, 51.0), -20.0, 90.0)
    out.ect_c = clamp(out.ect_c + 1.0 * k * smooth_noise(now_s, 0.07, 52.0), 40.0, 120.0)
    out.batt_v = clamp(out.batt_v + 0.08 * k * smooth_noise(now_s, 0.8, 61.0), 11.5, 15.5)

    if out.vtec:
        out.rpm = max(out.rpm, 5600.0)
        out.tps = max(out.tps, 35.0)
    return out


def with_auto_profile(v: SimValues, now_s: float) -> SimValues:
    if not v.auto_profile:
        return v

    out = SimValues(**v.__dict__)
    duration = clamp(out.auto_accel_sec, 5.0, 20.0)
    phase = (now_s % duration) / duration
    k = smoothstep01(phase)

    auto_rpm = 400.0 + (9000.0 - 400.0) * k
    auto_tps = 2.0 + (100.0 - 2.0) * k
    auto_map = out.auto_map_start + (out.auto_map_end - out.auto_map_start) * k
    auto_afr = out.auto_afr_start + (out.auto_afr_end - out.auto_afr_start) * k

    mix = clamp(out.auto_manual_mix, 0.0, 1.0)
    out.rpm = clamp(auto_rpm + (out.rpm - auto_rpm) * mix, 400.0, 9200.0)
    out.tps = clamp(auto_tps + (out.tps - auto_tps) * mix, 0.0, 100.0)
    out.map_mbar = clamp(auto_map + (out.map_mbar - auto_map) * mix, -70.0, 1790.0)
    out.afr = clamp(auto_afr + (out.afr - auto_afr) * mix, 10.0, 18.0)
    return out


def build_packet(v: SimValues, now_s: float) -> bytes:
    state = with_auto_profile(v, now_s)
    state = with_variation(state, now_s)
    p = bytearray(PACKET_SIZE)

    p[0] = temp_c_to_raw(state.ect_c)
    p[1] = temp_c_to_raw(state.iat_c)
    p[2] = afr_to_wb_raw(state.afr)
    p[4] = mbar_to_raw(state.map_mbar)
    p[5] = tps_to_raw(state.tps)

    div = rpm_to_div(state.rpm)
    p[6] = div & 0xFF
    p[7] = (div >> 8) & 0xFF

    speed_kmh = clamp((state.rpm - 900.0) * 0.02, 0.0, 240.0)
    p[16] = int(round(speed_kmh))

    inj_ms = clamp(1.2 + (state.tps / 100.0) * 8.5 + (state.rpm / 9000.0) * 2.8, 1.0, 12.5)
    inj_raw = int(clamp(round(inj_ms * 1000.0 / 3.2), 0, 65535))
    p[17] = inj_raw & 0xFF
    p[18] = (inj_raw >> 8) & 0xFF

    ign_deg = 14.0 + (state.rpm / 1000.0) * 0.9 - (state.map_mbar - 1013.0) * 0.005
    p[19] = ign_deg_to_raw(clamp(ign_deg, -2.0, 38.0))

    p[21] = 0x00
    p[23] = 0x80 if (state.vtec or (state.rpm >= 5600.0 and state.tps >= 35.0)) else 0x00

    p[24] = int(clamp(round(2.2 * 255.0 / 5.0), 0, 255))
    p[25] = battery_v_to_raw(state.batt_v)

    p[26] = 128
    for lo_i, hi_i in [(27, 28), (29, 30), (31, 32)]:
        p[lo_i] = 0x00
        p[hi_i] = 0x80
    p[33] = 128
    p[34] = 128
    p[35] = 128
    p[36] = 128

    p[39] = 0x40 if state.map_mbar > 1200 else 0x00
    p[40] = 40
    p[41] = 50

    iacv_raw = iacv_duty_to_raw(35.0 if state.rpm < 1500 else 15.0)
    p[49] = iacv_raw & 0xFF
    p[50] = (iacv_raw >> 8) & 0xFF

    return bytes(p)


class ProtocolServer:
    def __init__(self, shared: SharedState, mode: str, port: str | None) -> None:
        self.shared = shared
        self.mode = mode
        self.port = port
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.master_fd: int | None = None
        self.slave_fd: int | None = None
        self.slave_path: str | None = None
        self.ser = None
        self.start_time = time.time()

    def start(self) -> str:
        self.stop_event.clear()
        self.start_time = time.time()

        if self.mode == "pty":
            if pty is None:
                raise RuntimeError("PTY mode is not available on Windows. Use mode=serial.")
            self.master_fd, self.slave_fd = pty.openpty()
            self.slave_path = os.ttyname(self.slave_fd)
            self.thread = threading.Thread(target=self._run_pty, daemon=True)
            self.thread.start()
            return self.slave_path

        if serial is None:
            raise RuntimeError("pyserial is required for --mode serial (pip install pyserial)")
        if not self.port:
            raise RuntimeError("--port is required in serial mode")

        self.ser = serial.Serial(self.port, 38400, timeout=0.2)
        self.thread = threading.Thread(target=self._run_serial, daemon=True)
        self.thread.start()
        return self.port

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except Exception:
                pass
            self.master_fd = None

        if self.slave_fd is not None:
            try:
                os.close(self.slave_fd)
            except Exception:
                pass
            self.slave_fd = None

    def _run_pty(self) -> None:
        assert self.master_fd is not None
        fd = self.master_fd

        while not self.stop_event.is_set():
            try:
                r, _, _ = select.select([fd], [], [], 0.05)
            except Exception:
                continue
            if not r:
                continue

            try:
                cmd = os.read(fd, 1)
            except OSError:
                continue
            if not cmd:
                continue

            reply = self._reply_for(cmd[0])
            if reply is None:
                continue

            try:
                os.write(fd, reply)
            except OSError:
                continue

    def _run_serial(self) -> None:
        while not self.stop_event.is_set():
            try:
                cmd = self.ser.read(1)
            except Exception:
                time.sleep(0.05)
                continue

            if not cmd:
                continue
            reply = self._reply_for(cmd[0])
            if reply is None:
                continue

            try:
                self.ser.write(reply)
            except Exception:
                time.sleep(0.05)

    def _reply_for(self, cmd: int) -> bytes | None:
        if cmd == 0x10:
            return b"\xCD"
        if cmd == 0x20:
            now_s = time.time() - self.start_time
            return build_packet(self.shared.snapshot(), now_s)
        return None


class SimulatorGui:
    def __init__(self, mode: str, port: str | None) -> None:
        self.mode = mode
        self.port = port
        self.shared = SharedState()
        self.server: ProtocolServer | None = None

        self.root = tk.Tk()
        self.root.title("Honda Datalog Simulator (Linux GUI)")
        self.root.geometry("860x740")

        self.status_var = tk.StringVar(value="Stopped")
        self.port_var = tk.StringVar(value="-")
        self.selected_serial_port_var = tk.StringVar(value=port or "")
        self.mode_var = tk.StringVar(value=mode)
        self.serial_port_values: list[str] = []

        self.rpm_var = tk.DoubleVar(value=1200.0)
        self.tps_var = tk.DoubleVar(value=8.0)
        self.map_var = tk.DoubleVar(value=450.0)
        self.afr_var = tk.DoubleVar(value=14.5)
        self.iat_var = tk.DoubleVar(value=30.0)
        self.ect_var = tk.DoubleVar(value=85.0)
        self.batt_var = tk.DoubleVar(value=13.8)
        self.auto_profile_var = tk.BooleanVar(value=False)
        self.auto_accel_sec_var = tk.DoubleVar(value=10.0)
        self.auto_afr_start_var = tk.DoubleVar(value=11.0)
        self.auto_afr_end_var = tk.DoubleVar(value=16.0)
        self.auto_map_start_var = tk.DoubleVar(value=320.0)
        self.auto_map_end_var = tk.DoubleVar(value=1650.0)
        self.auto_manual_mix_var = tk.DoubleVar(value=0.35)
        self.auto_var = tk.BooleanVar(value=True)
        self.var_lvl_var = tk.DoubleVar(value=0.5)
        self.vtec_var = tk.BooleanVar(value=False)

        self._build_ui()
        self.refresh_serial_ports()
        self._tick_sync_state()

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Status:").grid(row=0, column=0, sticky="w")
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=1, sticky="w", padx=(4, 16))
        ttk.Label(top, text="Port:").grid(row=0, column=2, sticky="w")
        ttk.Label(top, textvariable=self.port_var).grid(row=0, column=3, sticky="w", padx=(4, 16))
        ttk.Label(top, text="Mode:").grid(row=0, column=6, sticky="e", padx=(12, 0))
        ttk.Label(top, textvariable=self.mode_var).grid(row=0, column=7, sticky="w", padx=(4, 0))

        self.start_btn = ttk.Button(top, text="Start", command=self.start)
        self.start_btn.grid(row=0, column=4, padx=4)
        self.stop_btn = ttk.Button(top, text="Stop", command=self.stop, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=5, padx=4)

        serial_row = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        serial_row.pack(fill=tk.X)
        ttk.Label(serial_row, text="Serial port:").grid(row=0, column=0, sticky="w")
        self.serial_combo = ttk.Combobox(
            serial_row,
            textvariable=self.selected_serial_port_var,
            values=self.serial_port_values,
            width=24,
        )
        self.serial_combo.grid(row=0, column=1, sticky="ew", padx=(6, 6))
        ttk.Button(serial_row, text="Refresh ports", command=self.refresh_serial_ports).grid(
            row=0, column=2, padx=(0, 6)
        )
        ttk.Label(serial_row, text="(COM12 or /dev/ttyUSB0)", foreground="#445").grid(
            row=0, column=3, sticky="w"
        )
        serial_row.columnconfigure(1, weight=1)

        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        self._slider(main, 0, "RPM", self.rpm_var, 800.0, 9200.0)
        self._slider(main, 1, "TPS %", self.tps_var, 0.0, 100.0)
        self._slider(main, 2, "MAP mbar", self.map_var, -70.0, 1790.0)
        self._slider(main, 3, "AFR", self.afr_var, 10.0, 18.0)
        self._slider(main, 4, "IAT C", self.iat_var, -20.0, 90.0)
        self._slider(main, 5, "ECT C", self.ect_var, 40.0, 120.0)
        self._slider(main, 6, "Battery V", self.batt_var, 11.5, 15.5)

        opts = ttk.LabelFrame(main, text="Options", padding=8)
        opts.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(10, 0))

        ttk.Checkbutton(opts, text="VTEC force ON", variable=self.vtec_var).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(opts, text="Auto variation", variable=self.auto_var).grid(row=0, column=1, sticky="w", padx=(18, 0))
        ttk.Label(opts, text="Variation level").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Scale(opts, from_=0.0, to=1.0, variable=self.var_lvl_var, orient=tk.HORIZONTAL).grid(
            row=1, column=1, sticky="ew", padx=(8, 0), pady=(8, 0)
        )
        opts.columnconfigure(1, weight=1)

        auto = ttk.LabelFrame(main, text="AUTO Acceleration", padding=8)
        auto.grid(row=8, column=0, columnspan=3, sticky="ew", pady=(10, 0))

        ttk.Checkbutton(auto, text="Enable AUTO profile", variable=self.auto_profile_var).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(auto, text="Accel duration (sec)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Scale(auto, from_=5.0, to=20.0, variable=self.auto_accel_sec_var, orient=tk.HORIZONTAL).grid(
            row=1, column=1, sticky="ew", padx=(8, 0), pady=(6, 0)
        )

        ttk.Label(auto, text="AFR start").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Scale(auto, from_=10.0, to=18.0, variable=self.auto_afr_start_var, orient=tk.HORIZONTAL).grid(
            row=2, column=1, sticky="ew", padx=(8, 0), pady=(6, 0)
        )

        ttk.Label(auto, text="AFR end").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Scale(auto, from_=10.0, to=18.0, variable=self.auto_afr_end_var, orient=tk.HORIZONTAL).grid(
            row=3, column=1, sticky="ew", padx=(8, 0), pady=(6, 0)
        )

        ttk.Label(auto, text="MAP start (mbar)").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Scale(auto, from_=-70.0, to=1790.0, variable=self.auto_map_start_var, orient=tk.HORIZONTAL).grid(
            row=4, column=1, sticky="ew", padx=(8, 0), pady=(6, 0)
        )

        ttk.Label(auto, text="MAP end (mbar)").grid(row=5, column=0, sticky="w", pady=(6, 0))
        ttk.Scale(auto, from_=-70.0, to=1790.0, variable=self.auto_map_end_var, orient=tk.HORIZONTAL).grid(
            row=5, column=1, sticky="ew", padx=(8, 0), pady=(6, 0)
        )

        ttk.Label(auto, text="Manual slider influence").grid(row=6, column=0, sticky="w", pady=(6, 0))
        ttk.Scale(auto, from_=0.0, to=1.0, variable=self.auto_manual_mix_var, orient=tk.HORIZONTAL).grid(
            row=6, column=1, sticky="ew", padx=(8, 0), pady=(6, 0)
        )

        auto.columnconfigure(1, weight=1)

        note = ttk.Label(
            main,
            text=(
                "Protocol: 0x10->0xCD, 0x20->52 bytes. "
                "Use the shown port in SupraRom Studio DATALOG."
            ),
            foreground="#334",
        )
        note.grid(row=9, column=0, columnspan=3, sticky="w", pady=(12, 0))

        main.columnconfigure(1, weight=1)

    def _slider(self, parent: ttk.Frame, row: int, text: str, var: tk.DoubleVar, lo: float, hi: float) -> None:
        ttk.Label(parent, text=text, width=12).grid(row=row, column=0, sticky="w")
        ttk.Scale(parent, from_=lo, to=hi, variable=var, orient=tk.HORIZONTAL).grid(
            row=row, column=1, sticky="ew", padx=6
        )
        ttk.Label(parent, textvariable=tk.StringVar(value=""), width=1).grid(row=row, column=2, sticky="e")

        value_label = ttk.Label(parent, width=10, anchor="e")
        value_label.grid(row=row, column=2, sticky="e")

        def refresh_label(*_args: object) -> None:
            value_label.configure(text=f"{var.get():.2f}")

        var.trace_add("write", refresh_label)
        refresh_label()

    def _tick_sync_state(self) -> None:
        self.shared.set_values(
            SimValues(
                rpm=self.rpm_var.get(),
                tps=self.tps_var.get(),
                map_mbar=self.map_var.get(),
                afr=self.afr_var.get(),
                iat_c=self.iat_var.get(),
                ect_c=self.ect_var.get(),
                batt_v=self.batt_var.get(),
                vtec=self.vtec_var.get(),
                auto_profile=self.auto_profile_var.get(),
                auto_accel_sec=self.auto_accel_sec_var.get(),
                auto_afr_start=self.auto_afr_start_var.get(),
                auto_afr_end=self.auto_afr_end_var.get(),
                auto_map_start=self.auto_map_start_var.get(),
                auto_map_end=self.auto_map_end_var.get(),
                auto_manual_mix=self.auto_manual_mix_var.get(),
                auto_variation=self.auto_var.get(),
                variation_level=self.var_lvl_var.get(),
            )
        )
        self.root.after(80, self._tick_sync_state)

    def refresh_serial_ports(self) -> None:
        ports: list[str] = []
        if list_ports is not None:
            try:
                ports = sorted(p.device for p in list_ports.comports())
            except Exception:
                ports = []

        self.serial_port_values = ports
        self.serial_combo["values"] = ports

        current = self.selected_serial_port_var.get().strip()
        if not current and ports:
            self.selected_serial_port_var.set(ports[0])

    def start(self) -> None:
        if self.server is not None:
            return

        selected_port = self.port
        if self.mode == "serial":
            selected_port = self.selected_serial_port_var.get().strip() or self.port
            if not selected_port:
                self.status_var.set("Error: select serial port (ex: COM12)")
                return

        try:
            self.server = ProtocolServer(self.shared, self.mode, selected_port)
            active_port = self.server.start()
        except Exception as e:
            self.server = None
            self.status_var.set(f"Error: {e}")
            return

        self.status_var.set("Running")
        self.port_var.set(active_port)
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)

    def stop(self) -> None:
        if self.server is None:
            return

        self.server.stop()
        self.server = None
        self.status_var.set("Stopped")
        self.start_btn.configure(state=tk.NORMAL)
        self.stop_btn.configure(state=tk.DISABLED)

    def run(self) -> None:
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self) -> None:
        self.stop()
        self.root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="Honda datalog GUI simulator")
    if os.name == "nt":
        mode_choices = ["serial"]
        default_mode = "serial"
    else:
        mode_choices = ["pty", "serial"]
        default_mode = "pty"

    parser.add_argument("--mode", choices=mode_choices, default=default_mode)
    parser.add_argument("--port", help="Serial port path for --mode serial (ex: /dev/ttyUSB0)")
    args = parser.parse_args()

    app = SimulatorGui(mode=args.mode, port=args.port)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
