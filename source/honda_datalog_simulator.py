#!/usr/bin/env python3
"""Simple standalone Honda datalog packet simulator.

This script emulates the HTS-like serial protocol used by docs/main.rs:
- waits for command byte 0x20
- replies with a 52-byte packet

It is intentionally separate from the app so it can be enabled/disabled easily.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import time
from dataclasses import dataclass

try:
    import pty
except Exception:
    pty = None

try:
    import serial  # type: ignore
except Exception:
    serial = None

PACKET_SIZE = 52
TICK_SEC = 0.05


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def smoothstep01(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def fract(x: float) -> float:
    return x - math.floor(x)


def hash01(n: float) -> float:
    # Deterministic pseudo-random number in [0, 1].
    return fract(math.sin(n * 12.9898 + 78.233) * 43758.5453)


def smooth_noise(now_s: float, hz: float, seed: float) -> float:
    # Smoothed 1D value noise in [-1, 1].
    x = now_s * hz + seed * 19.19
    i = math.floor(x)
    f = x - i
    a = hash01(i + seed * 101.0)
    b = hash01(i + 1.0 + seed * 101.0)
    t = smoothstep01(f)
    return ((a + (b - a) * t) * 2.0) - 1.0


def temp_c_to_raw(target_c: float) -> int:
    # Same inverse strategy as main.rs (search by minimum error)
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
    # Inverse of mbar ~= 365.9 * volts - 29.9 ; raw = volts * 255/5
    volts = (map_mbar + 29.9) / 365.9
    raw = round(volts * 255.0 / 5.0)
    return int(clamp(raw, 0, 255))


def tps_to_raw(tps_pct: float) -> int:
    raw = round(tps_pct * 2.04 + 25.0)
    return int(clamp(raw, 0, 255))


def afr_to_wb_raw(afr: float) -> int:
    # main.rs: lambda = 0.71 + wb_volt * ((1.3-0.71)/5)
    lam = afr / 14.7
    wb_volt = (lam - 0.71) * (5.0 / (1.3 - 0.71))
    raw = round(clamp(wb_volt, 0.0, 5.0) * 255.0 / 5.0)
    return int(clamp(raw, 0, 255))


def rpm_to_div(rpm: float) -> int:
    if rpm <= 1.0:
        return 65535
    div = round(1_851_562.0 / rpm)
    return int(clamp(div, 1, 65535))


def inj_ms_to_raw(inj_ms: float) -> int:
    raw = round(inj_ms * 1000.0 / 3.2)
    return int(clamp(raw, 0, 65535))


def ign_deg_to_raw(deg: float) -> int:
    raw = round((deg + 6.0) / 0.25)
    return int(clamp(raw, 0, 255))


def battery_v_to_raw(v: float) -> int:
    raw = round(v * 270.0 / 26.0)
    return int(clamp(raw, 0, 255))


def iacv_duty_to_raw(duty: float) -> int:
    raw = round((duty + 100.0) * 327.68)
    return int(clamp(raw, 0, 65535))


@dataclass
class SimState:
    rpm: float
    tps: float
    map_mbar: float
    afr: float
    battery_v: float
    ign_deg: float
    vtec: bool
    speed_kmh: float


def simulated_state(now_s: float) -> SimState:
    # 20 s cycle: continuous acceleration ramp (no decel segment).
    cycle = 20.0
    t = now_s % cycle

    k = smoothstep01(t / cycle)
    rpm = 950.0 + (9000.0 - 950.0) * k
    tps = 3.0 + (100.0 - 3.0) * k
    map_mbar = 360.0 + (1750.0 - 360.0) * k
    afr = 14.7 + (11.8 - 14.7) * k

    # Add smooth random behavior so traces are less repetitive and more life-like.
    driver_aggr = 0.5 + 0.5 * smooth_noise(now_s, 0.035, 7.0)
    rpm += (130.0 + 170.0 * driver_aggr) * smooth_noise(now_s, 0.22, 11.0)
    rpm += (55.0 + 85.0 * driver_aggr) * smooth_noise(now_s, 1.15, 12.0)

    tps += (1.8 + 2.8 * driver_aggr) * smooth_noise(now_s, 0.35, 21.0)
    tps += (0.8 + 1.3 * driver_aggr) * smooth_noise(now_s, 1.5, 22.0)

    map_mbar += (35.0 + 55.0 * driver_aggr) * smooth_noise(now_s, 0.28, 31.0)
    map_mbar += (16.0 + 24.0 * driver_aggr) * smooth_noise(now_s, 1.1, 32.0)

    afr += (0.10 + 0.18 * driver_aggr) * smooth_noise(now_s, 0.25, 41.0)
    afr += (0.04 + 0.08 * driver_aggr) * smooth_noise(now_s, 1.35, 42.0)

    # Mild enrichment pulses under load while preserving an acceleration trend.
    enr_evt = clamp((smooth_noise(now_s, 0.16, 95.0) - 0.80) / 0.20, 0.0, 1.0)
    rpm += 180.0 * enr_evt
    map_mbar += 120.0 * enr_evt
    afr -= 0.55 * enr_evt

    # Keep MAP loosely correlated to driver demand.
    map_mbar += (tps - 50.0) * 1.05

    rpm = clamp(rpm, 800.0, 9200.0)
    tps = clamp(tps, 0.0, 100.0)
    map_mbar = clamp(map_mbar, -70.0, 1790.0)
    afr = clamp(afr, 10.0, 18.0)

    vtec = rpm >= 5600.0 and tps >= 35.0
    battery_v = 13.85 + 0.16 * math.sin(now_s * 0.9) + 0.08 * smooth_noise(now_s, 0.4, 61.0)
    ign_deg = 16.0 + (rpm / 1000.0) * 0.8 - (map_mbar - 1013.0) * 0.006
    ign_deg = clamp(ign_deg, -2.0, 35.0)

    speed_kmh = clamp((rpm - 900.0) * 0.02, 0.0, 220.0)

    return SimState(
        rpm=rpm,
        tps=tps,
        map_mbar=map_mbar,
        afr=afr,
        battery_v=battery_v,
        ign_deg=ign_deg,
        vtec=vtec,
        speed_kmh=speed_kmh,
    )


def build_packet(state: SimState) -> bytes:
    p = bytearray(PACKET_SIZE)

    p[0] = temp_c_to_raw(86.0)   # ECT
    p[1] = temp_c_to_raw(34.0)   # IAT
    p[2] = afr_to_wb_raw(state.afr)  # wideband 0-5V input
    p[4] = mbar_to_raw(state.map_mbar)
    p[5] = tps_to_raw(state.tps)

    div = rpm_to_div(state.rpm)
    p[6] = div & 0xFF
    p[7] = (div >> 8) & 0xFF

    p[16] = int(clamp(round(state.speed_kmh), 0, 255))

    # injector pulse estimate from load
    inj_ms = clamp(1.4 + (state.tps / 100.0) * 8.8 + (state.rpm / 9000.0) * 3.0, 1.2, 12.5)
    inj_raw = inj_ms_to_raw(inj_ms)
    p[17] = inj_raw & 0xFF
    p[18] = (inj_raw >> 8) & 0xFF

    p[19] = ign_deg_to_raw(state.ign_deg)

    # inputs byte
    p[21] = 0x00

    # output/status bits: set VTS on bit7 in packet[23]
    p[23] = 0x80 if state.vtec else 0x00

    p[24] = int(clamp(round(2.2 * 255.0 / 5.0), 0, 255))  # ELD volt equivalent
    p[25] = battery_v_to_raw(state.battery_v)

    # fuel correction and trims neutral
    p[26] = 128
    for lo_i, hi_i in [(27, 28), (29, 30), (31, 32)]:
        p[lo_i] = 0x00
        p[hi_i] = 0x80
    p[33] = 128
    p[34] = 128
    p[35] = 128
    p[36] = 128

    # ebc/base duty placeholders
    p[40] = 40
    p[41] = 50

    # fan / outputs at packet[39]
    p[39] = 0x40 if state.map_mbar > 1200 else 0x00

    iacv_raw = iacv_duty_to_raw(35.0 if state.rpm < 1500 else 15.0)
    p[49] = iacv_raw & 0xFF
    p[50] = (iacv_raw >> 8) & 0xFF

    return bytes(p)


def run_pty_mode() -> None:
    if pty is None:
        raise RuntimeError(
            "PTY mode is only available on Unix-like systems. "
            "On Windows, create a virtual COM pair first (for example with com0com) "
            "or use a real serial adapter, then run: "
            "python scripts/honda_datalog_simulator.py --mode serial --port COM5"
        )

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)

    print("Honda datalog simulator started (PTY mode)")
    print(f"Connect app datalog port to: {slave_path}")
    print("Press Ctrl+C to stop.")

    t0 = time.time()
    try:
        while True:
            r, _, _ = select.select([master_fd], [], [], TICK_SEC)
            if not r:
                continue
            try:
                cmd = os.read(master_fd, 1)
            except OSError:
                # No active peer yet or peer disconnected: keep simulator alive.
                continue
            if not cmd:
                continue
            if cmd[0] == 0x10:
                # HTS handshake expected by connect_hts in docs/main.rs.
                reply = b"\xCD"
            elif cmd[0] == 0x20:
                state = simulated_state(time.time() - t0)
                reply = build_packet(state)
            else:
                continue

            try:
                os.write(master_fd, reply)
            except OSError:
                # Ignore transient peer disconnects and keep waiting.
                continue
    finally:
        os.close(master_fd)
        os.close(slave_fd)


def run_serial_mode(port_name: str) -> None:
    if serial is None:
        raise RuntimeError("pyserial is required for --mode serial (pip install pyserial)")

    print(f"Honda datalog simulator started (serial mode): {port_name}")
    print("Press Ctrl+C to stop.")

    with serial.Serial(port_name, 38400, timeout=0.2) as ser:
        t0 = time.time()
        while True:
            cmd = ser.read(1)
            if not cmd:
                continue
            if cmd[0] == 0x10:
                ser.write(b"\xCD")
            elif cmd[0] == 0x20:
                state = simulated_state(time.time() - t0)
                ser.write(build_packet(state))


def main() -> int:
    parser = argparse.ArgumentParser(description="Standalone Honda datalog simulator")
    parser.add_argument("--mode", choices=["pty", "serial"], default="pty")
    parser.add_argument(
        "--port",
        help="Serial port path for --mode serial (ex: /dev/ttyUSB0 or COM5)",
    )
    args = parser.parse_args()

    try:
        if args.mode == "serial":
            if not args.port:
                print("--port is required in serial mode", file=sys.stderr)
                return 2
            run_serial_mode(args.port)
        else:
            run_pty_mode()
    except KeyboardInterrupt:
        print("\nSimulator stopped.")
        return 0
    except Exception as e:
        print(f"Simulator error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
