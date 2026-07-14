#!/usr/bin/env python3
"""Fast serial-only Honda datalog simulator (no GUI).

Usage examples:
- python3 scripts/honda_datalog_simulator_com.py --port COM12
- python3 scripts/honda_datalog_simulator_com.py --port /dev/ttyUSB0

Protocol:
- 0x10 -> 0xCD
- 0x20 -> 52-byte packet
"""

from __future__ import annotations

import argparse
import math
import time

try:
    import serial  # type: ignore
except Exception as e:  # pragma: no cover
    raise SystemExit("pyserial is required: pip install pyserial") from e

PACKET_SIZE = 52


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def temp_c_to_raw(target_c: float) -> int:
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


def smoothstep01(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def simulated_values(now_s: float) -> tuple[float, float, float, float, float, float, bool]:
    # Simple 24s cycle: idle -> accel -> hold -> decel
    cycle = 24.0
    t = now_s % cycle

    if t < 2.0:
        rpm = 950.0
        tps = 3.0
        map_mbar = 360.0
        afr = 14.7
    elif t < 14.0:
        k = smoothstep01((t - 2.0) / 12.0)
        rpm = 1000.0 + (9000.0 - 1000.0) * k
        tps = 8.0 + (100.0 - 8.0) * k
        map_mbar = 450.0 + (1750.0 - 450.0) * k
        afr = 16.0 - (16.0 - 11.0) * k
    elif t < 18.0:
        rpm = 8800.0 + 120.0 * math.sin((t - 14.0) * 2.0)
        tps = 97.0
        map_mbar = 1680.0
        afr = 11.2
    else:
        k = smoothstep01((t - 18.0) / 6.0)
        rpm = 9000.0 - (9000.0 - 1100.0) * k
        tps = 95.0 - (95.0 - 5.0) * k
        map_mbar = 1700.0 - (1700.0 - 380.0) * k
        afr = 11.4 + (14.7 - 11.4) * k

    rpm = clamp(rpm, 400.0, 9200.0)
    tps = clamp(tps, 0.0, 100.0)
    map_mbar = clamp(map_mbar, -70.0, 1790.0)
    afr = clamp(afr, 10.0, 18.0)
    batt_v = 13.9 + 0.12 * math.sin(now_s * 0.8)
    ect_c = 86.0
    iat_c = 32.0 + 1.0 * math.sin(now_s * 0.15)
    vtec = rpm >= 5600.0 and tps >= 35.0
    return rpm, tps, map_mbar, afr, batt_v, iat_c, vtec


def build_packet(now_s: float) -> bytes:
    rpm, tps, map_mbar, afr, batt_v, iat_c, vtec = simulated_values(now_s)

    p = bytearray(PACKET_SIZE)
    p[0] = temp_c_to_raw(86.0)
    p[1] = temp_c_to_raw(iat_c)
    p[2] = afr_to_wb_raw(afr)
    p[4] = mbar_to_raw(map_mbar)
    p[5] = tps_to_raw(tps)

    div = rpm_to_div(rpm)
    p[6] = div & 0xFF
    p[7] = (div >> 8) & 0xFF

    speed_kmh = clamp((rpm - 900.0) * 0.02, 0.0, 240.0)
    p[16] = int(round(speed_kmh))

    inj_ms = clamp(1.2 + (tps / 100.0) * 8.5 + (rpm / 9000.0) * 2.8, 1.0, 12.5)
    inj_raw = int(clamp(round(inj_ms * 1000.0 / 3.2), 0, 65535))
    p[17] = inj_raw & 0xFF
    p[18] = (inj_raw >> 8) & 0xFF

    ign_deg = 14.0 + (rpm / 1000.0) * 0.9 - (map_mbar - 1013.0) * 0.005
    p[19] = ign_deg_to_raw(clamp(ign_deg, -2.0, 38.0))

    p[21] = 0x00
    p[23] = 0x80 if vtec else 0x00
    p[24] = int(clamp(round(2.2 * 255.0 / 5.0), 0, 255))
    p[25] = battery_v_to_raw(batt_v)

    p[26] = 128
    for lo_i, hi_i in [(27, 28), (29, 30), (31, 32)]:
        p[lo_i] = 0x00
        p[hi_i] = 0x80
    p[33] = 128
    p[34] = 128
    p[35] = 128
    p[36] = 128

    p[39] = 0x40 if map_mbar > 1200 else 0x00
    p[40] = 40
    p[41] = 50

    iacv_raw = iacv_duty_to_raw(35.0 if rpm < 1500 else 15.0)
    p[49] = iacv_raw & 0xFF
    p[50] = (iacv_raw >> 8) & 0xFF
    return bytes(p)


def run(port: str, baud: int) -> int:
    print(f"Honda COM simulator started on {port} @ {baud}")
    print("HTS handshake: 0x10->0xCD | Poll: 0x20->52 bytes")
    print("Press Ctrl+C to stop")

    with serial.Serial(port, baud, timeout=0.2) as ser:
        t0 = time.time()
        while True:
            cmd = ser.read(1)
            if not cmd:
                continue
            if cmd[0] == 0x10:
                ser.write(b"\xCD")
            elif cmd[0] == 0x20:
                ser.write(build_packet(time.time() - t0))


def main() -> int:
    parser = argparse.ArgumentParser(description="Fast COM-only Honda datalog simulator")
    parser.add_argument("--port", required=True, help="COM port (ex: COM12 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=38400, help="Serial baudrate (default: 38400)")
    args = parser.parse_args()

    try:
        return run(args.port, args.baud)
    except KeyboardInterrupt:
        print("\nSimulator stopped.")
        return 0
    except Exception as e:
        print(f"Simulator error: {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
