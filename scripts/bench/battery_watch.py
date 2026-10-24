#!/usr/bin/env python3
"""Bench battery-voltage watcher for the interceptor (6S pack, PX4 over TELEM2).

PURPOSE: the vehicle sits on the bench powered by its flight battery while
other bench work proceeds; nothing in the stack would notice the pack sagging
toward over-discharge. This tool samples the FC's own battery telemetry in
short BURSTS (open the port, read, close) so the TELEM2 UART stays free for
brn-05 / fc_link_check / any other bench tool between samples.

READ-ONLY, mechanically: the only MAVLink this sends is REQUEST_DATA_STREAM
(the same read-only pattern as scripts/bench/fc_status_read.py and the
scripts/field/fc_link_check.py audit). No arming, no params, no commands.

THRESHOLDS (6S; per-cell = pack/6 -- standard LiPo practice, e.g. the
ubiquitous "storage 3.8 V/cell, never below 3.5 V/cell under no load" rule;
the bench floor is set ABOVE field floors because a bench pack is idle, not
earning its discharge):
    WARN      22.8 V  = 3.80 V/cell   finish up; this is the bench floor
    CRITICAL  22.2 V  = 3.70 V/cell   stop the session, unplug the pack now

FAIL-CLOSED: an unreadable voltage is a MISS, logged loudly and counted --
never substituted. A status computed from zero samples says UNKNOWN, not OK;
a status whose newest sample is older than 3 intervals says UNKNOWN(stale).

Outputs under --out-dir (default <repo>/logs/battery_watch):
    bench_<UTC>.csv   one row per good sample (append-only, this session)
    status.json       latest sample + verdict + slope + projections (rewritten)
    CRITICAL          sentinel file, created the first time CRITICAL is hit

Run (on the Pi, where TELEM2 lives):
    .venv-pi/bin/python scripts/bench/battery_watch.py            # daemon loop
    .venv-pi/bin/python scripts/bench/battery_watch.py --once     # single burst
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime, timezone
from typing import List, Optional, Tuple

CELLS_DEFAULT = 6
WARN_V_DEFAULT = 22.8    # 3.80 V/cell * 6
CRIT_V_DEFAULT = 22.2    # 3.70 V/cell * 6
PROJECTION_WINDOW_S = 900.0   # linear fit over the last 15 min of samples


# --------------------------------------------------------------------------
# Pure logic (unit-tested in tests/test_battery_watch.py, no hardware)
# --------------------------------------------------------------------------

def classify(pack_v: float, warn_v: float = WARN_V_DEFAULT,
             crit_v: float = CRIT_V_DEFAULT) -> str:
    """OK / WARN / CRITICAL for a measured pack voltage."""
    if pack_v <= crit_v:
        return "CRITICAL"
    if pack_v <= warn_v:
        return "WARN"
    return "OK"


def fit_slope_v_per_s(rows: List[Tuple[float, float]],
                      window_s: float = PROJECTION_WINDOW_S
                      ) -> Optional[float]:
    """Least-squares voltage slope (V/s) over the samples inside the trailing
    window. None when fewer than 2 samples are in the window (no fit is
    honest; a made-up slope is not)."""
    if not rows:
        return None
    t_now = rows[-1][0]
    win = [(t, v) for (t, v) in rows if t >= t_now - window_s]
    if len(win) < 2:
        return None
    n = len(win)
    mt = sum(t for t, _ in win) / n
    mv = sum(v for _, v in win) / n
    denom = sum((t - mt) ** 2 for t, _ in win)
    if denom == 0.0:
        return None
    return sum((t - mt) * (v - mv) for t, v in win) / denom


def project_time_to_s(rows: List[Tuple[float, float]], threshold_v: float,
                      window_s: float = PROJECTION_WINDOW_S
                      ) -> Optional[float]:
    """Seconds until the fitted line crosses threshold_v, from the newest
    sample. None when there is no fit or the pack is not discharging
    (slope >= 0 -- a charger, noise, or too little data; projecting a rise
    to a floor would be fiction). 0.0 when already at/below the threshold."""
    if not rows:
        return None
    t_last, v_last = rows[-1]
    if v_last <= threshold_v:
        return 0.0
    slope = fit_slope_v_per_s(rows, window_s)
    if slope is None or slope >= 0.0:
        return None
    return (threshold_v - v_last) / slope


# --------------------------------------------------------------------------
# Hardware burst (pymavlink imported lazily so the pure logic imports anywhere)
# --------------------------------------------------------------------------

def sample_burst(device: str, baud: int, listen_s: float = 5.0,
                 heartbeat_timeout_s: float = 8.0) -> Optional[dict]:
    """One open->read->close burst. Returns the sample dict, or None (MISS).
    Holds the serial port only for the duration of the call."""
    from pymavlink import mavutil
    m = None
    try:
        m = mavutil.mavlink_connection(device, baud=baud)
        hb = m.wait_heartbeat(timeout=heartbeat_timeout_s)
        if hb is None:
            return None
        if m.target_component == 0:
            m.target_component = 1
        # Read-only: ask for the standard streams for the listen window
        # (same as fc_status_read.py). PX4 on TELEM2/Onboard usually emits
        # SYS_STATUS unprompted; the request just bounds the wait.
        m.mav.request_data_stream_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)
        pack_v = current_a = remaining = None
        deadline = time.time() + listen_s
        while time.time() < deadline:
            msg = m.recv_match(type=("SYS_STATUS", "BATTERY_STATUS"),
                               blocking=True,
                               timeout=max(0.1, deadline - time.time()))
            if msg is None:
                continue
            if msg.get_type() == "SYS_STATUS":
                if msg.voltage_battery not in (0, 0xFFFF):
                    pack_v = msg.voltage_battery / 1000.0
                if msg.current_battery != -1:
                    current_a = msg.current_battery / 100.0
                if msg.battery_remaining != -1:
                    remaining = float(msg.battery_remaining)
            elif msg.get_type() == "BATTERY_STATUS":
                if msg.current_battery != -1 and current_a is None:
                    current_a = msg.current_battery / 100.0
            if pack_v is not None and current_a is not None \
                    and remaining is not None:
                break
        if pack_v is None:
            return None          # fail-closed: no voltage -> MISS, no default
        return {"t_unix": time.time(), "pack_v": pack_v,
                "current_a": current_a, "remaining_pct": remaining}
    except Exception as e:  # noqa: BLE001 -- any burst failure is a MISS
        print(f"[battery_watch] MISS: burst failed: {e}", flush=True)
        return None
    finally:
        if m is not None:
            try:
                m.close()
            except Exception:  # noqa: BLE001
                pass


# --------------------------------------------------------------------------
# Session loop
# --------------------------------------------------------------------------

def _utc_iso(t_unix: float) -> str:
    return datetime.fromtimestamp(t_unix, tz=timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")


def write_status(path: str, *, rows: List[Tuple[float, float]],
                 latest: Optional[dict], n_miss: int, cells: int,
                 warn_v: float, crit_v: float, interval_s: float) -> dict:
    """Rewrite status.json atomically; returns the status dict."""
    now = time.time()
    if latest is None or not rows:
        status = {"verdict": "UNKNOWN", "reason": "no samples yet",
                  "n_samples": len(rows), "n_misses": n_miss}
    else:
        stale = (now - latest["t_unix"]) > 3 * interval_s
        verdict = classify(latest["pack_v"], warn_v, crit_v)
        if stale:
            verdict = "UNKNOWN(stale)"
        slope = fit_slope_v_per_s(rows)
        status = {
            "verdict": verdict,
            "ts_utc": _utc_iso(latest["t_unix"]),
            "age_s": round(now - latest["t_unix"], 1),
            "pack_v": round(latest["pack_v"], 2),
            "cell_v": round(latest["pack_v"] / cells, 3),
            "current_a": latest["current_a"],
            "remaining_pct": latest["remaining_pct"],
            "slope_v_per_min": (None if slope is None
                                else round(slope * 60.0, 4)),
            "projected_s_to_warn": _round_opt(
                project_time_to_s(rows, warn_v)),
            "projected_s_to_critical": _round_opt(
                project_time_to_s(rows, crit_v)),
            "warn_v": warn_v, "crit_v": crit_v,
            "n_samples": len(rows), "n_misses": n_miss,
        }
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(status, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)
    return status


def _round_opt(x: Optional[float]) -> Optional[float]:
    return None if x is None else round(x, 0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--interval", type=float, default=60.0,
                    help="seconds between bursts (port is FREE in between)")
    ap.add_argument("--cells", type=int, default=CELLS_DEFAULT)
    ap.add_argument("--warn-v", type=float, default=WARN_V_DEFAULT)
    ap.add_argument("--crit-v", type=float, default=CRIT_V_DEFAULT)
    ap.add_argument("--out-dir", default=None,
                    help="default: <repo>/logs/battery_watch")
    ap.add_argument("--once", action="store_true",
                    help="one burst, print + write, exit 0 good / 1 MISS")
    args = ap.parse_args()

    repo = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    out_dir = args.out_dir or os.path.join(repo, "logs", "battery_watch")
    os.makedirs(out_dir, exist_ok=True)
    session = datetime.now(tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    csv_path = os.path.join(out_dir, f"bench_{session}.csv")
    status_path = os.path.join(out_dir, "status.json")
    crit_sentinel = os.path.join(out_dir, "CRITICAL")

    print(f"[battery_watch] {args.device}@{args.baud} every {args.interval:.0f}s"
          f" | WARN<={args.warn_v}V ({args.warn_v/args.cells:.2f}/cell)"
          f" CRIT<={args.crit_v}V ({args.crit_v/args.cells:.2f}/cell)"
          f" | csv={csv_path}", flush=True)

    rows: List[Tuple[float, float]] = []
    n_miss = 0
    latest: Optional[dict] = None
    wrote_header = False
    while True:
        t_burst = time.time()
        sample = sample_burst(args.device, args.baud)
        if sample is None:
            n_miss += 1
            print(f"[battery_watch] {_utc_iso(t_burst)} MISS "
                  f"(#{n_miss}; port busy or FC silent -- retrying next "
                  f"interval)", flush=True)
        else:
            latest = sample
            rows.append((sample["t_unix"], sample["pack_v"]))
            with open(csv_path, "a", newline="") as f:
                w = csv.writer(f)
                if not wrote_header and f.tell() == 0:
                    w.writerow(["ts_utc", "pack_v", "cell_v", "current_a",
                                "remaining_pct"])
                wrote_header = True
                w.writerow([_utc_iso(sample["t_unix"]),
                            f"{sample['pack_v']:.2f}",
                            f"{sample['pack_v']/args.cells:.3f}",
                            ("" if sample["current_a"] is None
                             else f"{sample['current_a']:.2f}"),
                            ("" if sample["remaining_pct"] is None
                             else f"{sample['remaining_pct']:.0f}")])
        status = write_status(status_path, rows=rows, latest=latest,
                              n_miss=n_miss, cells=args.cells,
                              warn_v=args.warn_v, crit_v=args.crit_v,
                              interval_s=args.interval)
        verdict = status["verdict"]
        line = (f"[battery_watch] {status.get('ts_utc', '--')} "
                f"{status.get('pack_v', '--')} V "
                f"({status.get('cell_v', '--')} V/cell) -> {verdict}")
        if verdict == "CRITICAL":
            line = ("\n" + "!" * 72 + f"\n{line}\n"
                    f"[battery_watch] STOP THE SESSION -- UNPLUG THE PACK "
                    f"(<= {args.crit_v} V = {args.crit_v/args.cells:.2f} "
                    f"V/cell)\n" + "!" * 72)
            with open(crit_sentinel, "a") as f:
                f.write(f"{_utc_iso(time.time())} pack_v="
                        f"{status.get('pack_v')}\n")
        elif verdict == "WARN":
            line += (f"  <-- bench floor reached ({args.warn_v} V); "
                     f"wrap up the battery work")
        print(line, flush=True)

        if args.once:
            return 0 if sample is not None else 1
        # Sleep the REMAINDER of the interval so the cadence holds even
        # though a burst itself takes several seconds.
        time.sleep(max(1.0, args.interval - (time.time() - t_burst)))


if __name__ == "__main__":
    sys.exit(main())
