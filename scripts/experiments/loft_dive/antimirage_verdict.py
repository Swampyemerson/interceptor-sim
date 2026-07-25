#!/usr/bin/env python3
"""Anti-mirage 2x2 verdict for the Phase A loft-dive A/B (ADR-0076 #18g/#18h).

The question the Phase A verdict left OPEN: is the with-camera intercept
CAMERA-DRIVEN, or is the recorded miss just the open-loop coded-dash's
ballistic CPA (a "mirage" — the camera terminal added ~nothing)?

The decisive test is a paired dash-only CONTROL arm: same geometry, same
seeds, but the acquire gate is held shut (--coded-dash-acquire-range-min 999)
so ENGAGE never fires -> the recorded miss is pure dash ballistics through CPA.
Those control flights exit clean=0 / breakoff=python_exit_1 (no camera intercept
ever completed); their miss_m is still the true CPA of the airframe.

Verdict logic, PER DIRECTION, PAIRED by run_idx (seed-matched geometry):
  delta = miss(camera ON) - miss(camera OFF/dash-only)
    delta << 0  -> camera TIGHTENS the miss  -> camera-DRIVEN (not a mirage)
    delta ~ 0   -> camera changed nothing     -> MIRAGE (dash ballistics)
    delta >> 0  -> camera LOOSENS the miss     -> camera steers it WRONG
                                                  (real effect, wrong sign;
                                                   e.g. ADR-0056 aspect bias)
A single median hides the pairing; we report the paired per-flight deltas,
their median + sign-consistency, and a Wilcoxon-style sign tally. No wall time,
no gt in the guidance path — this reads only logged miss_m (a scoring quantity).

Usage:
  antimirage_verdict.py A=logs/mc_loftdive_armA_line9_s123.csv \
                        Adash=logs/mc_loftdive_armAdash_line9_s123.csv \
                        B=logs/mc_loftdive_armB_line9_s123.csv \
                        Bdash=logs/mc_loftdive_armBdash_line9_s123.csv
"""
import csv
import statistics as st
import sys

# Per-flight geometry columns that MUST match between a camera arm and its
# dash-only control for the pairing to mean anything. cue_seed is the decisive
# one: it is the only column that distinguishes the seed-123 and seed-777
# replicas of the same arm (direction cannot -- all flown arms share one
# direction sequence), and a one-character filename slip pairs run_idx 0 of one
# master seed against run_idx 0 of a completely different geometry.
GEOM_KEYS = ("target_start_y", "target_start_x", "target_vy")
GEOM_TOL = 1e-6


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r.get("miss_m"):
                rows[int(r["run_idx"])] = {
                    "miss": float(r["miss_m"]),
                    "dir": r["direction"],
                    "clean": r["clean"],
                    "handoff": r.get("handoff") or "",
                    # Pairing identity (2026-07-25, review 2 MEDIUM). These were
                    # DISCARDED by load(), so nothing could check that the two
                    # arms were the same experiment.
                    "cue_seed": (r.get("cue_seed") or "").strip(),
                    "path_seed": (r.get("path_seed") or "").strip(),
                    "geom": {k: _f(r.get(k)) for k in GEOM_KEYS},
                    "src": path,
                }
    return rows


def direction_of(rows, idx):
    return rows[idx]["dir"]


def assert_paired_identity(cam, dash, label):
    """HARD GUARD: every paired run_idx must be the SAME flight geometry in both
    arms (2026-07-25, review 2).

    The tool used to pair on `i in dash and cam[i]['dir'] == d` alone -- it never
    checked the CONTROL arm's direction, cue_seed or start geometry, even though
    all three columns sit in the CSVs it reads. Passing two arms from different
    master seeds (the seed-123 / seed-777 replicas now sit side by side in
    logs/) produced a full per-flight table and a confident
    "CAMERA-DRIVEN ... NOT a mirage" verdict off completely unrelated flights.
    On any mismatch we print the offending run_idx and EXIT 2 without printing a
    verdict at all -- a wrong verdict here is worse than no verdict, because this
    is the gate that stands between the project and a repeat of the retracted
    "--track breakthrough" geometry artifact (ADR-0076 add #7/#8).
    """
    bad = []
    for i in sorted(set(cam) & set(dash)):
        c, d = cam[i], dash[i]
        if c["dir"] != d["dir"]:
            bad.append(f"run_idx={i}: direction {c['dir']} vs {d['dir']}")
            continue
        if c["cue_seed"] != d["cue_seed"]:
            bad.append(f"run_idx={i}: cue_seed {c['cue_seed']!r} vs "
                       f"{d['cue_seed']!r} -- arms are NOT seed-matched")
            continue
        if c["path_seed"] != d["path_seed"]:
            bad.append(f"run_idx={i}: path_seed {c['path_seed']!r} vs "
                       f"{d['path_seed']!r}")
            continue
        for k in GEOM_KEYS:
            a, b = c["geom"][k], d["geom"][k]
            if a is None or b is None:
                if a is not b:
                    bad.append(f"run_idx={i}: {k} present in one arm only "
                               f"({a} vs {b})")
                    break
                continue
            if abs(a - b) > GEOM_TOL:
                bad.append(f"run_idx={i}: {k} {a} vs {b} (delta {a - b:+.6g} m)")
                break
    if bad:
        print(f"\n{'='*66}\n{label}: GEOMETRY MISMATCH -- REFUSING TO PAIR\n{'='*66}",
              file=sys.stderr)
        print(f"  camera-ON arm : {next(iter(cam.values()))['src']}", file=sys.stderr)
        print(f"  dash-only arm : {next(iter(dash.values()))['src']}", file=sys.stderr)
        for b in bad[:12]:
            print(f"  {b}", file=sys.stderr)
        if len(bad) > 12:
            print(f"  ... and {len(bad) - 12} more", file=sys.stderr)
        print("  These are NOT the same flights. Pair each arm with the twin "
              "flown at the SAME master seed (mc_*_arm<X>_line9_s<SEED>.csv vs "
              "mc_*_arm<X>dash_line9_s<SEED>.csv). No verdict printed.",
              file=sys.stderr)
        sys.exit(2)


def paired(cam, dash, label):
    """cam = camera-ON arm rows, dash = dash-only control rows."""
    print(f"\n{'='*66}\n{label}: camera-ON vs dash-only control (paired by run_idx)\n{'='*66}")
    if cam and dash:
        # IDENTITY IN THE OUTPUT: name both source files, then hard-abort on any
        # per-flight mismatch before a single delta is printed.
        print(f"  camera-ON = {next(iter(cam.values()))['src']}")
        print(f"  dash-only = {next(iter(dash.values()))['src']}")
        assert_paired_identity(cam, dash, label)
    for d in ("l2r", "r2l"):
        idxs = sorted(i for i in cam if i in dash and cam[i]["dir"] == d)
        if not idxs:
            print(f"  {d}: no paired flights")
            continue
        deltas = []
        print(f"  {d}:  run  cam_miss  dash_miss   delta(cam-dash)")
        for i in idxs:
            c, dh = cam[i]["miss"], dash[i]["miss"]
            deltas.append(c - dh)
            tag = ""
            if c - dh < -0.3:
                tag = "  <- camera tighter"
            elif c - dh > 0.3:
                tag = "  <- camera worse"
            print(f"       {i:>3}  {c:>7.2f}  {dh:>8.2f}   {c-dh:>+8.2f}{tag}")
        med = st.median(deltas)
        n_tight = sum(1 for x in deltas if x < -0.3)
        n_worse = sum(1 for x in deltas if x > 0.3)
        n_same = len(deltas) - n_tight - n_worse
        cam_med = st.median([cam[i]["miss"] for i in idxs])
        dash_med = st.median([dash[i]["miss"] for i in idxs])
        # plain verdict
        if n_tight >= max(1, len(deltas) - 1) and med < -0.3:
            verd = "CAMERA-DRIVEN (tightens, consistent) -> NOT a mirage"
        elif n_same >= len(deltas) - 1 and abs(med) <= 0.3:
            verd = "MIRAGE: camera ~= dash ballistics (adds nothing)"
        elif n_worse >= max(1, len(deltas) - 1) and med > 0.3:
            verd = "CAMERA STEERS WRONG (real effect, wrong sign; aspect bias?)"
        else:
            verd = "MIXED: no consistent camera effect at this n"
        print(f"    -> n={len(deltas)}  cam_med={cam_med:.2f}  dash_med={dash_med:.2f}  "
              f"paired_delta_med={med:+.2f}")
        print(f"       tighter/worse/same = {n_tight}/{n_worse}/{n_same}   VERDICT: {verd}")


def main(argv):
    arms = {}
    for a in argv[1:]:
        k, _, path = a.partition("=")
        arms[k] = load(path)
    missing = [k for k in ("A", "Adash", "B", "Bdash") if k not in arms]
    if missing:
        print(f"[antimirage] note: missing arms {missing} (pass them to compare)")
    if "A" in arms and "Adash" in arms:
        paired(arms["A"], arms["Adash"], "ARM A (flat dash baseline, no lever)")
    if "B" in arms and "Bdash" in arms:
        paired(arms["B"], arms["Bdash"], "ARM B (loft-dive + accel-cap LEVER)")
    print("\n(reads logged miss_m only; guidance never sees gt. "
          "'dash-only' = --coded-dash-acquire-range-min 999, ENGAGE never fires.)")


if __name__ == "__main__":
    main(sys.argv)
