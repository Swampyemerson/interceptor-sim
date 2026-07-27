#!/usr/bin/env python3
"""skr-07 — sustained AprilTag / CPU-YOLO fps on the REAL Pi 5, through a thermal soak.

WHAT THIS IS FOR (read before quoting a number out of it)
--------------------------------------------------------
The tripod money gate (docs/tripod_test_protocol.md §8.1) burns range while the
5-in-a-row handoff streak forms, and that burn goes as 1/fps:

    R_streak_burn = (E[T] / stream_fps) x V_closing

The ~$740 Tier-2 verdict flips across ~24 fps at the predicted decode range, so
`--stream-fps` is a load-bearing MEASUREMENT. `tripod_score.py` refuses to invent
it (the invented-30-fps defect, 2026-07-24). This script is the only thing that
produces it honestly: protocol §7.3, contract build_tab step `skr-07`.

WHY NOT JUST USE pi_capture.py's decode_loop_fps
------------------------------------------------
`pi_capture.py` is a RECORDER. Its loop pays a per-frame PNG write that the
flight loop never pays, and its `decode_loop_fps` is a short-session number taken
on a COLD SoC. Both differ from the flying seeker. This script:

  * writes NO frames (measures the flight loop, not the recorder),
  * runs long enough for the Pi 5 to reach THERMAL STEADY STATE and publishes the
    steady-state window only, with the cold warm-up window reported separately,
  * samples SoC temperature, `get_throttled` and the ARM clock throughout, so a
    number taken while the part was quietly downclocking cannot masquerade as a
    sustained rate.

THE DIRECTION OF EVERY ERROR HERE IS DELIBERATE
-----------------------------------------------
An OVERSTATED fps SHRINKS the streak burn and pushes a ~$740 purchase toward GO.
So every judgement call in this file is made in the pessimistic direction and the
tool fails CLOSED rather than publishing a flattering number:

  1. BOUNDED, like pi_capture (2026-07-26). What the timer brackets is the decode
     call alone; the wait for the next frame is outside it. So 1/median(decode_dt)
     is decode THROUGHPUT — an UPPER BOUND on a cadence, not a cadence. Frames
     cannot be processed faster than they arrive, so the published figure is
     min(decode throughput, delivered frame cadence).

     BE PRECISE ABOUT WHAT THAT MIN IS DOING. The inter-arrival interval measured
     here spans one whole trip round the loop (wait for frame + decode it), so it
     is ALWAYS >= the decode interval, and the min() therefore ALWAYS selects the
     cadence. That is not a redundant guard — it means the published number is the
     ACHIEVED END-TO-END DETECTION CADENCE, which is precisely the rate the gate's
     burn model wants, and never the decode-only figure that would flatter it.
     `compute_headroom_x` = throughput / cadence then says WHERE the loop's time
     goes: >>1 means it is mostly waiting on the camera (camera-bound), ~1 means
     decode dominates (compute-bound). At the inherited 30 fps cap this rig read
     3.9x (camera-bound); uncapped it read 1.21x (compute-bound).
  2. The published gate figure is the STEADY-STATE window, never the whole run.
  3. `fps_p10_slow_tail` (worst-decile) is reported beside the median as the
     honest input for a purchase gate on a part that throttles.
  4. NO TAG IN VIEW => NOT GATE-ELIGIBLE. A scene with no tag understates decode
     cost (the detector finds no candidate quads to refine), which overstates fps
     — the flattering direction. Publishing that as the gate divisor is exactly
     the class of silent failure this project keeps retracting, so it is refused
     unless --allow-no-tag is passed BY NAME, and even then it is stamped
     PROVISIONAL and gate_eligible=false.
  5. `--quad-decimate` is MANDATORY. qd buys range and costs fps and the burn
     model goes as 1/fps, so quoting a qd=2.0 rate against a qd=1.0 envelope is
     the flattering-direction error the gate cannot afford (protocol §4.2b).
     There is no default because a guessed decimate silently rescales the answer.
  6. A run that never reaches thermal steady state is refused: a still-rising SoC
     has not shown you its sustained rate yet.

USAGE
-----
    # on the Pi, inside .venv-pi
    scripts/seeker/pi_fps_soak.py --mode apriltag --quad-decimate 2.0 \
        --duration-s 900 --out runs/soak_qd2

    scripts/seeker/pi_fps_soak.py --mode yolo \
        --weights scripts/seeker/weights/nn_tier/n-mono.onnx \
        --duration-s 900 --out runs/soak_yolo

    scripts/seeker/pi_fps_soak.py --self-test      # no camera, no soak, exits 0/1

OUTPUT
------
    <out>/soak.json     the full record (per-sample thermals, both windows)
    <out>/thermals.csv  t_s, temp_c, throttled_hex, arm_hz
    <out>/verdict.txt   the human read, and what may/may not be quoted
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
import time

# --- constants -------------------------------------------------------------

# The soak must be long enough for a Pi 5 under an active cooler to stop
# climbing. 15 min is the protocol default; below MIN_DURATION_S a run cannot
# be gate-eligible no matter what it measures.
MIN_DURATION_S = 600.0
DEFAULT_DURATION_S = 900.0
DEFAULT_WARMUP_S = 300.0

# Steady state = the mean SoC temperature over the second half of the steady
# window is no more than this much hotter than over the first half.
STEADY_TEMP_DRIFT_C = 1.5

# Fraction of steady-window frames that must contain a decoded tag for the
# AprilTag number to be gate-eligible. See rule 4 in the header.
DEFAULT_MIN_TAG_FRAC = 0.50

THERMAL_SAMPLE_S = 2.0

# The deployed seeker weights (flight/deploy/seeker_loop.py:1636). NOT
# drone_finetuned_quad_v2 — that is the blind-sim model; benching it would
# certify a model the vehicle does not fly (audit 2026-07-25).
DEFAULT_YOLO_WEIGHTS = "scripts/seeker/weights/nn_tier/n-mono.onnx"


# --- small helpers ---------------------------------------------------------

def _median(xs):
    return statistics.median(xs) if xs else None


def _pctl(xs, q):
    """q-th percentile (0..1) by nearest rank; xs need not be sorted."""
    if not xs:
        return None
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def _read_temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as fh:
            return int(fh.read().strip()) / 1000.0
    except Exception:
        return None


def _vcgencmd(arg):
    try:
        out = subprocess.run(["vcgencmd"] + arg.split(), capture_output=True,
                             text=True, timeout=5)
        return out.stdout.strip() if out.returncode == 0 else None
    except Exception:
        return None


def _read_throttled():
    """`throttled=0x...` -> the hex string, or None. Non-zero means the SoC
    under-volted / capped / throttled at some point; bit 0-3 are 'now'."""
    s = _vcgencmd("get_throttled")
    if s and "=" in s:
        return s.split("=", 1)[1].strip()
    return None


def _read_arm_hz():
    s = _vcgencmd("measure_clock arm")
    if s and "=" in s:
        try:
            return int(s.split("=", 1)[1])
        except ValueError:
            return None
    return None


def _read_governor():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor") as fh:
            return fh.read().strip()
    except Exception:
        return None


def _pi_model():
    try:
        with open("/proc/device-tree/model") as fh:
            return fh.read().strip("\x00").strip()
    except Exception:
        return None


def _throttle_flags(hexstr):
    """Decode the get_throttled bitmask into plain English."""
    if not hexstr:
        return []
    try:
        v = int(hexstr, 16)
    except ValueError:
        return []
    bits = [
        (0, "under-voltage NOW"), (1, "arm frequency capped NOW"),
        (2, "currently throttled"), (3, "soft temperature limit NOW"),
        (16, "under-voltage occurred"), (17, "arm frequency cap occurred"),
        (18, "throttling occurred"), (19, "soft temperature limit occurred"),
    ]
    return [name for bit, name in bits if v & (1 << bit)]


# --- the soak --------------------------------------------------------------

class Thermals:
    """Samples SoC temp / throttle / clock on a wall-clock cadence."""

    def __init__(self):
        self.rows = []          # (t_s, temp_c, throttled_hex, arm_hz)
        self._next = 0.0

    def maybe_sample(self, t_s):
        if t_s < self._next:
            return
        self._next = t_s + THERMAL_SAMPLE_S
        self.rows.append((round(t_s, 2), _read_temp_c(), _read_throttled(),
                          _read_arm_hz()))

    def window(self, t0, t1):
        return [r for r in self.rows if r[0] is not None and t0 <= r[0] <= t1]

    def steady_check(self, t0, t1):
        """Did the SoC stop climbing? Returns (is_steady, drift_c, note)."""
        rows = [r for r in self.window(t0, t1) if r[1] is not None]
        if len(rows) < 6:
            return False, None, ("too few thermal samples to demonstrate steady "
                                 "state")
        mid = t0 + (t1 - t0) / 2.0
        first = [r[1] for r in rows if r[0] <= mid]
        second = [r[1] for r in rows if r[0] > mid]
        if not first or not second:
            return False, None, "steady window too short to split"
        drift = statistics.fmean(second) - statistics.fmean(first)
        if drift > STEADY_TEMP_DRIFT_C:
            return False, drift, (
                f"SoC still climbing: +{drift:.2f} C across the steady window "
                f"(limit {STEADY_TEMP_DRIFT_C} C) — this run has NOT shown its "
                "sustained rate; soak longer")
        return True, drift, f"steady (drift {drift:+.2f} C)"


def _open_camera(width, height, exposure_us, target_fps=None):
    """picamera2, mono, FIXED exposure. Deliberately the SAME configuration call
    as pi_capture.py:487 — no explicit format (the OV9281 may present raw mono or
    an ISP-upsampled frame depending on driver; asking for R8 on the main stream
    is rejected outright), normalized afterwards by pi_capture's own _to_gray.
    Reusing that function rather than writing a second one keeps the bench and
    the recorder on ONE convention.

    AE/AGC are off with a fixed exposure: an auto-exposure loop would vary the
    frame interval and contaminate the delivered-cadence measurement.

    Returns (picam2, applied_exposure_us)."""
    from picamera2 import Picamera2   # noqa: WPS433 (lazy: Pi-only dep)

    picam2 = Picamera2()
    controls = {"AeEnable": False, "ExposureTime": int(exposure_us),
                "AnalogueGain": 1.0}
    # FrameDurationLimits — the CAP, and it is the whole ball game (2026-07-26).
    # Left alone, picamera2 picks a sensor mode / frame duration that delivered
    # exactly 30.0 fps on this rig while the decode itself ran at ~117 fps: the
    # seeker was CAMERA-CAPPED, not compute-bound, and the money gate's burn goes
    # as 1/fps. The OV9281 advertises 143 fps at 1280x800. So the cap is made an
    # EXPLICIT, RECORDED parameter rather than an inherited default -- the same
    # treatment quad_decimate got (pi_capture.py:195) and for the same reason: an
    # unrecorded default silently sets a number the $740 verdict turns on.
    if target_fps:
        dur = int(round(1e6 / float(target_fps)))
        if dur < exposure_us:
            raise SystemExit(
                f"[pi_fps_soak] --target-fps {target_fps} needs a {dur} us frame "
                f"duration, shorter than the {exposure_us} us exposure. Exposure "
                "is a hard floor on frame duration; lower it or lower the target.")
        controls["FrameDurationLimits"] = (dur, dur)
    cfg = picam2.create_video_configuration(main={"size": (width, height)},
                                            controls=controls)
    picam2.configure(cfg)
    picam2.start()
    picam2.set_controls(controls)
    time.sleep(1.0)                      # let the controls actually apply
    md = picam2.capture_metadata()
    return picam2, md.get("ExposureTime")


def run_soak(args):
    model = _pi_model()
    t_start = time.monotonic()

    # ---- build the workload ------------------------------------------------
    if args.mode == "apriltag":
        from pyapriltags import Detector
        det = Detector(families="tag36h11", nthreads=args.nthreads,
                       quad_decimate=args.quad_decimate)

        def work(frame):
            return len(det.detect(frame, estimate_tag_pose=False))
    else:
        import cv2
        import numpy as np
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = args.nthreads
        sess = ort.InferenceSession(args.weights, so,
                                    providers=["CPUExecutionProvider"])
        iname = sess.get_inputs()[0].name
        sz = args.imgsz

        def work(frame):
            img = cv2.resize(frame, (sz, sz))
            # mono sensor -> the exported model takes 3 channels
            blob = np.repeat(img[None, :, :], 3, axis=0)[None].astype("float32")
            blob /= 255.0
            sess.run(None, {iname: blob})
            return 0

    # Reuse the recorder's greyscale normalisation — ONE convention, not two.
    import cv2 as _cv2
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from pi_capture import _to_gray

    picam2, applied_exp = _open_camera(args.width, args.height,
                                       args.exposure_us, args.target_fps)

    therm = Thermals()
    samples = []        # (t_s, decode_dt, arrival_dt, n_tags)
    prev_arrival = None
    n_frames = 0

    try:
        while True:
            t_s = time.monotonic() - t_start
            if t_s >= args.duration_s:
                break
            therm.maybe_sample(t_s)

            request = picam2.capture_request()
            try:
                img = request.make_array("main")
            finally:
                request.release()
            t_arr = time.monotonic()
            arrival_dt = (t_arr - prev_arrival) if prev_arrival else None
            prev_arrival = t_arr

            # The grey conversion is INSIDE the timed region: the flight loop
            # pays it too, so excluding it would overstate fps — the flattering
            # direction for the gate.
            t0 = time.monotonic()
            n_tags = work(_to_gray(_cv2, img))
            decode_dt = time.monotonic() - t0

            samples.append((t_s, decode_dt, arrival_dt, n_tags))
            n_frames += 1
    finally:
        try:
            picam2.stop()
            picam2.close()
        except Exception:
            pass

    elapsed = time.monotonic() - t_start
    return _analyse(args, samples, therm, elapsed, model, applied_exp, n_frames)


def _window_stats(rows):
    """rows: (t_s, decode_dt, arrival_dt, n_tags) -> the bounded fps figures."""
    dec = [r[1] for r in rows if r[1] and r[1] > 0]
    arr = [r[2] for r in rows if r[2] and r[2] > 0]
    if not dec:
        return None

    dec_med = _median(dec)
    thr_fps = 1.0 / dec_med                       # UPPER BOUND, not a cadence
    arr_fps = (1.0 / _median(arr)) if arr else None

    # Rule 1: clamp throughput by the rate frames actually arrive.
    if arr_fps is not None:
        fps = min(thr_fps, arr_fps)
        src = (f"BOUNDED min(decode throughput {thr_fps:.2f}, delivered cadence "
               f"{arr_fps:.2f})")
    else:
        fps = thr_fps
        src = f"decode throughput {thr_fps:.2f} (UNBOUNDED — no arrival cadence)"

    # Worst-decile: the slow tail is what a purchase gate should be sized on.
    dec_p90 = _pctl(dec, 0.90)                    # slowest decile of decodes
    p10_fps = (1.0 / dec_p90) if dec_p90 else None
    if p10_fps is not None and arr_fps is not None:
        p10_fps = min(p10_fps, arr_fps)

    n_tag_frames = sum(1 for r in rows if r[3] and r[3] > 0)

    # CLAMP-DOMINATED? (diagnostic, NOT a licence to skip the tag.)
    # When delivered cadence is far below decode throughput, min() is set by the
    # CAMERA, so adding a tag — which only makes decode slower — cannot move the
    # published number until it slows decode past the delivered rate. That bounds
    # how much the tagless caveat can possibly bite, and the headroom ratio says
    # by how much. It does NOT make a tagless run gate-eligible: it is reported
    # so the caveat can be reasoned about with a number instead of a shrug.
    headroom = (thr_fps / arr_fps) if (arr_fps and arr_fps > 0) else None
    clamp_dominated = bool(headroom and headroom >= 1.5)

    return {
        "n_frames": len(rows),
        "fps_published": round(fps, 3),
        "fps_source": src,
        "fps_p10_slow_tail": round(p10_fps, 3) if p10_fps else None,
        "decode_throughput_fps": round(thr_fps, 3),
        "delivered_cadence_fps": round(arr_fps, 3) if arr_fps else None,
        "compute_headroom_x": round(headroom, 2) if headroom else None,
        "clamp_dominated": clamp_dominated,
        "bound_by": ("camera delivery" if clamp_dominated else "compute/decode"),
        "decode_ms_median": round(dec_med * 1000.0, 3),
        "decode_ms_p90": round(dec_p90 * 1000.0, 3) if dec_p90 else None,
        "frames_with_tag": n_tag_frames,
        "tag_frac": round(n_tag_frames / len(rows), 4) if rows else 0.0,
    }


def _tag_timeline(samples, bucket_s=30.0):
    """Per-bucket tag-detection fraction, so a DROPOUT can be located in time.

    Added 2026-07-26 after the qd=1.0 arm came back tag_frac=0.44 and the record
    could not say whether the tags vanished at minute 2 or minute 12 — the
    difference between 'the display died partway' and 'the detector is marginal',
    which are opposite diagnoses. An aggregate that cannot be localised is a
    number you can only guess about. Output-only: it changes no measured value.
    """
    if not samples:
        return []
    out, t_end = [], samples[-1][0]
    t = samples[0][0]
    while t < t_end:
        rows = [r for r in samples if t <= r[0] < t + bucket_s]
        if rows:
            hits = sum(1 for r in rows if r[3] and r[3] > 0)
            out.append({"t_s": round(t, 1), "n": len(rows),
                        "tag_frac": round(hits / len(rows), 3)})
        t += bucket_s
    return out


def _analyse(args, samples, therm, elapsed, model, applied_exp, n_frames):
    warm = [r for r in samples if r[0] < args.warmup_s]
    steady = [r for r in samples if r[0] >= args.warmup_s]

    warm_stats = _window_stats(warm)
    steady_stats = _window_stats(steady)

    is_steady, drift, steady_note = therm.steady_check(args.warmup_s, elapsed)

    temps = [r[1] for r in therm.rows if r[1] is not None]
    throttle_seen = sorted({r[2] for r in therm.rows if r[2] and r[2] != "0x0"})
    flags = sorted({f for h in throttle_seen for f in _throttle_flags(h)})

    # ---- gate eligibility: fail CLOSED ------------------------------------
    refusals = []
    if not model or "Raspberry Pi 5" not in model:
        refusals.append(f"not a Raspberry Pi 5 (model={model!r})")
    if elapsed < MIN_DURATION_S:
        refusals.append(f"soak too short: {elapsed:.0f}s < {MIN_DURATION_S:.0f}s")
    if steady_stats is None:
        refusals.append("no frames in the steady-state window")
    if not is_steady:
        refusals.append(f"thermal steady state not demonstrated — {steady_note}")
    if args.mode == "apriltag" and steady_stats is not None:
        if steady_stats["tag_frac"] < args.min_tag_frac:
            msg = (f"NO TAG IN VIEW for most of the soak "
                   f"(tag_frac={steady_stats['tag_frac']:.2f} < "
                   f"{args.min_tag_frac:.2f}): with no candidate quads to refine, "
                   "decode cost is UNDERSTATED and fps OVERSTATED — the "
                   "flattering direction for the ~$740 gate")
            if args.allow_no_tag:
                refusals.append("PROVISIONAL, --allow-no-tag was passed: " + msg)
            else:
                refusals.append(msg)

    rec = {
        "tool": "pi_fps_soak.py",
        "contract_step": "skr-07",
        "protocol": "docs/tripod_test_protocol.md §7.3",
        "mode": args.mode,
        "gate_eligible": not refusals,
        "refusals": refusals,
        # WHICH GATE THIS NUMBER MAY FEED (2026-07-26). Only the AprilTag arm may
        # supply tripod_score's --stream-fps: curve (a)'s burn model is the TAG
        # pipeline's cadence, and handing it a CPU-YOLO rate would be the
        # wrong-quantity error one layer up (the markerless rate gates the
        # deferred Hailo decision, not the ~$740 interceptor order). The tool used
        # to stamp both arms identically, which is exactly how a plausible number
        # ends up in the wrong slot.
        "may_be_quoted_as_stream_fps": (
            steady_stats["fps_published"]
            if (not refusals and steady_stats and args.mode == "apriltag")
            else None),
        "gates": ("curve (a) / the ~$740 Tier-2 order (tripod_score --stream-fps)"
                  if args.mode == "apriltag"
                  else "the DEFERRED Hailo/markerless phase ONLY — never "
                       "tripod_score --stream-fps"),
        "host": {
            "model": model, "governor": _read_governor(),
            "python": sys.version.split()[0], "nthreads": args.nthreads,
        },
        "capture": {
            "source": "picamera2", "width": args.width, "height": args.height,
            "exposure_us_requested": args.exposure_us,
            "exposure_us_applied": applied_exp,
            "target_fps_requested": args.target_fps,
            "frame_duration_capped": args.target_fps is not None,
            "frames_written": 0,
            "note": ("NO frames are written: this measures the FLIGHT loop, not "
                     "the recorder (pi_capture.py pays a PNG write per frame "
                     "that the flight loop never pays)"),
        },
        "workload": ({"detector": "pyapriltags tag36h11",
                      "quad_decimate": args.quad_decimate}
                     if args.mode == "apriltag"
                     else {"onnx": args.weights, "imgsz": args.imgsz,
                           "provider": "CPUExecutionProvider"}),
        "soak": {
            "duration_s_requested": args.duration_s,
            "duration_s_elapsed": round(elapsed, 2),
            "warmup_s": args.warmup_s,
            "n_frames_total": n_frames,
            "temp_c_start": temps[0] if temps else None,
            "temp_c_max": max(temps) if temps else None,
            "temp_c_end": temps[-1] if temps else None,
            "steady_state": is_steady,
            "steady_drift_c": round(drift, 3) if drift is not None else None,
            "steady_note": steady_note,
            "throttled_values_seen": throttle_seen or ["0x0"],
            "throttle_flags": flags,
        },
        "warmup_window": warm_stats,
        "steady_window": steady_stats,
        "tag_timeline_30s": _tag_timeline(samples),
    }
    return rec, therm


def write_outputs(rec, therm, outdir):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "soak.json"), "w") as fh:
        json.dump(rec, fh, indent=2)
    with open(os.path.join(outdir, "thermals.csv"), "w") as fh:
        fh.write("t_s,temp_c,throttled_hex,arm_hz\n")
        for t_s, temp, thr, hz in therm.rows:
            fh.write(f"{t_s},{'' if temp is None else temp},"
                     f"{thr or ''},{'' if hz is None else hz}\n")

    sw = rec.get("steady_window") or {}
    lines = [
        f"skr-07 compute bench — mode={rec['mode']}",
        f"  host      : {rec['host']['model']}  gov={rec['host']['governor']}",
        f"  workload  : {json.dumps(rec['workload'])}",
        f"  soak      : {rec['soak']['duration_s_elapsed']:.0f}s, "
        f"temp {rec['soak']['temp_c_start']} -> {rec['soak']['temp_c_end']} C "
        f"(max {rec['soak']['temp_c_max']}), {rec['soak']['steady_note']}",
        f"  throttle  : {', '.join(rec['soak']['throttle_flags']) or 'none'} "
        f"(raw {rec['soak']['throttled_values_seen']})",
        "",
        f"  STEADY fps (published) : {sw.get('fps_published')}",
        f"    source               : {sw.get('fps_source')}",
        f"    p10 slow tail        : {sw.get('fps_p10_slow_tail')}",
        f"    decode ms med / p90  : {sw.get('decode_ms_median')} / "
        f"{sw.get('decode_ms_p90')}",
        f"    bound by             : {sw.get('bound_by')} "
        f"(compute headroom {sw.get('compute_headroom_x')}x)",
        f"    tag_frac             : {sw.get('tag_frac')}",
    ]
    warm = rec.get("warmup_window") or {}
    if warm:
        lines.append(f"  cold warm-up fps       : {warm.get('fps_published')} "
                     "(DIAGNOSTIC ONLY — never the gate number)")
    lines += ["", ("  GATE-ELIGIBLE: may be passed to tripod_score --stream-fps"
                   if rec["gate_eligible"] else
                   "  NOT GATE-ELIGIBLE — refusals:")]
    lines += [f"    - {r}" for r in rec["refusals"]]
    txt = "\n".join(lines) + "\n"
    with open(os.path.join(outdir, "verdict.txt"), "w") as fh:
        fh.write(txt)
    return txt


def self_test():
    """No camera, no soak. Exercises the analysis path — including that it
    REFUSES a short/cold/tagless run. Exits 0/1."""
    fails = []

    class A:
        mode = "apriltag"; quad_decimate = 2.0; warmup_s = 1.0; duration_s = 5.0
        width = 1280; height = 800; exposure_us = 1000; nthreads = 4
        min_tag_frac = 0.5; allow_no_tag = False; weights = ""; imgsz = 640
        target_fps = None

    # a short, tagless run must be refused on BOTH counts
    samples = [(i * 0.05, 0.02, 0.03, 0) for i in range(200)]
    therm = Thermals()
    for i in range(20):
        therm.rows.append((i * 0.5, 50.0 + i * 0.01, "0x0", 2400000000))
    rec, _ = _analyse(A(), samples, therm, 5.0, "Raspberry Pi 5 Model B", 994, 200)
    if rec["gate_eligible"]:
        fails.append("a 5 s tagless soak was accepted as gate-eligible")
    if not any("too short" in r for r in rec["refusals"]):
        fails.append("short soak not refused")
    if not any("NO TAG" in r for r in rec["refusals"]):
        fails.append("tagless soak not refused")
    if rec["may_be_quoted_as_stream_fps"] is not None:
        fails.append("a refused run still published a quotable fps")

    # the bound must bite: throughput 50 fps clamped by a 20 fps arrival cadence
    st = _window_stats([(1.0, 0.02, 0.05, 1)] * 50)
    if st["fps_published"] > 20.001:
        fails.append(f"clamp failed: published {st['fps_published']} > arrival 20")

    # a still-climbing SoC must be refused
    t2 = Thermals()
    for i in range(20):
        t2.rows.append((i * 0.5, 50.0 + i * 0.5, "0x0", 2400000000))
    ok, drift, note = t2.steady_check(0.0, 10.0)
    if ok:
        fails.append("a climbing SoC passed the steady-state check")

    for f in fails:
        print(f"FAIL: {f}")
    print(f"self-test: {'PASS' if not fails else f'{len(fails)} FAILURE(S)'}")
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("apriltag", "yolo"), default="apriltag")
    ap.add_argument("--quad-decimate", type=float, default=None,
                    help="MANDATORY for --mode apriltag. No default on purpose: "
                         "qd rescales both range and fps (protocol §4.2b).")
    ap.add_argument("--weights", default=DEFAULT_YOLO_WEIGHTS,
                    help="ONNX weights for --mode yolo (default: the DEPLOYED "
                         "n-mono.onnx, not the blind-sim quad_v2)")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    ap.add_argument("--warmup-s", type=float, default=DEFAULT_WARMUP_S)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--exposure-us", type=int, default=1000)
    ap.add_argument("--target-fps", type=float, default=None,
                    help="set FrameDurationLimits to uncap the sensor. Left "
                         "unset, picamera2's inherited default delivered 30.0 "
                         "fps on this rig while decode ran at ~117 fps — i.e. "
                         "camera-capped, not compute-bound. Recorded either way.")
    ap.add_argument("--nthreads", type=int, default=4)
    ap.add_argument("--min-tag-frac", type=float, default=DEFAULT_MIN_TAG_FRAC)
    ap.add_argument("--allow-no-tag", action="store_true",
                    help="run without a tag in view; result is stamped "
                         "PROVISIONAL and stays gate_eligible=false")
    ap.add_argument("--out", default=None)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.mode == "apriltag" and args.quad_decimate is None:
        ap.error("--quad-decimate is MANDATORY for --mode apriltag: a guessed "
                 "decimate silently rescales the published fps, and the gate "
                 "goes as 1/fps")
    if not args.out:
        ap.error("--out is required")
    if args.warmup_s >= args.duration_s:
        ap.error("--warmup-s must be less than --duration-s")

    rec, therm = run_soak(args)
    print(write_outputs(rec, therm, args.out))
    return 0 if rec["gate_eligible"] else 2


if __name__ == "__main__":
    sys.exit(main())
