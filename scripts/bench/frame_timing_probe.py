#!/usr/bin/env python3
"""Bench probe: how well the Pi timestamps camera frames.

Runs ON THE Pi (Raspberry Pi 5 + innomaker/Arducam OV9281 mono global-shutter
camera via picamera2). Implements docs/bench_specs/pi_frame_timing.md.

WHAT THIS MEASURES, per frame, using the camera's hardware `SensorTimestamp`
(from picamera2 request metadata, nanoseconds) as the reference instant a
frame was exposed:
  - frame-to-frame interval (from consecutive SensorTimestamp values)
  - "capture -> Python" latency: SensorTimestamp vs. time.monotonic_ns() /
    time.clock_gettime_ns(CLOCK_BOOTTIME) read the instant the frame array is
    available in this process
  - "capture -> after AprilTag decode" latency: same, but after also running
    the repo's real AprilTag detector (scripts/apriltag_detector.Detector,
    the pupil_apriltags/pyapriltags shim every flight decode site uses -- see
    scripts/seeker/pi_capture.py for the production TAG_FAMILY /
    quad_decimate this mirrors) on the frame
  - which of MONOTONIC / BOOTTIME SensorTimestamp sits closer to (measured,
    not assumed -- the spec's own instruction)
  - dropped frames, INFERRED from gaps in the SensorTimestamp sequence
    (picamera2's blocking capture_request() does not expose a frame-sequence
    counter directly, so this is an estimate, not a hardware drop counter --
    said plainly, not silently)

WHAT THIS DOES NOT MEASURE (see spec, and repeated in the printed summary):
the Pi <-> flight-controller clock offset. That needs the FC's clock wired
in over UART/telemetry and is a separate, later bench.

Usage (on the Pi, inside the pre-provisioned venv that already has picamera2
+ pyapriltags -- see scripts/pi_setup/requirements-pi.txt):
    .venv-pi/bin/python3 scripts/bench/frame_timing_probe.py \\
        --duration 20 --out-csv runs/frame_timing/frame_timing_probe.csv
"""

import argparse
import csv
import json
import statistics
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

TAG_FAMILY = "tag36h11"  # matches scripts/seeker/pi_capture.py TAG_FAMILY
QUAD_DECIMATE = 2.0      # matches pi_capture.py DEFAULT_QUAD_DECIMATE (library default)

_DEL_NEUTRALIZED = False


def _neutralize_detector_del(Detector):
    """Same fix as scripts/seeker/pi_capture.py: the native pupil_apriltags/
    pyapriltags Detector.__del__ can SIGSEGV/SIGABRT at interpreter shutdown.
    We build exactly one Detector here, but that one destructor still runs at
    process exit, so neutralize it defensively -- costs nothing, avoids losing
    a written CSV to a crash on the way out."""
    global _DEL_NEUTRALIZED
    if _DEL_NEUTRALIZED:
        return
    try:
        Detector.__del__ = lambda self: None
    except (AttributeError, TypeError):
        pass
    _DEL_NEUTRALIZED = True


def _to_gray(np, cv2, img):
    """OV9281 is mono; normalize whatever channel layout picamera2 hands back
    (XBGR8888 / RGB888 / already-2D) to a single 8-bit plane for the detector."""
    if img is None:
        return None
    if img.ndim == 2:
        return img
    if img.ndim == 3 and img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    if img.ndim == 3 and img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img[..., 0]


def pctl(sorted_vals, p):
    """Nearest-rank percentile, no interpolation dependency (numpy optional)."""
    if not sorted_vals:
        return None
    k = max(0, min(len(sorted_vals) - 1, int(round(p / 100.0 * (len(sorted_vals) - 1)))))
    return sorted_vals[k]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=20.0, help="capture duration, seconds")
    ap.add_argument("--exposure-us", type=int, default=1000, help="fixed exposure, microseconds")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--gain", type=float, default=1.0, help="AnalogueGain")
    ap.add_argument("--target-fps", type=float, default=143.0,
                     help="FrameDurationLimits cap, default = the sensor's own max at "
                          "1280x800 (see scripts/seeker/probe_flight_frame_rate.py / ADR-0090: "
                          "with NO FrameDurationLimits picamera2 silently defaults to ~30 fps "
                          "on this exact rig -- 'a default nobody chose'. This flag exists so "
                          "'as fast as the mode allows' is an explicit, recorded choice, not "
                          "another unchosen default.")
    ap.add_argument("--out-csv", type=Path,
                     default=REPO_ROOT / "runs" / "frame_timing" / "frame_timing_probe.csv")
    ap.add_argument("--out-summary-json", type=Path, default=None,
                     help="default: alongside --out-csv, same stem + _summary.json")
    args = ap.parse_args()

    if args.out_summary_json is None:
        args.out_summary_json = args.out_csv.with_name(args.out_csv.stem + "_summary.json")
    args.out_csv.parent.mkdir(parents=True, exist_ok=True)

    import numpy as np
    import cv2
    from picamera2 import Picamera2
    from apriltag_detector import Detector, DETECTOR_BACKEND  # noqa: E402 (repo shim)

    detector = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=QUAD_DECIMATE)
    _neutralize_detector_del(Detector)

    picam2 = Picamera2()
    controls = {"AeEnable": False, "ExposureTime": int(args.exposure_us),
                "AnalogueGain": float(args.gain)}
    # "As fast as the mode allows" is NOT the picamera2 default (ADR-0090,
    # measured on this exact rig: no FrameDurationLimits -> silently 30.048
    # fps while the sensor does 143 and AprilTag decode ~112 -- "a default
    # nobody chose"). So pin FrameDurationLimits explicitly to --target-fps
    # (default 143, the sensor's own ceiling at 1280x800) -- an explicit,
    # recorded choice, not another unchosen default. Exposure is a hard floor
    # on frame duration (same guard as flight/deploy/seeker_loop.py).
    dur_us = int(round(1e6 / args.target_fps))
    if dur_us < args.exposure_us:
        print(f"[frame_timing_probe] --target-fps {args.target_fps} needs a {dur_us} us "
              f"frame duration, shorter than the {args.exposure_us} us exposure; "
              f"clamping target duration to the exposure floor.", file=sys.stderr)
        dur_us = args.exposure_us
    controls["FrameDurationLimits"] = (dur_us, dur_us)
    cfg = picam2.create_video_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"}, controls=controls)
    picam2.configure(cfg)
    picam2.start()
    picam2.set_controls(controls)
    print(f"[frame_timing_probe] FrameDurationLimits pinned to {controls['FrameDurationLimits']} "
          f"(target {args.target_fps:.1f} fps)")

    rows = []
    n = 0
    boot_minus_mono_samples = []  # to characterize the two clocks' relative offset
    t_deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < t_deadline:
            request = picam2.capture_request()
            try:
                img = request.make_array("main")
                md = request.get_metadata()
            finally:
                request.release()
            t_mono_capture = time.monotonic_ns()
            t_boot_capture = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            boot_minus_mono_samples.append(t_boot_capture - t_mono_capture)
            sensor_ts = md.get("SensorTimestamp")
            exposure_applied = md.get("ExposureTime")

            gray = _to_gray(np, cv2, img)
            detections = detector.detect(gray, estimate_tag_pose=False)

            t_mono_detect = time.monotonic_ns()
            t_boot_detect = time.clock_gettime_ns(time.CLOCK_BOOTTIME)

            rows.append({
                "frame_idx": n,
                "sensor_timestamp_ns": sensor_ts,
                "monotonic_ns_capture": t_mono_capture,
                "boottime_ns_capture": t_boot_capture,
                "monotonic_ns_after_detect": t_mono_detect,
                "boottime_ns_after_detect": t_boot_detect,
                "exposure_us_applied": exposure_applied,
                "n_tag_detections": len(detections),
            })
            n += 1
    finally:
        picam2.stop()
        picam2.close()  # release the camera device so another process/worker can open it

    if len(rows) < 3:
        print(f"[frame_timing_probe] only {len(rows)} frames captured -- refusing to "
              f"compute statistics on this few (no vacuous verdicts).", file=sys.stderr)
        sys.exit(1)

    with open(args.out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # ---- frame interval, from SensorTimestamp ----
    sensor_ts_list = [r["sensor_timestamp_ns"] for r in rows]
    have_sensor_ts = all(v is not None for v in sensor_ts_list)
    intervals = None
    if have_sensor_ts:
        intervals = [b - a for a, b in zip(sensor_ts_list, sensor_ts_list[1:])]
        intervals_sorted = sorted(intervals)
        interval_mean = statistics.mean(intervals)
        interval_std = statistics.pstdev(intervals)
        interval_max = max(intervals)
        interval_median = statistics.median(intervals)
    else:
        interval_mean = interval_std = interval_max = interval_median = None

    # ---- capture -> Python latency (both clock hypotheses) ----
    lat_mono = [r["monotonic_ns_capture"] - r["sensor_timestamp_ns"] for r in rows
                if r["sensor_timestamp_ns"] is not None]
    lat_boot = [r["boottime_ns_capture"] - r["sensor_timestamp_ns"] for r in rows
                if r["sensor_timestamp_ns"] is not None]
    lat_mono_sorted, lat_boot_sorted = sorted(lat_mono), sorted(lat_boot)

    # ---- capture -> after-detect latency ----
    lat_detect_mono = [r["monotonic_ns_after_detect"] - r["sensor_timestamp_ns"] for r in rows
                        if r["sensor_timestamp_ns"] is not None]
    lat_detect_mono_sorted = sorted(lat_detect_mono)
    detect_only_ns = [r["monotonic_ns_after_detect"] - r["monotonic_ns_capture"] for r in rows]

    # ---- which clock is SensorTimestamp closer to? ----
    # boot_minus_mono_samples characterizes the BOOTTIME-MONOTONIC offset on
    # THIS Pi at the times we sampled it; if that offset is itself ~0 (no
    # suspend since boot), the two latency hypotheses (lat_mono, lat_boot)
    # will be statistically indistinguishable and we say so rather than guess.
    boot_mono_offset_mean = statistics.mean(boot_minus_mono_samples)
    boot_mono_offset_std = statistics.pstdev(boot_minus_mono_samples)
    mean_lat_mono = statistics.mean(lat_mono)
    mean_lat_boot = statistics.mean(lat_boot)
    std_lat_mono = statistics.pstdev(lat_mono)
    std_lat_boot = statistics.pstdev(lat_boot)

    clock_distinguishable = abs(boot_mono_offset_std) > 1000 or abs(boot_mono_offset_mean) > 1_000_000
    if not clock_distinguishable and abs(mean_lat_mono - mean_lat_boot) < max(std_lat_mono, std_lat_boot):
        clock_verdict = ("INDETERMINATE -- BOOTTIME and MONOTONIC differ by "
                          f"{boot_mono_offset_mean:.0f} ns (std {boot_mono_offset_std:.0f} ns) on this "
                          "Pi (it has not suspended since boot), so the two latency hypotheses "
                          "cannot be told apart by this measurement.")
    else:
        closer = "MONOTONIC" if mean_lat_mono >= 0 and mean_lat_mono < mean_lat_boot else "BOOTTIME"
        clock_verdict = (f"SensorTimestamp is closer to {closer} "
                          f"(mean latency-if-MONOTONIC={mean_lat_mono/1e6:.3f} ms, "
                          f"mean latency-if-BOOTTIME={mean_lat_boot/1e6:.3f} ms)")

    # ---- dropped frames, inferred from interval gaps ----
    dropped_estimate = 0
    gap_events = 0
    if intervals:
        for dt in intervals:
            if dt > 1.5 * interval_median:
                gap_events += 1
                dropped_estimate += max(0, round(dt / interval_median) - 1)

    summary = {
        "n_frames": len(rows),
        "duration_s_requested": args.duration,
        "width": args.width, "height": args.height,
        "exposure_us_requested": args.exposure_us,
        "exposure_us_applied_mean": statistics.mean(
            [r["exposure_us_applied"] for r in rows if r["exposure_us_applied"] is not None])
            if any(r["exposure_us_applied"] is not None for r in rows) else None,
        "detector_backend": DETECTOR_BACKEND,
        "tag_family": TAG_FAMILY, "quad_decimate": QUAD_DECIMATE,
        "frame_interval_ns": {"mean": interval_mean, "std": interval_std,
                               "median": interval_median, "max": interval_max},
        "frame_interval_ms": {"mean": interval_mean / 1e6 if interval_mean else None,
                               "std": interval_std / 1e6 if interval_std else None,
                               "max": interval_max / 1e6 if interval_max else None},
        "implied_fps_mean": (1e9 / interval_mean) if interval_mean else None,
        "capture_to_python_latency_ms": {
            "mean_if_monotonic": mean_lat_mono / 1e6, "std_if_monotonic": std_lat_mono / 1e6,
            "p99_if_monotonic": pctl(lat_mono_sorted, 99) / 1e6,
            "mean_if_boottime": mean_lat_boot / 1e6, "std_if_boottime": std_lat_boot / 1e6,
            "p99_if_boottime": pctl(lat_boot_sorted, 99) / 1e6,
        },
        "capture_to_after_detect_latency_ms": {
            "mean_if_monotonic": statistics.mean(lat_detect_mono) / 1e6,
            "std_if_monotonic": statistics.pstdev(lat_detect_mono) / 1e6,
            "p99_if_monotonic": pctl(lat_detect_mono_sorted, 99) / 1e6,
        },
        "detect_only_ms": {"mean": statistics.mean(detect_only_ns) / 1e6,
                            "std": statistics.pstdev(detect_only_ns) / 1e6,
                            "max": max(detect_only_ns) / 1e6},
        "boottime_minus_monotonic_ns": {"mean": boot_mono_offset_mean, "std": boot_mono_offset_std},
        "clock_verdict": clock_verdict,
        "dropped_frames_estimate": dropped_estimate,
        "gap_events_gt_1p5x_median": gap_events,
        "n_tag_detections_total": sum(r["n_tag_detections"] for r in rows),
        "does_not_measure": ("Pi <-> flight-controller clock offset -- needs the FC's clock "
                              "wired in over UART/telemetry, a separate later bench."),
    }

    with open(args.out_summary_json, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"[frame_timing_probe] backend={DETECTOR_BACKEND} family={TAG_FAMILY} "
          f"quad_decimate={QUAD_DECIMATE}")
    print(f"[frame_timing_probe] frames captured: {len(rows)} over "
          f"{(rows[-1]['monotonic_ns_capture'] - rows[0]['monotonic_ns_capture'])/1e9:.2f} s wall")
    print(f"[frame_timing_probe] applied exposure (mean, us): "
          f"{summary['exposure_us_applied_mean']}")
    print(f"[frame_timing_probe] frame interval (ms): mean={summary['frame_interval_ms']['mean']:.4f} "
          f"std={summary['frame_interval_ms']['std']:.4f} max={summary['frame_interval_ms']['max']:.4f} "
          f"(implied fps mean={summary['implied_fps_mean']:.2f})")
    print(f"[frame_timing_probe] capture->Python latency (ms), MONOTONIC hyp: "
          f"mean={summary['capture_to_python_latency_ms']['mean_if_monotonic']:.4f} "
          f"std={summary['capture_to_python_latency_ms']['std_if_monotonic']:.4f} "
          f"p99={summary['capture_to_python_latency_ms']['p99_if_monotonic']:.4f}")
    print(f"[frame_timing_probe] capture->Python latency (ms), BOOTTIME hyp: "
          f"mean={summary['capture_to_python_latency_ms']['mean_if_boottime']:.4f} "
          f"std={summary['capture_to_python_latency_ms']['std_if_boottime']:.4f} "
          f"p99={summary['capture_to_python_latency_ms']['p99_if_boottime']:.4f}")
    print(f"[frame_timing_probe] capture->after-detect latency (ms), MONOTONIC hyp: "
          f"mean={summary['capture_to_after_detect_latency_ms']['mean_if_monotonic']:.4f} "
          f"std={summary['capture_to_after_detect_latency_ms']['std_if_monotonic']:.4f} "
          f"p99={summary['capture_to_after_detect_latency_ms']['p99_if_monotonic']:.4f}")
    print(f"[frame_timing_probe] detect-only cost (ms): mean={summary['detect_only_ms']['mean']:.4f} "
          f"std={summary['detect_only_ms']['std']:.4f} max={summary['detect_only_ms']['max']:.4f}")
    print(f"[frame_timing_probe] BOOTTIME-MONOTONIC offset (ns): "
          f"mean={boot_mono_offset_mean:.0f} std={boot_mono_offset_std:.0f}")
    print(f"[frame_timing_probe] clock verdict: {clock_verdict}")
    print(f"[frame_timing_probe] dropped frames (inferred, gap>1.5x median interval): "
          f"{dropped_estimate} across {gap_events} gap events")
    print(f"[frame_timing_probe] tag detections total: {summary['n_tag_detections_total']} / "
          f"{len(rows)} frames")
    print(f"[frame_timing_probe] DOES NOT MEASURE: {summary['does_not_measure']}")
    print(f"[frame_timing_probe] wrote {args.out_csv}")
    print(f"[frame_timing_probe] wrote {args.out_summary_json}")

    # ---- verdict line the spec asks for ----
    p99_mono = summary["capture_to_python_latency_ms"]["p99_if_monotonic"]
    p99_boot = summary["capture_to_python_latency_ms"]["p99_if_boottime"]
    residual_ms = max(p99_mono, p99_boot)
    print(f"[frame_timing_probe] VERDICT: if guidance uses SensorTimestamp, the residual "
          f"timestamp error is {residual_ms:.2f} ms (p99 capture->Python latency, worse of the "
          f"two clock hypotheses; excludes the Pi<->FC offset, not measured here).")


if __name__ == "__main__":
    main()
