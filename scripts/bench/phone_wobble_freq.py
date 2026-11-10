#!/usr/bin/env python3
"""Turn a slow-motion phone clip of the hovering quad into a wobble frequency.

WHY: the flight controller's SD slot is dead and the Pi is down, so there is no
onboard recording. A slow-motion phone clip is the only instrument left. This
reads the clip, finds the aircraft in every frame, measures how its outline
TILTS from frame to frame, and reports the strongest rocking frequency in REAL
hertz (not slowed-down hertz).

HOW TO FILM (this decides whether the number is any good):
  * Stand directly BEHIND or IN FRONT of the aircraft, phone level with it, so
    rocking shows up as the whole outline tilting left-right.
  * Plain background behind it: open sky is best. Fill a good part of the frame.
  * Hold the phone as still as you can (brace on something). 10 s is plenty.
  * Send the ORIGINAL file. A clip "shared" from the Photos app is often
    re-rendered so only the middle is slow -- then the time scale changes
    mid-clip and no single number is right. If in doubt, trim to the slow part.

THE ONE NUMBER YOU MUST GIVE: --capture-fps, the rate the phone really recorded
at (iPhone slo-mo: 240, or 120 on some settings). The file may be labelled
240 fps or 30 fps depending on how it was exported; the tool never guesses.
Real time per frame is always 1 / capture-fps.

HOW TO READ THE RESULT (same rule of thumb as hover_wobble_analysis.py):
  below ~8 Hz   attitude loop too stiff for the rate loop under it, or
                something mechanical swinging
  above ~12 Hz  rate loop gain too high, or too much delay (filters, PWM)
  in between    ambiguous -- change one thing and re-measure

The tilt SIZE in degrees is only meaningful if you filmed along the axis it
rocks about; the tool always says so. The FREQUENCY does not depend on that.
Slow drift of a hand-held phone lands below ~2 Hz; treat a peak down there
with suspicion.

FAIL-CLOSED (exit 2, with the reason, never a default number): clip shorter
than 3 s of real time; aircraft not found in enough frames; no peak that
clearly stands above the noise.

    .venv/bin/python scripts/bench/phone_wobble_freq.py clip.mov --capture-fps 240
    .venv/bin/python scripts/bench/phone_wobble_freq.py clip.mov --capture-fps 240 --roi 400,200,900,600
    .venv/bin/python scripts/bench/phone_wobble_freq.py clip.mov --capture-fps 240 --bright
    .venv/bin/python scripts/bench/phone_wobble_freq.py --self-test
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile

import cv2
import numpy as np

MIN_REAL_SECONDS = 3.0
MIN_VALID_FRACTION = 0.8      # aircraft must be found in this share of frames
MIN_ELONGATION = 1.3          # outline must be longer than tall for a tilt angle
MIN_PEAK_TO_MEDIAN = 8.0      # spectral peak vs the median of the band
MIN_TILT_DEG = 0.10           # below this a "peak" is pixel/compression noise
BAND_LO_HZ = 1.0
BAND_HI_HZ = 60.0
WORK_WIDTH = 480              # frames are shrunk to this width for speed


def measure_frame(gray, bright: bool):
    """Outline of the aircraft in one frame -> (tilt_deg, cx, cy) or None."""
    g = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if not bright:                      # default: dark aircraft, light background
        mask = cv2.bitwise_not(mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = stats[i, cv2.CC_STAT_AREA]
    frac = area / mask.size
    if frac < 0.0005 or frac > 0.4:     # nothing there, or it grabbed the background
        return None
    m = cv2.moments((labels == i).astype(np.uint8), binaryImage=True)
    if m["m00"] <= 0:
        return None
    mu20, mu02, mu11 = m["mu20"], m["mu02"], m["mu11"]
    common = math.sqrt(4 * mu11 * mu11 + (mu20 - mu02) ** 2)
    major, minor = mu20 + mu02 + common, mu20 + mu02 - common
    if minor <= 0 or math.sqrt(major / minor) < MIN_ELONGATION:
        return None
    tilt = 0.5 * math.degrees(math.atan2(2 * mu11, mu20 - mu02))
    return tilt, m["m10"] / m["m00"], m["m01"] / m["m00"]


def spectrum_peak(x, fs):
    """Strongest sinusoid in the band: (freq_hz, amplitude, peak-to-median)."""
    n = len(x)
    t = np.arange(n) / fs
    x = x - np.polyval(np.polyfit(t, x, 2), t)
    win = np.hanning(n)
    spec = np.abs(np.fft.rfft(x * win)) * 2.0 / win.sum()
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    sel = (freqs >= BAND_LO_HZ) & (freqs <= min(BAND_HI_HZ, fs / 2 - 1))
    if sel.sum() < 8:
        return None
    s, f = spec[sel], freqs[sel]
    i = int(np.argmax(s))
    med = float(np.median(s))
    return float(f[i]), float(s[i]), float(s[i] / med) if med > 0 else float("inf")


def analyse(path, capture_fps, roi=None, bright=False):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return 2, f"FAIL: cannot open {path}"
    container_fps = cap.get(cv2.CAP_PROP_FPS)
    rows, n_frames = [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        n_frames += 1
        if roi:
            x, y, w, h = roi
            frame = frame[y:y + h, x:x + w]
            if frame.size == 0:
                return 2, "FAIL: --roi lies outside the picture"
        scale = WORK_WIDTH / frame.shape[1]
        if scale < 1:
            frame = cv2.resize(frame, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA)
        rows.append(measure_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), bright))
    cap.release()

    real_s = n_frames / capture_fps
    stretch = capture_fps / container_fps if container_fps > 0 else float("nan")
    lines = [f"file says {container_fps:.2f} fps, {n_frames} frames; "
             f"you say it was captured at {capture_fps:g} fps",
             f"  => {real_s:.2f} s of REAL time; playback is slowed x{stretch:.2f} "
             f"(1.00 means the file plays in real time)"]
    if n_frames == 0:
        return 2, "\n".join(lines + ["FAIL: no frames could be read"])
    if real_s < MIN_REAL_SECONDS:
        return 2, "\n".join(lines + [
            f"FAIL: only {real_s:.2f} s of real time (need {MIN_REAL_SECONDS:.0f}). "
            "Film longer, or check --capture-fps."])
    valid = np.array([r is not None for r in rows])
    frac = valid.mean()
    lines.append(f"aircraft outline found in {valid.sum()}/{n_frames} frames ({frac:.0%})")
    if frac < MIN_VALID_FRACTION:
        return 2, "\n".join(lines + [
            f"FAIL: tracking lost (need {MIN_VALID_FRACTION:.0%}). Try --roi x,y,w,h "
            "around the aircraft, --bright if it is lighter than the background, "
            "or re-film against plain sky from directly behind."])
    idx = np.arange(n_frames)
    data = np.array([r for r in rows if r is not None])
    sig = {name: np.interp(idx, idx[valid], data[:, k])
           for k, name in enumerate(("tilt", "x", "y"))}

    res = {name: spectrum_peak(s, capture_fps) for name, s in sig.items()}
    if res["tilt"] is None:
        return 2, "\n".join(lines + ["FAIL: clip too short for a spectrum"])
    f0, amp, ratio = res["tilt"]
    lines.append(f"  tilt of the outline : peak {f0:5.1f} Hz, +/-{amp:.2f} deg, "
                 f"{ratio:.0f}x the noise floor")
    for name, label in (("x", "sideways position"), ("y", "up-down position ")):
        if res[name]:
            fx, ax, rx = res[name]
            lines.append(f"  {label} : peak {fx:5.1f} Hz, +/-{ax:.1f} px, "
                         f"{rx:.0f}x the noise floor (cross-check only)")
    if ratio < MIN_PEAK_TO_MEDIAN or amp < MIN_TILT_DEG:
        return 2, "\n".join(lines + [
            f"FAIL: no clear rocking peak (need {MIN_PEAK_TO_MEDIAN:.0f}x the noise "
            f"floor and at least +/-{MIN_TILT_DEG:.2f} deg; got {ratio:.1f}x, "
            f"+/-{amp:.2f} deg). Either it is not rocking, or the view does not show it."])
    band = ("ATTITUDE-loop / mechanical band" if f0 < 8 else
            "RATE-loop band" if f0 > 12 else "ambiguous band")
    lines.append(f"VERDICT: dominant rock {f0:.1f} Hz -> {band}")
    lines.append("  (the +/-deg figure is the tilt as seen by the camera; it equals the "
                 "real tilt only if you filmed along the axis it rocks about)")
    if f0 < 2.0:
        lines.append("  CAUTION: below 2 Hz a hand-held phone's own sway looks the same.")
    return 0, "\n".join(lines)


def _synth(path, f_hz, capture_fps, container_fps, real_s, amp_deg=4.0):
    """Self-test fixture: a dark bar rocking at f_hz, filmed at capture_fps but
    written with a DIFFERENT container fps (as an exported slo-mo clip is)."""
    w, h = 320, 240
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), container_fps, (w, h))
    if not vw.isOpened():
        raise RuntimeError("cannot open a video writer for the self-test")
    rng = np.random.default_rng(0)
    for i in range(int(real_s * capture_fps)):
        t = i / capture_fps
        ang = amp_deg * math.sin(2 * math.pi * f_hz * t)
        img = np.full((h, w, 3), 235, np.uint8)
        box = cv2.boxPoints(((w / 2 + 3 * math.sin(0.7 * t), h / 2), (150, 22), ang))
        cv2.fillPoly(img, [box.astype(np.int32)], (30, 30, 30), lineType=cv2.LINE_AA)
        img = np.clip(img.astype(np.int16) + rng.integers(-4, 5, img.shape), 0, 255)
        vw.write(img.astype(np.uint8))
    vw.release()


def self_test() -> int:
    cv2.setNumThreads(1)      # tiny frames: thread hand-off costs more than it saves
    ok = True
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "clip.avi")
        for f_hz, cap_fps, cont_fps in ((5.0, 240.0, 30.0), (17.0, 240.0, 30.0),
                                        (9.0, 120.0, 30.0)):
            _synth(p, f_hz, cap_fps, cont_fps, 5.0)
            rc, txt = analyse(p, cap_fps)
            good = rc == 0 and f"dominant rock {f_hz:.1f} Hz" in txt
            print(f"{'ok  ' if good else 'FAIL'} recovers {f_hz} Hz from a {cap_fps:g} fps "
                  f"capture stored as {cont_fps:g} fps")
            if not good:
                print(txt)
            ok &= good
        # the trap this tool exists to avoid: trusting the file's own frame rate
        _synth(p, 16.0, 240.0, 30.0, 5.0)
        rc, txt = analyse(p, 30.0)
        good = rc == 0 and "dominant rock 2.0 Hz" in txt
        print(f"{'ok  ' if good else 'FAIL'} same clip read at the FILE's 30 fps gives the "
              "wrong, 8x-too-slow answer (why --capture-fps is mandatory)")
        ok &= good
        _synth(p, 6.0, 240.0, 30.0, 5.0, amp_deg=0.0)
        rc, _ = analyse(p, 240.0)
        print(f"{'ok  ' if rc == 2 else 'FAIL'} a clip with no rocking fails closed")
        ok &= rc == 2
        _synth(p, 6.0, 240.0, 30.0, 2.0)
        rc, _ = analyse(p, 240.0)
        print(f"{'ok  ' if rc == 2 else 'FAIL'} a 2 s clip fails closed")
        ok &= rc == 2
        _synth(p, 6.0, 240.0, 30.0, 5.0)
        rc, _ = analyse(p, 240.0, bright=True)   # wrong polarity: grabs the background
        print(f"{'ok  ' if rc == 2 else 'FAIL'} lost tracking fails closed")
        ok &= rc == 2
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("video", nargs="?")
    ap.add_argument("--capture-fps", type=float,
                    help="REQUIRED: the rate the phone really recorded at (e.g. 240)")
    ap.add_argument("--roi", help="x,y,w,h in original pixels, around the aircraft")
    ap.add_argument("--bright", action="store_true",
                    help="aircraft is LIGHTER than the background")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not a.video:
        ap.error("give a video file or --self-test")
    if not a.capture_fps or a.capture_fps <= 0:
        print("FAIL: --capture-fps is required (the phone's real recording rate, "
              "e.g. 240). The file's own frame rate is not trusted.")
        return 2
    roi = None
    if a.roi:
        try:
            roi = [int(v) for v in a.roi.split(",")]
            assert len(roi) == 4 and roi[2] > 0 and roi[3] > 0
        except (ValueError, AssertionError):
            print("FAIL: --roi must be x,y,w,h")
            return 2
    rc, txt = analyse(a.video, a.capture_fps, roi, a.bright)
    print(txt)
    return rc


if __name__ == "__main__":
    sys.exit(main())
