#!/usr/bin/env python3
"""M4 gate script: intercept a MOVING AprilTag using ONLY the live camera as
target feedback, running either a pursuit or a proportional-navigation
(pro-nav) guidance law, and log the miss distance. See GOALS.md milestone
M4, scripts/m3_static_intercept.py (reused wholesale for setup/detection/
ground-truth plumbing), and scripts/m4_target_mover.py (the separate
process that drives the tag along a straight line).

THE STRAPDOWN-SEEKER PROBLEM (why yaw compensation is not optional here):
M3's camera boresight pointed at a STATIONARY target, so "how fast is the
tag drifting across the image" and "how fast is the true line of sight
(LOS) rotating in the world" were the same number. M4's camera is mounted
on a vehicle whose YAW ACTIVELY TRACKS the tag (same yaw-centering law as
M3), so the two are no longer the same thing at all: if the vehicle turns
in lockstep with the target, the tag can sit almost still in the image
(bearing beta roughly constant) while the true LOS is sweeping through the
sky just as fast as the vehicle is turning. A pro-nav law built naively off
d(beta)/dt would then see a near-zero rate and command almost no lateral
turn -- an inert, gutted pro-nav that looks like it's "running" but isn't
reacting to anything. The fix is to reconstruct the INERTIAL LOS azimuth,

    lambda = psi + beta

where psi is the vehicle's own yaw (from PX4's own EKF, MAVSDK
`attitude_euler()` -- this is the vehicle's OWN state, not target
information, so it is fair game under the "own-state from PX4, target from
the camera" split from ADR-0008/M3) and beta is the camera bearing (still
purely a camera measurement). lambda_dot -- the derivative of THIS
quantity, not of beta alone -- is the real LOS rate pro-nav needs. (psi and
beta share the same rotational-sign convention -- MAVSDK's yaw_deg is
"positive turning right/clockwise seen from above", and beta is "positive
when the tag is right of the nose" -- so adding them is dimensionally and
directionally sound; see the guidance-law section below and --bench, which
exists purely to prove this sign compensation is really working.)

ALPHA-BETA FILTER, IN TWO SENTENCES: lambda and range both come from a
noisy, occasionally-dropped-out camera measurement, so each is tracked with
a small constant-velocity (g-h / alpha-beta) filter that PREDICTS every
control tick (x_hat += xdot_hat * dt) and only CORRECTS -- nudging the
estimate and its rate toward the measurement -- on ticks where a genuinely
new detection arrived. This gives both a smoothed instantaneous value
(lambda_hat, R_hat) and, just as important for pro-nav, a rate estimate
(lambda_dot_hat) that keeps predicting sanely through a brief dropout
instead of going to zero or undefined the instant a frame is missed.

PURSUIT VS PRO-NAV -- SHARED EVERYTHING BUT THE LATERAL TERM: both laws use
the exact same closing-speed control, the exact same yaw-centering
sensor-pointing loop, the exact same altitude loop, the exact same filters,
and the exact same NED command path. The ONLY difference is the lateral
("perpendicular to LOS") velocity term: pursuit always holds it at zero
(aim straight up the LOS -- lag against a mover, by design, is the point of
running this baseline); pro-nav integrates `a_cmd = N * Vc * lambda_dot_hat`
into it. Sharing every other line of code is what makes the miss-distance
comparison between the two an honest one -- any difference in outcome
traces to that one term, not to some other incidental gain difference.

CAMERA-ONLY BREAKOFF, GROUND-TRUTH-ONLY MISS (ADR-0008 lineage): exactly
like M3, every control decision -- when to acquire, when to break off, what
to command -- is made from the camera's own measurements and the vehicle's
own EKF state. Ground truth (from `/world/apriltag/pose/info`, same
ADR-0006 transform chain) is sampled every tick purely to SCORE the run
(the miss-distance number in the gate) -- never read by the control law.
grep for "gt_" and you will only find it downstream of a command that was
already sent.

SEEKER SOURCE (--seeker apriltag|markerless, ADR-0033 item 2): the default
`apriltag` path is byte-identical to before -- pupil-apriltags on the fiducial.
`--seeker markerless` swaps ONLY the detection SOURCE to the two-stage markerless
seeker (scripts/seeker/two_stage_seeker.py, via scripts/seeker/markerless_loop.py):
it reads the target BODY, never a marker, and emits the SAME Measurement, so the
guidance/tracker/pro-nav math is unchanged. It reads ONLY camera pixels + fixed
intrinsics (never gt_*), so being a NEW guidance path it RE-EARNS the numeric
no-cheat audit at the live Gazebo A/B (the NEXT gated step). Its deps
(onnxruntime + opencv-contrib) live in .venv-seeker, not the main .venv.

WHY NED VELOCITY + ABSOLUTE YAW SETPOINTS (changed from body-frame +
yawspeed mid-M4; ADR-0009 second addendum): the guidance law already
produces a world-frame velocity vector, PX4's body-frame velocity
tracking measurably degrades at speed, and commanding yaw as an ABSOLUTE
angle (psi + beta, the tag's azimuth) hands the pointing problem to
PX4's attitude loop -- no LOS-rate feedforward or yaw-rate tuning needed.
The camera is still the only target sensor.

TERMINAL-PHASE RULES, AND WHY: as range R -> 0, lambda_dot is mathematically
singular (a fixed absolute angular wobble corresponds to an ever-larger
apparent rate as you get close), so v_perp integration FREEZES inside
TERMINAL_FREEZE_RANGE_M -- we stop trusting lambda_dot near the end and just
coast on the last commanded lateral rate. Likewise, a brief loss of the tag
from the frame in the last few meters is EXPECTED, not a bug: pursuit in
particular can spin hard enough chasing a fast LOS rate that the tag
briefly leaves the field of view right as it matters most -- that is
precisely the failure mode this milestone exists to demonstrate, so instead
of instantly panicking to a hover, the terminal-range dropout rule holds
the last command for up to TERMINAL_HOLD_MAX_S and lets the geometry (and
the log) speak for itself.

S2 -- TWO-STAGE EXTERNAL-CUE HANDOFF (--handoff, requires --fpv; ADR-0010
decisions #4/#5, ADR-0011 third addendum): a fourth phase machine, layered
in front of the SAME, UNMODIFIED ENGAGE/BREAKOFF terminal logic above.
CUE_WAIT (hover, wait on scripts/s2_cue_mock.py's degraded external-cue
UDP stream instead of the camera) -> DASH (lead-pursuit toward the shared
TargetTracker -- the SAME class PIP's law uses -- at a fixed DASH_SPEED, a
"running start" against a target that starts well outside the camera's own
detection envelope) -> HANDOFF (>=3 consecutive fresh camera detections at
range <= HANDOFF_RANGE_M; the cue channel is then LATCHED closed one-way --
socket closed, thread stopped, the Python reference itself set to None, so
any future read attempt is structurally impossible, not merely unused by
convention, ADR-0006's lesson) -> phase becomes ENGAGE and every line above
this section runs completely unchanged, now flying off a tracker that the
cue has already warmed up with a real velocity estimate instead of a cold
start. See scripts/s2_cue_mock.py for the cue process itself and
docs/decisions.md's ADR-0010 (#4: exactly 3 degradation knobs) and #5 (the
hard handoff / no fusion / illegal-state-unrepresentable rule).

P-6 -- MID-COURSE FUSION + WARM HANDOFF (--fuse-midcourse / --warm-handoff,
both default OFF, both require --handoff; ADR-0015 fusion decision, ported
from guidance_lab.py's FusedTrack): an OPT-IN mid-course aid layered on the
DASH phase. --fuse-midcourse maintains a bearing-weighted polar FusedTrack
(the camera owns the LOS angle; the cue's range is folded in
inverse-variance-weighted; the cue's emitted velocity is used when present)
and aims the dash off it through the window where BOTH sources exist (first
camera detection -> the HANDOFF latch). --warm-handoff, at the latch,
seeds the camera-only terminal filters (lambda, lambda_dot, R, Rdot, v_perp)
+ the shared PIP track from that fused (or dash) track instead of letting
them converge from cold. This does NOT weaken ADR-0010 #5: fusion is a
PRE-latch mid-course aid using legal pre-latch cue reads; POST-latch the cue
socket stays closed one-way and the terminal is camera-only, unchanged. Lab
ranking (guidance_lab.py --p6-fusion, ADR-0015 fusion study): fusion helps
TRACK CONTINUITY through the terminal under EXPECTED realism (Pk@2 and
terminal detection coverage rise) but is neutral/sub-noise on mean miss and
inert under an early jammer link-cutoff; warm handoff is a small sub-noise
positive for a pronav terminal -- so a Gazebo A/B is what decides (NEXT.md
P-6, "lab ranks, Gazebo decides").

S2 WORLD-FRAME MAPPING (empirically verified, not assumed -- do not guess
this): the cue reports Gazebo WORLD x, y, z (same frame as gt_cam_x/y/z,
gt_tag_x/y/z elsewhere in this file), but the guidance/tracker frame is PX4
local NED (state.pos_n/pos_e from telemetry.position_velocity_ned). The
mapping is

    north (NED) = world_y      east (NED) = world_x

(standard Gazebo ENU world -- East/North/Up -- composed with PX4's fixed
ENU->NED bridge; GOALS.md's "World: ENU, origin at the interceptor's
start" means no translation term is needed either, since PX4's local NED
origin IS the spawn point.) Verified two independent ways against
existing FPV flight logs (main checkout's logs/, read-only during this
port):
  (1) Net ENGAGE-phase world displacement direction vs. mean commanded NED
      velocity direction, three FPV flights (logs/m4_intercept_pronav_
      20260705T044945Z.csv, ..._065536Z.csv, m4_intercept_pursuit_
      20260705T033640Z.csv): assuming world_x=east/world_y=north gives
      angle residuals of -14.0, -9.6, -10.1 degrees (small -- expected,
      since the velocity vector itself rotates during the window while
      this test uses one mean direction); assuming the OPPOSITE mapping
      (world_x=north/world_y=east, the naive first guess) gives -33.2,
      -68.1, -75.4 degrees -- ruled out.
  (2) Independent cross-check on the SAME three logs: at the ACQUIRE ->
      ENGAGE transition (right after the yaw-centering law has converged
      on the tag's true azimuth), the vehicle's actual measured yaw
      (psi_deg) reads ~127.3-128.4 degrees. Computing that same azimuth
      analytically from the known geometry (target start (6.5,-4),
      interceptor spawn ~(0,0)) as atan2(east_rel, north_rel) gives ~121.6
      degrees under world_x=east/world_y=north (a close match) vs. ~-31.6
      degrees under the opposite mapping (nowhere close).
Both checks agree: north=world_y, east=world_x, no sign flip. See
scripts/m4_intercept.py's ext-cue consumption block (search "WORLD ->
NED mapping") for where this is applied.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running, and the tag pre-placed at the engagement start position -- see
scripts/check_m4.sh's launch block for the exact env and pre-placement
command):

    .venv/bin/python scripts/m4_intercept.py --law pronav
    .venv/bin/python scripts/m4_intercept.py --law pursuit
    .venv/bin/python scripts/m4_intercept.py --law pronav --bench
    .venv/bin/python scripts/m4_intercept.py --law pip --fpv --handoff

Normally this is invoked by scripts/check_m4.sh (non-handoff) or
scripts/check_s2.sh (--handoff), which boot a fresh sim per law/run,
pre-place the tag, and run.
"""

import argparse
import asyncio
import csv
import json
import math
import os
import queue
import shlex
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Optional

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.clock_pb2 import Clock

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw
from mavsdk.telemetry import LandedState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from m2_detect import (  # noqa: E402
    get_camera_intrinsics,
    PoseTracker,
    IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    POSE_TOPIC,
)
from m0_takeoff import (  # noqa: E402
    TelemetryState,
    track_position,
    track_flight_mode,
    track_armed,
    track_landed_state,
    wait_for_connection,
    wait_for_health,
    SYSTEM_ADDRESS,
)
from m3_static_intercept import (  # noqa: E402
    Measurement,
    LatestFrame,
    MeasurementHolder,
    detection_loop,
    sample_measurement,
    ground_truth_world_points,
    _clamp,
)

# --- Guidance targets / gains (GOALS.md M4; pro-nav mechanization + gain
# council-decided per CLAUDE.md -- do not retune without an ADR). See the
# module docstring for the guidance law itself. ---
# Camera sits ~0.25 m above the model origin (base_link ~0 m + mount offset,
# see ADR-0006) -> roughly ~0.75 m up; tag center is at 0.5 m, so a LOWER
# reference altitude than M3's 1.0 m keeps the vertical component small so
# it doesn't eat into the <1 m 3D miss-distance budget.
ALT_REF_M = 0.5
N_PRONAV = 4.0  # pro-nav gain (typical range 3-5, GOALS.md guidance arc)
ALPHA = 0.5  # alpha-beta filter position gain (both lambda and range channels)
# Rate gains split per channel after dev run ...T012819Z: the lambda
# channel's 0.15 rate gain lagged a building LOS rate by ~0.5 s (est -3
# deg/s while truth was -16), making the yaw feedforward useless exactly
# when it mattered. 0.30 halves that lag; range keeps the calmer 0.15
# (Vc doesn't need to be fast, it needs to be smooth).
BETA_GAIN_LAMBDA = 0.30  # alpha-beta rate gain, lambda (LOS) channel
BETA_GAIN_RANGE = 0.15  # alpha-beta rate gain, range channel
# Closing speed is CONSTANT during ENGAGE (changed after the first official
# gate run, m4_intercept_pronav_...T025458Z: the original v_close =
# 0.8*R_hat law -- inherited from M3's gentle standoff approach -- throttled
# the interceptor to ~1.1 m/s inside 1.5 m while the target cruised at
# 2.0 m/s, so the target simply outran it at CPA and the terminal LOS rate
# blew up as the symptom. An interceptor does not slow down at the target;
# rendezvous-style braking is a standoff behavior, not an intercept one.)
V_CLOSE_MAX = 3.0  # m/s, commanded along-LOS speed for the whole ENGAGE phase
# V_PERP_MAX raised 2.0 -> 3.0 after dev run ...T011913Z: the lead demand at
# the council geometry saturated a 2.0 m/s cap for the whole engagement,
# capping how fast pro-nav could null the LOS rate.
V_PERP_MAX = 3.0  # m/s, lateral (pro-nav) velocity clamp
V_TOTAL_MAX = 4.0  # m/s, horizontal command safety clamp after combining close+perp
# Yaw sensor-pointing loop: retuned after dev run ...T011913Z, where a pure
# P-loop (gain 1.5) lagged the rotating LOS by a growing margin (commanded
# -37 deg/s, LOS moving ~-25 deg/s, bearing diverged to -40 deg -> detection
# lost: off-axis + oblique board view kills the detector well inside the
# nominal +-50 deg half-FOV). ENGAGE now adds the filtered LOS rate as a
# FEEDFORWARD term (yawspeed = lambda_dot_hat + KYAW*beta) -- zero
# steady-state lag against a constantly rotating LOS, still built purely
# from camera + own-state. Both laws share it (fairness).
KYAW_DEG_PER_DEG = 3.0  # yaw-centering P-gain (sensor pointing)
YAWSPEED_MAX_DEG_S = 60.0  # raised from M3's 30 deg/s -- endgame LOS rates demand it
KP_ALT = 1.0  # 1/s
V_VERT_MAX = 0.5  # m/s
CONTROL_RATE_HZ = 20.0
MEAS_STALE_S = 0.4  # ignore (treat as no detection) measurements older than this
ACQUIRE_MIN_DETECTIONS = 10  # consecutive FRESH detections required to leave ACQUIRE
ACQUIRE_MIN_S = 1.0  # ... and at least this many seconds must also have elapsed
# ... and the tag must be CENTERED before engaging (added after dev run
# ...T012356Z, which engaged 1.02 s in -- mid-yaw-slew, 20 deg off-boresight,
# with the lambda filter still polluted by the initial turn; the engagement
# then started from a 20 deg hole it never climbed out of).
ACQUIRE_MAX_BEARING_DEG = 5.0
ACQUIRE_CENTERED_STREAK = 6  # ~0.4 s of consecutive centered detections
ACQUIRE_TIMEOUT_S = 20.0
ENGAGE_TIMEOUT_S = 30.0
TERMINAL_RANGE_M = 3.0  # inside this: a dropout holds the last command instead of hovering
TERMINAL_HOLD_MAX_S = 1.0  # max time to hold-last-command through a terminal dropout
# Terminal coast (raised 1.5 -> 2.0 and extended from freezing just v_perp to
# freezing the WHOLE commanded velocity vector, after official gate run
# ...T030007Z): as R->0 lambda_dot is singular -- by 1.5 m the estimate had
# already blown up to -53 deg/s (vs -13 at 2.0 m), whipping the commanded
# velocity DIRECTION and the yaw feedforward so fast that PX4's actual
# velocity collapsed to ~1.75 m/s sideways and the 2.0 m/s target pulled
# 1.0 m ahead by CPA (a pure tail-chase miss; cross-track was 0.10 m). A
# real interceptor's terminal phase does the same thing this fix does:
# stop chasing the singular LOS rate and fly the established collision
# course straight through the intercept.
TERMINAL_FREEZE_RANGE_M = 2.0  # inside this: coast -- hold the frozen velocity vector
BREAKOFF_ARM_RANGE_M = 2.5  # past-CPA breakoff logic arms only after R_hat dips below this
BREAKOFF_RANGE_INCREASES = 3  # consecutive FRESH detections w/ increasing range -> breakoff
BREAKOFF_HARD_FLOOR_M = 0.5  # measured range below this -> immediate breakoff
LOST_TAG_ABORT_S = 5.0  # continuous no-detection outside terminal range -> abort
CLIMB_S = 1.5  # breakoff climb duration
CLIMB_V = 1.0  # m/s up during breakoff climb
POST_BREAKOFF_LOG_S = 2.0  # total time spent (logging) in the BREAKOFF phase

# --- FPV profile (M4.5 sub-step S1, ADR-0010). Activated by --fpv; the
# module defaults above stay exactly at the M4-validated values so
# check_m4.sh is untouched. This profile OVERRIDES a bundle of constants
# for the higher-speed regime and is applied once in apply_fpv_profile()
# before the guidance loop runs. Design (ADR-0010 decision #2): DECOUPLE a
# fast run-in closing speed (to catch a ~6 m/s target) from a throttled
# terminal closing speed (to keep the terminal LOS-rate math at ~M4's
# validated ~3-4 m/s difficulty), and rescale every terminal RANGE so its
# TIME semantics survive the faster terminal speed. Numbers are the
# council's starting proposals -- validated/retuned by S1 dev runs, logged
# in ADR-0010 as they settle.
FPV = {
    # Two-speed closing law (see compute_v_close): fast run-in above the
    # throttle range, throttled terminal speed below it.
    "V_CLOSE_RUNIN": 9.0,      # m/s, closing speed while still far (catch the mover)
    "V_CLOSE_TERMINAL": 5.5,   # m/s, throttled closing speed in the terminal band
    "THROTTLE_RANGE_M": 5.0,   # r_hat at/below which closing throttles toward terminal
    # Lateral / total clamps scaled up for the faster regime.
    "V_PERP_MAX": 8.0,         # m/s, lateral (pro-nav) velocity clamp (S1 dev: 6 saturated, starving the Y-lead)
    "V_TOTAL_MAX": 13.0,       # m/s, horizontal command safety clamp
    # Terminal ranges rescaled ~1.7x (terminal speed 3.0 -> 5.5 m/s) so coast
    # DURATION and dropout-hold DURATION match M4's proven time semantics.
    "TERMINAL_RANGE_M": 5.0,
    "TERMINAL_FREEZE_RANGE_M": 3.5,
    "BREAKOFF_ARM_RANGE_M": 4.0,
    "VC_FLOOR_M_S": 4.5,       # range-rate floor (S1 dev: Vc pinned at 3.0 under-powered a=N*Vc*lam_dot; true closing ~4.5)
    # Range-rate filter must track a faster-swinging Rdot (the ADR-0009
    # lambda-lag pathology, now on the range channel per council seat A).
    "BETA_GAIN_RANGE": 0.45,
    # PX4 params set via MAVSDK before arming (ADR-0010 decision #3).
    "N_PRONAV": 5.0,          # lab trade study: N=5 minimized pure-PN miss (ADR-0011)
    "PX4_PARAMS": {
        "MPC_XY_VEL_MAX": 20.0,
        "MPC_ACC_HOR_MAX": 12.0,
        "MPC_TILTMAX_AIR": 60.0,
        "MPC_JERK_MAX": 30.0,
    },
}


def apply_fpv_profile():
    """Overwrite the module-level guidance constants with the FPV bundle
    (S1, ADR-0010). Called once from main() when --fpv is set, BEFORE the
    guidance loop reads any of them. Leaves the PX4 params for main() to
    push over MAVSDK (they aren't module constants)."""
    global V_PERP_MAX, V_TOTAL_MAX, TERMINAL_RANGE_M, TERMINAL_FREEZE_RANGE_M
    global BREAKOFF_ARM_RANGE_M, BETA_GAIN_RANGE, N_PRONAV
    V_PERP_MAX = FPV["V_PERP_MAX"]
    V_TOTAL_MAX = FPV["V_TOTAL_MAX"]
    TERMINAL_RANGE_M = FPV["TERMINAL_RANGE_M"]
    TERMINAL_FREEZE_RANGE_M = FPV["TERMINAL_FREEZE_RANGE_M"]
    BREAKOFF_ARM_RANGE_M = FPV["BREAKOFF_ARM_RANGE_M"]
    BETA_GAIN_RANGE = FPV["BETA_GAIN_RANGE"]
    N_PRONAV = FPV["N_PRONAV"]


# --- S2 two-stage external-cue handoff (M4.5 sub-step S2, ADR-0010 decisions
# #4/#5; guidance_lab.py's TwoStageDash + the ADR-0011 third-addendum dash
# sweep). Activated by --handoff (requires --fpv -- enforced in parse_args).
# CUE_WAIT (hover, wait for the external cue) -> DASH (lead-pursuit toward
# the cue-fed shared TargetTracker at DASH_SPEED, a "running start") ->
# HANDOFF (camera acquires within range; cue channel latched closed,
# ADR-0010 #5) -> the EXISTING, unmodified ENGAGE/BREAKOFF terminal logic.
# Defaults are the lab's validated S2 sweep winner (ADR-0011 third addendum:
# dash 10 m/s, handoff 10 m, terminal PIP beats pure PN once the cue has
# warmed the shared tracker's velocity estimate before handoff -- the
# opposite of PIP's camera-only-from-hover result, ADR-0011/its addendum).
S2 = {
    "DASH_SPEED": 10.0,          # m/s, cue-guided lead-pursuit closing speed during DASH
    "HANDOFF_RANGE_M": 10.0,     # camera-measured range at/below which HANDOFF may trigger
                                 # (Tier-2 tried 12 with a 60deg lens; both REJECTED, ADR-0024
                                 #  3rd addendum -- 60deg couldn't hold a fast crosser. The
                                 #  validated fast-regime acquisition fix is streak-min=2 via
                                 #  --early-handoff, not FOV/range. Wide lens + gate 10 stands.)
    "DASH_MAX_LEAD_S": 4.0,      # PIP-style intercept-triangle lead cap during DASH
    "CUE_PORT": 47800,
    "DASH_TIMEOUT_S": 20.0,      # DASH phase abort timeout
    "CUE_WAIT_TIMEOUT_S": 15.0,  # CUE_WAIT phase abort timeout
    "CUE_WAIT_MIN_DATAGRAMS": 5,   # >= this many cue datagrams received ...
    "CUE_WAIT_MIN_CORRECTIONS": 3,  # ... AND the shared tracker corrected >= this many times ...
    "HANDOFF_STREAK_MIN": 3,      # ... consecutive fresh in-range camera detections to trigger HANDOFF
    # Cue mock degradation (ADR-0010 #4: exactly 3 knobs -- sigma, latency,
    # rate). Forwarded to scripts/s2_cue_mock.py's CLI.
    "CUE_MOCK_SIGMA_M": 0.5,
    "CUE_MOCK_LATENCY_S": 0.12,
    "CUE_MOCK_RATE_HZ": 10.0,
    "CUE_MOCK_RUN_DURATION_S": 60.0,  # generous; the cue mock outlives HANDOFF (audit evidence), killed at run end
    # Engagement geometry defaults used ONLY when --handoff is set and the
    # corresponding CLI flag was not explicitly given (parse_args) -- mirrors
    # guidance_lab.py's S2_PATH_PARAMS (x0=6.5, y0_mag=14.0 -> ~15.4 m
    # initial range, "real dash runway" per the lab's own comment, ADR-0011
    # third addendum), NOT the M4/S1 short-standoff geometry (which sits
    # inside the camera's own detection envelope and gives the cue no
    # runway by construction).
    "TARGET_START_DEFAULT": "6.5,-14,0.5",
    "TARGET_VEL_DEFAULT": "0,6.0",
    "MOVER_DURATION_DEFAULT_S": 20.0,
}
CUE_MOCK_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "s2_cue_mock.py")
# ADR-0015 dev-flight passthrough: extra flags appended VERBATIM (shlex-split)
# to the auto-spawned s2_cue_mock.py command, sourced from this env var so a
# space-separated realism flag list survives check_s2.sh's word-splitting of
# EXTRA_ARGS. Unset/empty => the cue command is byte-identical to before
# ADR-0015 (the S2 gate default path is untouched). Example:
#   S2_CUE_MOCK_EXTRA="--sigma-range --emit-velocity --dropout-markov"
CUE_MOCK_EXTRA_ENV = "S2_CUE_MOCK_EXTRA"

# --- ADR-0023 Tier-1 guidance-reclaim flags (--early-handoff /
# --split-freeze; both default OFF = byte-identical S2 gate). Values are
# the guidance_lab.py --tier1 pre-sweep picks (EXPECTED tier, 6 m/s,
# 120 seeds/cell, logs/guidance_lab_tier1_20260706T164637Z.csv) -- ported
# for the Gazebo A/B; "lab ranks, Gazebo decides."
#   --early-handoff (Tier-1 lever A): trigger the one-way HANDOFF latch on
#     a SHORTER solid camera streak (2 instead of 3 consecutive fresh
#     in-range detections -- the same existing streak logic, just a lower
#     count; a fiducial detection has a near-zero false-positive rate, so
#     the 3-streak was settling, not spoof rejection). Engages the
#     camera-only terminal ~1 m earlier. HONESTY: the latch stays one-way
#     and closes the cue SOONER -- strictly less cue data (ADR-0010 #5).
#     NB the lab ranked this lever NEGATIVE at every tier (the dash's
#     cue+camera lead solve out-corrects the camera-only terminal in that
#     band); it is ported anyway because the lab's dash is documented
#     optimistic vs Gazebo's (ZEM@handoff 0.4 m lab vs 1.69 m Gazebo) --
#     exactly the kind of blind spot only a Gazebo A/B settles.
#   --split-freeze (Tier-1 lever B, ADR-0014 lever 2): replace the
#     whole-vector terminal freeze with a v_perp-magnitude-only freeze
#     (v_close/yaw/lambda_hat stay LIVE off fresh detections; only the
#     lambda_dot-driven term is singular as R->0), cap |lambda_dot| in
#     BOTH a_cmd and the filter's own predict() (AlphaBetaFilter.rate_cap
#     -- the anti-whipsaw requirement), and move the freeze later
#     (TERMINAL_FREEZE_RANGE_M 3.5 -> 1.5, which per compute_v_close also
#     extends the run-in throttle band exactly as the lab's B variant
#     does). Lever C (warm-settled terminal filters) needs no new flag --
#     it IS the existing ADR-0018 --warm-handoff seeding.
EARLY_HANDOFF_STREAK_MIN = 2
SPLIT_FREEZE_RANGE_M = 1.5
SPLIT_FREEZE_LAMBDA_DOT_CAP_DEG_S = 60.0

# --- ADR-0028 Gazebo-confirm flag (--accel-boost; default OFF, requires
# --fpv): the lab's SEPARATE "airframe agility" lever (as opposed to the
# "running start" engagement-geometry lever, which needs no code change --
# mc_batch.sh's existing --y0-mag/--x0 + m4_intercept.py's existing
# --dash-speed already cover it). Overrides two of the four FPV["PX4_PARAMS"]
# runtime-set values ~2x: MPC_ACC_HOR_MAX 12 -> 20 m/s^2, MPC_TILTMAX_AIR
# 60 -> 70 deg. ADR-0028's lab caveat: the x500 airframe may be tilt/
# thrust-limited well below a commanded 20 m/s^2 -- that is an expected,
# reportable outcome (compare ACHIEVED accel from the flight's own velocity
# trace against the commanded ceiling), not a bug.
ACCEL_BOOST_MPC_ACC_HOR_MAX = 20.0
ACCEL_BOOST_MPC_TILTMAX_AIR = 70.0

# --- ADR-0028 addendum follow-up flag (--dash-unclamp; default OFF,
# requires --fpv): the running-start dash commands a unit vector toward the
# lead point scaled to S2["DASH_SPEED"] (see the DASH phase's vh0/vh1
# construction), so its norm is DASH_SPEED exactly -- then the FPV
# V_TOTAL_MAX safety clamp silently rescales it back down. At the
# ADR-0028-confirm dash speed (16 m/s) this is a real, silent inconsistency:
# FPV["V_TOTAL_MAX"] defaults to 13, so a `--dash-speed 16` dash is clamped
# to 13 m/s the whole time while the lead-point solve (solve_intercept_time)
# assumed the vehicle actually flies at 16. Raising the ceiling to 18 lets a
# 16 m/s dash reach its commanded speed with a few m/s of lateral-lead
# headroom to spare, without touching V_PERP_MAX (unrelated -- that's the
# terminal pro-nav lateral clamp, not the dash's total-speed clamp).
DASH_UNCLAMP_V_TOTAL_MAX = 18.0

# ADR-0015 --coast-search (link-loss-before-lock dead-reckon + bounded seeker
# search; docs/decisions.md ADR-0015 "Handoff continuity"). All sim-time.
COAST_STALE_S = 1.0           # cue silent longer than this (sim) => stale/link-loss
COAST_ACQ_RANGE_M = 10.0      # predicted range at/below which the bounded search opens
COAST_SEARCH_YAW_AMP_DEG = 20.0   # +/- yaw sweep amplitude around the predicted LOS
COAST_SEARCH_PERIOD_S = 4.0       # sweep period (=> ~31 deg/s peak, a "modest rate")
COAST_SEARCH_BUDGET_S = 8.0       # no acquisition within this (sim) after cue loss => BREAKOFF
# Tracking-refinement PORT 2 (--cue-latency-comp): sim-time topic, standard
# gz-transport Clock message (same one scripts/m4_target_mover.py's clock
# helper child process reads).
CLOCK_TOPIC = "/clock"


def compute_v_close(r_hat, fpv_on):
    """Commanded along-LOS closing speed. M4 default: constant V_CLOSE_MAX.
    FPV (S1): fast run-in far out, linearly throttled to V_CLOSE_TERMINAL as
    r_hat falls through THROTTLE_RANGE_M down to TERMINAL_FREEZE_RANGE_M --
    catch the fast target, then slow the terminal geometry so lambda_dot
    doesn't blow up (ADR-0010 decision #2, seat A's detection-window warning)."""
    if not fpv_on:
        return V_CLOSE_MAX
    runin = FPV["V_CLOSE_RUNIN"]
    term = FPV["V_CLOSE_TERMINAL"]
    hi = FPV["THROTTLE_RANGE_M"]
    lo = FPV["TERMINAL_FREEZE_RANGE_M"]
    if r_hat is None or r_hat >= hi:
        return runin
    if r_hat <= lo:
        return term
    frac = (r_hat - lo) / (hi - lo)  # 1 at hi -> 0 at lo
    return term + frac * (runin - term)

# --- MAVSDK / takeoff constants (mirrors scripts/m3_static_intercept.py's
# own constants -- not reused via import, since M3 doesn't export these as
# module-level names meant for reuse; they're the same numbers by design). ---
CONNECT_TIMEOUT_S = 60
HEALTH_TIMEOUT_S = 90
ALTITUDE_SUCCESS_FRACTION = 0.8
ALTITUDE_TIMEOUT_S = 60
LAND_TIMEOUT_S = 90
LOG_POLL_HZ = 5.0
POSE_WAIT_TIMEOUT_S = 30.0
CAMERA_INFO_TIMEOUT_S = 30.0

# --- --bench mode constants (sign-convention validation; see run_bench()). ---
BENCH_WAVE_DEG_S = 20.0
BENCH_SEG_S = 2.0
BENCH_CYCLES = 2
BENCH_RECENTER_S = 3.0
BENCH_PASS_MEAN_DEG_S = 3.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")
MOVER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "m4_target_mover.py")

CSV_HEADER = [
    # t = wall-clock elapsed since script start (time.monotonic() - started);
    # t_sim = SIM time from /clock (SimClockHolder.t), ONLY populated under
    # --handoff (the only mode that subscribes to /clock -- see
    # SimClockHolder). Blank otherwise. Demo-tooling need (docs/demo_plan.md):
    # RTF sag makes wall != sim, so HUD/video time-sync must key off t_sim,
    # not t.
    "t", "t_sim", "phase", "law", "detected",
    "meas_x", "meas_y", "meas_z", "meas_range", "bearing_rad",
    "psi_deg", "lambda_deg", "lambda_dot_deg_s",
    "r_hat_m", "rdot_hat_m_s", "vc_m_s", "a_cmd_m_s2", "v_perp_m_s",
    "cmd_vn", "cmd_ve", "cmd_vd", "cmd_yaw_deg",
    "alt_m",
    "gt_cam_x", "gt_cam_y", "gt_cam_z",
    "gt_tag_x", "gt_tag_y", "gt_tag_z", "gt_range",
    # S2 (--handoff): NED-mapped external cue used THIS tick (blank when no
    # cue corrected the tracker this tick, including always-blank post-
    # HANDOFF -- ADR-0010 #5) and the shared TargetTracker's own state
    # (always present once initialized, any law/mode -- the same tracker
    # PIP's law reads, see class TargetTracker).
    "ext_x", "ext_y", "ext_z", "ext_fresh",
    "tgt_n_hat", "tgt_e_hat", "tgt_vn_hat", "tgt_ve_hat",
    # Tracking-refinement PORT 2 (--cue-latency-comp): computed cue age
    # (sim_now - t_sim, clamped) at the tick a cue actually corrected the
    # tracker; blank when no cue was used this tick (mirrors ext_fresh).
    "ext_age_s",
    # ADR-0015 --coast-search (link-loss-before-lock): cue_stale = the cue
    # stream has gone silent longer than COAST_STALE_S (1 = stale, blank when
    # --coast-search is off); coast_phase = "coast" (dead-reckon dash) /
    # "search" (bounded yaw sweep at the predicted basket) / blank.
    "cue_stale", "coast_phase",
    # Fusion capstone (design D2/F4, ADR-0041): the EKF's OWN relative
    # target state + P diagonal, scored against gt_* in the RELATIVE frame
    # (tgt_n_hat/tgt_e_hat above always come from the alpha-beta
    # TargetTracker in EVERY arm -- ADR-0037's position-RMSE null was
    # structurally guaranteed; this is the field that actually measures the
    # EKF). Empty string unless --tracker ekf (existing empty-field
    # convention -- see write_row_m4's fmt()).
    "ekf_dn_hat", "ekf_de_hat", "ekf_dvn_hat", "ekf_dve_hat",
    "ekf_p_diag0", "ekf_p_diag1", "ekf_p_diag2", "ekf_p_diag3",
]


def wrap_pi(angle: float) -> float:
    """Wrap an angle (radians) into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# --- Kalata-derived alpha-beta gains (tracking-refinement PORT 1, --kalata).
# Ported VERBATIM (formula + variable names) from scripts/guidance_lab.py's
# kalata_alpha_beta() -- see that function's docstring for the full
# derivation/citation (T.P. Kalata, 1984, "The Tracking Index..."). Do not
# re-derive; this is a direct port of the validated lab code. ---
def kalata_alpha_beta(sigma_process: float, sigma_meas: float, dt: float):
    """Steady-state alpha-beta gains from the Kalata tracking index, given
    an assumed process (target-maneuver) noise std `sigma_process` (units =
    the tracked quantity's 2nd derivative), a measurement-noise std
    `sigma_meas` (units = the tracked quantity itself), and the ACTUAL
    sample interval `dt` since the last correction -- recomputing at the
    real interval is the point: a longer gap (e.g. after a dropout)
    produces a larger tracking index and hence larger alpha/beta, correctly
    trusting the next real measurement more because the prediction has had
    longer to drift. See scripts/guidance_lab.py's kalata_alpha_beta() for
    the full citation and derivation this is ported from."""
    dt = max(dt, 1e-3)
    sigma_meas = max(sigma_meas, 1e-9)
    lambda_idx = sigma_process * dt * dt / sigma_meas
    r = (4.0 + lambda_idx - math.sqrt(8.0 * lambda_idx + lambda_idx * lambda_idx)) / 4.0
    alpha = _clamp(1.0 - r * r, 0.0, 0.999)
    beta = 2.0 * (2.0 - alpha) - 4.0 * math.sqrt(max(0.0, 1.0 - alpha))
    return alpha, beta


# Lab-recommended Kalata params (tracking-refinement study winner #1),
# forwarded verbatim by the coordinator: lambda channel assumes a
# 150 deg/s^2 process noise against a 6 deg measurement sigma; range
# channel assumes 0.1 m/s^2 against a 0.5 m measurement sigma (matches this
# file's own camera range-noise expectation).
KALATA_LAMBDA_SIGMA_PROCESS_DEG_S2 = 150.0
KALATA_LAMBDA_SIGMA_MEAS_DEG = 6.0
KALATA_RANGE_SIGMA_PROCESS_M_S2 = 0.1
KALATA_RANGE_SIGMA_MEAS_M = 0.5
KALATA_LOG_EVERY = 20  # print sampled (alpha, beta) at correction #1 and every Nth after


class AlphaBetaFilter:
    """Constant-velocity (g-h / alpha-beta) tracker for one scalar channel
    (either lambda, the inertial LOS azimuth, or R, the range). See the
    module docstring's "alpha-beta filter, in two sentences" for the idea.

    `predict()` is meant to be called every control tick; `correct()` only
    on ticks where a genuinely new (fresh) measurement is available. Set
    `angular=True` for the lambda channel: the RESIDUAL (measurement minus
    current estimate) is wrapped to [-pi, pi] before use, but x_hat itself
    is never wrapped -- that keeps lambda continuous (able to accumulate
    past +-180 deg as the vehicle spins) while still comparing correctly
    against a measurement that only makes sense mod 2*pi.

    KALATA MODE (--kalata, opt-in, tracking-refinement PORT 1): pass
    `kalata_sigma_process` (not None) to IGNORE the fixed `alpha`/`beta` and
    instead recompute them at every correct() from kalata_alpha_beta(...)
    using the ACTUAL elapsed time since the last correction -- ported
    faithfully from scripts/guidance_lab.py's AlphaBetaFilter "KALATA MODE"
    docstring. Default (`kalata_sigma_process=None`) is the untouched,
    baseline fixed-gain behavior -- byte-identical to before this port.
    `label` (e.g. "lambda"/"range") is used only to print a sampled
    (alpha, beta) every KALATA_LOG_EVERY corrections for a sanity check.
    """

    def __init__(
        self, alpha: float, beta: float, angular: bool = False,
        kalata_sigma_process=None, kalata_sigma_meas=None, label: Optional[str] = None,
        rate_cap: Optional[float] = None,
    ):
        self.alpha = alpha
        self.beta = beta
        self.angular = angular
        self.kalata_sigma_process = kalata_sigma_process
        self.kalata_sigma_meas = kalata_sigma_meas
        self.label = label
        # RATE CAP (--split-freeze, ADR-0014 lever 2 / ADR-0023 Tier-1 lever
        # B; ported from guidance_lab.py's AlphaBetaFilter): clamp xdot_hat
        # to +-rate_cap (rad/s or m/s) immediately after every correct().
        # Because predict() forward-integrates x_hat off this SAME stored
        # xdot_hat, one clamp here caps BOTH the rate any caller reads
        # (a_cmd = N*Vc*lambda_dot) AND the filter's own forward integration
        # -- capping only a_cmd while predict() keeps integrating a raw,
        # unbounded rate is the single easiest way to silently recreate the
        # ADR-0009 whipsaw (ADR-0014 seat C's biggest-risk flag). Default
        # None = untouched baseline behavior (byte-identical).
        self.rate_cap = rate_cap
        self.n_corrections = 0
        self.x_hat: Optional[float] = None
        self.xdot_hat: float = 0.0
        self.last_innovation: float = 0.0
        self._last_correction_t: Optional[float] = None

    @property
    def initialized(self) -> bool:
        return self.x_hat is not None

    def predict(self, dt: float) -> None:
        if self.x_hat is None:
            return
        self.x_hat += self.xdot_hat * dt

    def correct(self, meas: float, t: float, gain_scale: float = 1.0) -> None:
        """`gain_scale` (P-6 --fuse-midcourse, default 1.0 = byte-identical
        baseline -- x*1.0 is IEEE-exact) scales BOTH gains for this one
        correction: how FusedTrack folds a LOWER-confidence source (the cue
        range) into a filter whose nominal gains are tuned for the camera,
        without touching the stored gains. The weight is inverse-variance,
        computed by the caller from KNOWN sensor specs (FusedTrack.update_cue)
        -- ported from guidance_lab.py's AlphaBetaFilter.correct."""
        if self.x_hat is None:
            # First measurement: initialize position, zero rate (spec).
            self.x_hat = meas
            self.xdot_hat = 0.0
            self.last_innovation = 0.0
            self._last_correction_t = t
            return
        residual = meas - self.x_hat
        if self.angular:
            residual = wrap_pi(residual)
        dt_since = max(1e-3, t - self._last_correction_t)
        if self.kalata_sigma_process is not None:
            alpha, beta = kalata_alpha_beta(self.kalata_sigma_process, self.kalata_sigma_meas, dt_since)
        else:
            alpha, beta = self.alpha, self.beta
        self.x_hat += alpha * gain_scale * residual
        self.xdot_hat += beta * gain_scale * residual / dt_since
        if self.rate_cap is not None:
            self.xdot_hat = _clamp(self.xdot_hat, -self.rate_cap, self.rate_cap)
        self.last_innovation = residual
        self._last_correction_t = t
        self.n_corrections += 1
        if (
            self.kalata_sigma_process is not None and self.label
            and (self.n_corrections == 1 or self.n_corrections % KALATA_LOG_EVERY == 0)
        ):
            print(
                f"[kalata] {self.label}: correction #{self.n_corrections} "
                f"dt_since={dt_since:.4f}s -> alpha={alpha:.3f} beta={beta:.3f}"
            )


# --- Predicted Intercept Point (PIP) machinery (ADR-0011). Ported from the
# guidance_lab.py trade study, where PIP roughly halved the miss vs pure PN
# by aiming where the target WILL be, not where it is. Reuses AlphaBetaFilter
# on the target's ABSOLUTE north/east position (from own-EKF-position + the
# camera-measured relative vector -- target info still camera-only). ---
class TargetTracker:
    """Alpha-beta filters on the target's absolute (north, east) position,
    yielding a position+velocity estimate from a stream of possibly-noisy,
    intermittent absolute-position measurements. The velocity estimate is
    what PIP needs to solve the intercept triangle and lead the target."""

    def __init__(self, alpha=0.6, beta=0.2):
        self.fn = AlphaBetaFilter(alpha, beta)
        self.fe = AlphaBetaFilter(alpha, beta)

    def predict(self, dt):
        self.fn.predict(dt)
        self.fe.predict(dt)

    def correct(self, abs_ne, t):
        self.fn.correct(abs_ne[0], t)
        self.fe.correct(abs_ne[1], t)

    def set_velocity(self, vn, ve):
        """ADR-0015 table #2 (--cue-velocity): overwrite the alpha-beta rate
        states directly from an externally-supplied (cue) velocity, rather
        than letting the filter differentiate a noisy position stream (which
        the council found injects ~7 m/s of velocity noise -- a PIP-killer).
        Only meaningful after at least one position correction has
        initialized the filters."""
        if self.fn.x_hat is not None:
            self.fn.xdot_hat = vn
            self.fe.xdot_hat = ve

    @property
    def initialized(self):
        return self.fn.x_hat is not None

    @property
    def pos_hat(self):
        if self.fn.x_hat is None:
            return None
        return (self.fn.x_hat, self.fe.x_hat)

    @property
    def vel_hat(self):
        if self.fn.x_hat is None:
            return None
        return (self.fn.xdot_hat, self.fe.xdot_hat)


def solve_intercept_time(rel, vt, v_close, max_lead_s):
    """Classic lead-pursuit / PIP intercept-triangle solve: smallest positive
    t such that |rel + vt*t| = v_close*t (target at `rel` relative to us,
    moving at constant `vt`; we close at constant speed `v_close`). Returns 0
    (aim at the target's current estimated position -- plain pursuit) when the
    target outruns v_close and diverges, or the quadratic degenerates.
    Ported verbatim from guidance_lab.py (ADR-0011)."""
    a = vt[0] * vt[0] + vt[1] * vt[1] - v_close * v_close
    b = 2.0 * (rel[0] * vt[0] + rel[1] * vt[1])
    c = rel[0] * rel[0] + rel[1] * rel[1]
    t_go = 0.0
    if abs(a) < 1e-9:
        if abs(b) > 1e-9:
            t = -c / b
            if t > 0:
                t_go = t
    else:
        disc = b * b - 4 * a * c
        if disc >= 0:
            sq = math.sqrt(disc)
            candidates = [tt for tt in ((-b + sq) / (2 * a), (-b - sq) / (2 * a)) if tt > 0]
            if candidates:
                t_go = min(candidates)
    return min(max(t_go, 0.0), max_lead_s)


# PIP tuning (ADR-0011 / guidance_lab.py winner: V_CLOSE=6 minimized miss).
PIP_TRACK_ALPHA = 0.6
PIP_TRACK_BETA = 0.2
PIP_MAX_LEAD_S = 3.0

# --- P-6 mid-course fusion / warm handoff (ADR-0015 fusion decision;
# NEXT.md P-6; ported from guidance_lab.py's FusedTrack -- keep the two in
# sync). All behind --fuse-midcourse / --warm-handoff (both default OFF, both
# require --handoff), so the S2 gate default path is byte-identical. Fusion
# is a PRE-LATCH mid-course aid only; POST-latch the cue socket stays closed
# one-way (ADR-0010 #5) and the terminal is camera-only, UNCHANGED. ---
# Drone-side assumed cue sigma model for the fused-range inverse-variance
# weight (the ADR-0015 recommended realistic sigma_R(R) = base + quad*R^2)
# RSS'd with a FIXED datum-bias budget at the shared-RTK design value. NOT
# read from the live cue: a fielded fusion is tuned to its design spec.
FUSE_CUE_SIGMA_BASE_M = 0.4
FUSE_CUE_SIGMA_QUAD = 0.008
FUSE_DATUM_BUDGET_M = 0.5
# Assumed onboard-seeker (AprilTag) range sigma fraction for the same weight
# ("know your own sensor spec"; ADR-0015 puts the AprilTag range at ~5-8%,
# 0.10 is a conservative match to guidance_lab.py's RANGE_NOISE_FRAC).
FUSE_CAM_RANGE_FRAC = 0.10
FUSE_RANGE_SIGMA_FLOOR_M = 0.3
FUSE_VEL_STALE_S = 0.6        # emitted-velocity freshness horizon (sim-time)


class FusedTrack:
    """P-6 mid-course fusion (ADR-0015 fusion mechanization; ported from
    guidance_lab.py's FusedTrack -- keep in sync). A POLAR fused target
    estimate maintained through the window where BOTH sources exist (camera
    detecting AND cue alive, i.e. first camera detection until the HANDOFF
    latch closes the cue). BEARING-WEIGHTED BY DESIGN:

      - lambda (inertial LOS azimuth psi+beta) channel: alpha-beta on the
        CAMERA only. The cue NEVER touches the angle. ADR-0015's own ruling
        is that bearing-only suffices for pro-nav (it consumes the LOS
        *rate*, an angle), so fusion must HELP the lambda_dot estimate, not
        fight it -- and a 2.5 m standard-GPS datum at 8 m range is ~18 deg
        of bearing error that would poison the seeker's clean few-degree
        angle. Camera wins the angle, always.
      - range channel: alpha-beta corrected by BOTH the camera monocular
        range (full gain) AND the cue-implied range |cue_ned - own_ned|
        (gain-scaled by an inverse-variance weight from KNOWN specs: camera
        sigma = FUSE_CAM_RANGE_FRAC*R_hat; cue sigma = the assumed
        sigma_R(R) RSS'd with the datum budget).
      - velocity: the cue's EMITTED NED velocity when fresh (ADR-0015's #1
        lever), else the caller's fallback (the shared dash TargetTracker's
        estimate).

    Per-tick scalar math -- trivially Pi-implementable. Note the lambda
    channel here is a SEPARATE filter from the guidance loop's own
    lambda_filter; at a warm handoff its state (+ the fused range/velocity)
    is what re-anchors the terminal filters."""

    def __init__(self):
        self.lam = AlphaBetaFilter(ALPHA, BETA_GAIN_LAMBDA, angular=True)
        self.rng_f = AlphaBetaFilter(ALPHA, BETA_GAIN_RANGE)
        self.vel_ned = None      # latest cue-EMITTED NED velocity (or None)
        self.vel_t = None
        self.last_camera_t = None

    def predict(self, dt):
        self.lam.predict(dt)
        self.rng_f.predict(dt)

    def update_camera(self, lambda_meas, range_m, t):
        """Camera correction: owns the angle, full-gain range."""
        self.lam.correct(lambda_meas, t)
        self.rng_f.correct(range_m, t)
        self.last_camera_t = t

    def update_cue(self, cue_range_m, cue_vel_ned, t):
        """Cue correction: velocity capture always; range folded in ONLY once
        the camera side exists (fusion window starts at first detection -- a
        biased cue must never define the CAMERA-anchored track's initial
        state, the angle-poisoning this mechanization exists to prevent)."""
        if cue_vel_ned is not None:
            self.vel_ned = cue_vel_ned
            self.vel_t = t
        if not (self.rng_f.initialized and self.lam.initialized):
            return
        r_hat = max(self.rng_f.x_hat, 1e-3)
        sigma_cam = max(FUSE_RANGE_SIGMA_FLOOR_M, FUSE_CAM_RANGE_FRAC * r_hat)
        sigma_cue_stat = FUSE_CUE_SIGMA_BASE_M + FUSE_CUE_SIGMA_QUAD * cue_range_m * cue_range_m
        sigma_cue = math.sqrt(sigma_cue_stat * sigma_cue_stat + FUSE_DATUM_BUDGET_M * FUSE_DATUM_BUDGET_M)
        w = min(1.0, (sigma_cam * sigma_cam) / max(1e-6, sigma_cue * sigma_cue))
        self.rng_f.correct(cue_range_m, t, gain_scale=w)

    def camera_fresh(self, t):
        return self.last_camera_t is not None and (t - self.last_camera_t) <= MEAS_STALE_S

    def target_vel(self, t, fallback):
        if (self.vel_ned is not None and self.vel_t is not None
                and (t - self.vel_t) <= FUSE_VEL_STALE_S):
            return self.vel_ned
        return fallback

    def state(self, t, own_ned, own_vel_ned, fallback_vel):
        """Fused snapshot (pos_ned, vel_ned, lam, R, rdot, lamdot) or None if
        the camera side hasn't initialized the polar track. Rates are
        GEOMETRIC from the fused target velocity (cue-emitted when fresh)
        minus own velocity:  rdot = (vt-vo).u,  lamdot = cross(u, vt-vo)/R
        -- not the alpha-beta's own still-converging rate states (the whole
        point of the warm handoff is that ~1-2 s of noisy camera corrections
        cannot match a transmitted filtered velocity)."""
        if not (self.lam.initialized and self.rng_f.initialized):
            return None
        lam = self.lam.x_hat
        R = max(self.rng_f.x_hat, 1e-3)
        u = (math.cos(lam), math.sin(lam))
        vt = self.target_vel(t, fallback_vel) or (0.0, 0.0)
        vo = own_vel_ned or (0.0, 0.0)
        vrel = (vt[0] - vo[0], vt[1] - vo[1])
        rdot = vrel[0] * u[0] + vrel[1] * u[1]
        lamdot = (u[0] * vrel[1] - u[1] * vrel[0]) / R
        pos = (own_ned[0] + R * u[0], own_ned[1] + R * u[1])
        return dict(pos=pos, vel=vt, lam=lam, R=R, rdot=rdot, lamdot=lamdot)

# Tracking-refinement PORT 2 (--cue-latency-comp): clamp bounds for the
# computed cue age (sim_now - t_sim), a defensive floor/ceiling against a
# clock-subscription hiccup or a wedged datagram producing a nonsensical
# value -- mirrors guidance_lab.py's Measurement.age_s usage, adapted to a
# DYNAMICALLY COMPUTED age (this real port measures sim_now - t_sim; the
# lab used its own known fixed cue_latency constant directly, since it has
# no wall/sim clock distinction to model -- see CueReader/SimClockHolder
# usage below for the real computation).
CUE_AGE_CLAMP_MIN_S = 0.0
CUE_AGE_CLAMP_MAX_S = 0.5


class SimClockHolder:
    """Latest sim time from /clock (tracking-refinement PORT 2,
    --cue-latency-comp): needed to compute a cue datagram's AGE (sim_now -
    t_sim) so a stale cue can be advanced along the tracker's velocity
    estimate before it corrects the shared TargetTracker.

    SAFE TO SUBSCRIBE HERE (unlike scripts/m4_target_mover.py, which must
    stay subscription-free): the gz-transport13 quirk found during M4
    (ADR-0009) is that a process holding ANY topic subscription never
    receives SERVICE RESPONSES again. m4_intercept.py makes ZERO gz service
    calls of its own -- the mover calls `/world/apriltag/set_pose` and the
    cue mock's own service surface (none) both live in their OWN separate
    processes -- so this subscription-only process is unaffected."""

    def __init__(self):
        self.t: Optional[float] = None

    def on_clock(self, msg: Clock) -> None:
        self.t = msg.sim.sec + msg.sim.nsec * 1e-9


class CueReader:
    """Background-thread UDP reader for scripts/s2_cue_mock.py's degraded
    external-cue datagrams (S2, ADR-0010 #4). Latest-cue-wins: mirrors
    m3_static_intercept.MeasurementHolder / LatestFrame -- the reader thread
    replaces one attribute reference per datagram, a single Python
    assignment that is atomic under the GIL, so the asyncio guidance loop
    can read `.read()` at any time with no lock.

    LATCH ONE-WAY (ADR-0010 #5, "illegal-state-unrepresentable"): `close()`
    stops the thread and closes the socket; the guidance loop then also
    sets its own `cue_reader` variable to `None` (see run_acquire_and_engage
    -- not done here, since this object can't un-reference itself from the
    outside). After that, the ONLY code that ever touches a CueReader (the
    ext-cue consumption block) guards on `cue_reader is not None` -- so a
    post-close cue is not silently "ignored by convention", it is
    STRUCTURALLY gone: the object handing it out no longer exists, and any
    future edit that forgets the None-check gets an immediate AttributeError
    on `None.read()` rather than a silent stale read.
    """

    def __init__(self, port: int):
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", port))
        self._sock.settimeout(0.5)
        # (t_recv_mono, t_sim, x, y, z, seq, vx, vy, vz) or None. vx/vy/vz are
        # None unless the mock was run with --emit-velocity (ADR-0015 table
        # #2); consumers MUST tolerate the None case (old-mock compatibility).
        self.latest = None
        self.n_received = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break  # socket closed out from under us (close()) -- exit quietly
            try:
                msg = json.loads(data.decode("utf-8"))
                # vx/vy/vz optional (ADR-0015 table #2). msg.get(...) is None
                # for the pre-ADR-0015 5-key datagram -> never a KeyError,
                # never a crash for the differentiate path.
                def _optf(key):
                    v = msg.get(key)
                    return None if v is None else float(v)
                cue = (
                    time.monotonic(), float(msg["t_sim"]),
                    float(msg["x"]), float(msg["y"]), float(msg["z"]),
                    int(msg["seq"]),
                    _optf("vx"), _optf("vy"), _optf("vz"),
                )
            except (ValueError, KeyError, UnicodeDecodeError):
                continue
            self.latest = cue
            self.n_received += 1

    def read(self):
        """Latest (t_recv_mono, t_sim, x, y, z, seq, vx, vy, vz) tuple, or None
        if no datagram has arrived yet. vx/vy/vz are None unless the mock ran
        with --emit-velocity (ADR-0015)."""
        return self.latest

    def close(self):
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)


class M4TelemetryState(TelemetryState):
    """TelemetryState plus the vehicle's own yaw (to reconstruct the inertial
    LOS azimuth lambda = psi + beta) and its own north/east position (for
    PIP's absolute target track). Both are OWN state (PX4 EKF), not target
    info -- the camera stays the only target sensor."""

    def __init__(self):
        super().__init__()
        self.yaw_deg: Optional[float] = None
        self.pos_n: Optional[float] = None
        self.pos_e: Optional[float] = None
        # Own NED velocity (P-6 fused/warm-handoff geometric rate init needs
        # it; own-state = legal). Same position_velocity_ned() stream as pos.
        self.vel_n: Optional[float] = None
        self.vel_e: Optional[float] = None


async def track_attitude(drone, state: "M4TelemetryState") -> None:
    async for euler in drone.telemetry.attitude_euler():
        state.yaw_deg = euler.yaw_deg


async def track_local_position(drone, state: "M4TelemetryState") -> None:
    async for pv in drone.telemetry.position_velocity_ned():
        state.pos_n = pv.position.north_m
        state.pos_e = pv.position.east_m
        state.vel_n = pv.velocity.north_m_s
        state.vel_e = pv.velocity.east_m_s


def acquire_command(detected: bool, meas: "Optional[Measurement]", alt_m, psi_deg):
    """ACQUIRE-phase command: hover + point the nose at the tag, NO closing
    (the tag is still stationary at this point -- see check_m4.sh, which
    pre-places it before this script even starts). Commands are NED velocity
    + ABSOLUTE yaw angle (see the "why NED" note in the module docstring):
    when the tag is visible, yaw setpoint = psi + beta (the tag's absolute
    azimuth); when it isn't, hold the current heading."""
    v_down = (
        _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
        if alt_m is not None else 0.0
    )
    psi = psi_deg if psi_deg is not None else 0.0
    if not detected:
        return 0.0, 0.0, v_down, psi
    yaw_deg = psi + math.degrees(meas.bearing_rad)
    return 0.0, 0.0, v_down, yaw_deg


def write_row_m4(
    writer, log_file, t: float, phase: str, law: str,
    detected: Optional[bool], meas: "Optional[Measurement]",
    psi_deg, lambda_rad, lambda_dot_rad_s, r_hat_m, rdot_hat_m_s,
    vc_m_s, a_cmd_m_s2, v_perp_m_s, cmd, alt_m, gt_cam, gt_tag, gt_range,
    ext_xyz=None, ext_fresh=None, tgt_state=None, ext_age_s=None,
    cue_stale=None, coast_phase=None, t_sim=None, ekf_state=None,
):
    def fmt(value, spec="{:.4f}"):
        return "" if value is None else spec.format(value)

    meas_xyz = meas.meas_xyz if (meas is not None and detected) else None
    meas_range = meas.range_m if (meas is not None and detected) else None
    bearing_rad = meas.bearing_rad if (meas is not None and detected) else None

    row = [
        f"{t:.3f}",
        fmt(t_sim, "{:.3f}"),
        phase,
        law,
        "" if detected is None else int(detected),
        fmt(meas_xyz[0]) if meas_xyz is not None else "",
        fmt(meas_xyz[1]) if meas_xyz is not None else "",
        fmt(meas_xyz[2]) if meas_xyz is not None else "",
        fmt(meas_range),
        fmt(bearing_rad, "{:.5f}"),
        fmt(psi_deg, "{:.3f}"),
        fmt(math.degrees(lambda_rad), "{:.3f}") if lambda_rad is not None else "",
        fmt(math.degrees(lambda_dot_rad_s), "{:.3f}") if lambda_dot_rad_s is not None else "",
        fmt(r_hat_m),
        fmt(rdot_hat_m_s),
        fmt(vc_m_s),
        fmt(a_cmd_m_s2),
        fmt(v_perp_m_s),
        fmt(cmd[0]) if cmd is not None else "",
        fmt(cmd[1]) if cmd is not None else "",
        fmt(cmd[2]) if cmd is not None else "",
        fmt(cmd[3]) if cmd is not None else "",
        fmt(alt_m),
        fmt(gt_cam[0]) if gt_cam is not None else "",
        fmt(gt_cam[1]) if gt_cam is not None else "",
        fmt(gt_cam[2]) if gt_cam is not None else "",
        fmt(gt_tag[0]) if gt_tag is not None else "",
        fmt(gt_tag[1]) if gt_tag is not None else "",
        fmt(gt_tag[2]) if gt_tag is not None else "",
        fmt(gt_range),
        fmt(ext_xyz[0]) if ext_xyz is not None else "",
        fmt(ext_xyz[1]) if ext_xyz is not None else "",
        fmt(ext_xyz[2]) if ext_xyz is not None else "",
        "" if ext_fresh is None else int(ext_fresh),
        fmt(tgt_state[0]) if tgt_state is not None else "",
        fmt(tgt_state[1]) if tgt_state is not None else "",
        fmt(tgt_state[2]) if tgt_state is not None else "",
        fmt(tgt_state[3]) if tgt_state is not None else "",
        fmt(ext_age_s, "{:.4f}"),
        "" if cue_stale is None else int(cue_stale),
        "" if coast_phase is None else coast_phase,
        # Fusion capstone (design D2/F4, ADR-0041): ekf_state is
        # (dn, de, dvn, dve, p00, p11, p22, p33) or None -- None on every
        # call site except run_acquire_and_engage's, and even there only
        # when --tracker ekf AND the EKF is initialized (existing
        # empty-field convention).
        fmt(ekf_state[0]) if ekf_state is not None else "",
        fmt(ekf_state[1]) if ekf_state is not None else "",
        fmt(ekf_state[2]) if ekf_state is not None else "",
        fmt(ekf_state[3]) if ekf_state is not None else "",
        fmt(ekf_state[4], "{:.5f}") if ekf_state is not None else "",
        fmt(ekf_state[5], "{:.5f}") if ekf_state is not None else "",
        fmt(ekf_state[6], "{:.5f}") if ekf_state is not None else "",
        fmt(ekf_state[7], "{:.5f}") if ekf_state is not None else "",
    ]
    writer.writerow(row)
    log_file.flush()


def recompute_min_gt_range_from_csv(log_path: str) -> float:
    """Independent recomputation of the miss distance directly from the
    just-written CSV (CLAUDE.md: "analyze from logs"). Should match the
    running-min tracked during flight -- see the cross-check in main()."""
    min_val = None
    with open(log_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            value = row.get("gt_range", "")
            if value:
                v = float(value)
                if min_val is None or v < min_val:
                    min_val = v
    return min_val if min_val is not None else float("nan")


def preplace_target(target_start: str, timeout_s: float = 6.0) -> bool:
    """ADR-0033 (M5 finish; fixes the ADR-0032 pre-placement race): teleport
    the AprilTag target to --target-start via a SHORT-LIVED SUBPROCESS running
    the `gz service` CLI (mirroring scripts/mc_batch.sh's external pre-place),
    BEFORE this process does any in-process gz-transport init.

    Why a subprocess and NOT node.request(): this process holds a gz-transport
    /clock subscription under --handoff (SimClockHolder), and the gz-transport13
    quirk documented on SimClockHolder means a process holding ANY subscription
    never receives gz service RESPONSES. More decisively, this must run before
    Node() even exists so the always-on camera detection thread never sees the
    stale world-file default-position board (~5 m away). A separate subprocess
    sidesteps the quirk entirely (the mover pattern).

    World name comes from the INTERCEPTOR_WORLD_NAME env override (default
    "apriltag", ADR-0032), matching m4_target_mover.py / s2_cue_mock.py.

    Non-fatal: returns False (with a warning) on any failure. mc_batch.sh's own
    external pre-place remains a valid guard and a second set_pose is idempotent,
    so a failure here is a warning, not an abort. Uses a wall-clock subprocess
    timeout purely as an outer process guard (not a sim-scheduled duration); the
    gz CLI's own --timeout 2000 ms fires first."""
    world_name = os.environ.get("INTERCEPTOR_WORLD_NAME", "apriltag")
    service = f"/world/{world_name}/set_pose"
    try:
        parts = [float(v) for v in target_start.split(",")]
        x, y, z = parts[0], parts[1], parts[2]
    except (ValueError, IndexError):
        print(
            f"[m4] WARNING: could not parse --target-start {target_start!r} for "
            "tag pre-placement; skipping (mover / mc_batch pre-place still applies)"
        )
        return False
    # Same request shape as mc_batch.sh: name + position, orientation left at
    # identity (the target model bakes its facing into geometry -- apriltag_target
    # via the tag mount, fpv_target_markerless via the body, both identity).
    # INTERCEPTOR_TARGET_MODEL retargets the pre-placed model for the markerless
    # A/B (ADR-0033 item 2); default "apriltag_target" keeps the gated path exact.
    model_name = os.environ.get("INTERCEPTOR_TARGET_MODEL", "apriltag_target")
    req = f'name: "{model_name}" position {{ x: {x} y: {y} z: {z} }}'
    cmd = [
        "gz", "service", "-s", service,
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    print(
        f"[m4] Pre-placing tag at ({x}, {y}, {z}) via {service} "
        "(subprocess gz service CLI, before any gz-transport init -- ADR-0033)..."
    )
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout_s
        )
    except FileNotFoundError:
        print(
            "[m4] WARNING: 'gz' CLI not found; skipping tag pre-placement "
            "(mover / mc_batch external pre-place still applies)"
        )
        return False
    except subprocess.TimeoutExpired:
        print(
            f"[m4] WARNING: tag pre-placement timed out after {timeout_s}s; "
            "continuing (mover / mc_batch external pre-place still applies)"
        )
        return False
    if result.returncode != 0:
        print(
            f"[m4] WARNING: tag pre-placement gz service returned "
            f"{result.returncode}; continuing (mover / mc_batch external "
            f"pre-place still applies). stderr: {result.stderr.strip()}"
        )
        return False
    print("[m4] Tag pre-placed.")
    return True


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--law", choices=["pursuit", "pronav", "pip"], required=True,
        help="guidance law to run (pip = predicted intercept point, ADR-0011)",
    )
    # target-start / target-vel / mover-duration default to None here (not a
    # literal value) so parse_args() below can pick the M4/S1 short-standoff
    # defaults when --handoff is absent (byte-for-byte unchanged from before
    # S2) or the S2 long-standoff dash-runway defaults when --handoff is set
    # -- WITHOUT changing behavior for anyone who explicitly passes any of
    # these three flags either way.
    parser.add_argument(
        "--target-start", default=None,
        help="tag start position 'x,y,z' (m) -- forwarded to m4_target_mover.py "
             "(default: 6.5,-4,0.5, or 6.5,-14,0.5 under --handoff)",
    )
    parser.add_argument(
        "--target-vel", default=None,
        help="tag velocity 'vx,vy' (m/s) -- forwarded to m4_target_mover.py "
             "(default: 0,2.0, or 0,6.0 under --handoff)",
    )
    parser.add_argument(
        "--no-preplace", action="store_true",
        help="ADR-0033 (default OFF, i.e. internal pre-placement ON): skip the "
             "one-shot 'gz service .../set_pose' subprocess that teleports the "
             "tag to --target-start BEFORE any in-process gz-transport init, so "
             "the always-on camera detection thread cannot lock the world-file "
             "default-position board first (the ADR-0032 race). Only fires when "
             "--target-start is EXPLICITLY given; harmless/idempotent alongside "
             "mc_batch.sh's own external pre-place. Set this to opt out.",
    )
    parser.add_argument(
        "--require-miss", type=float, default=None,
        help="exit nonzero if the final miss distance is >= this (meters)",
    )
    parser.add_argument(
        "--bench", action="store_true",
        help="run the sign-convention bench check instead of an intercept",
    )
    parser.add_argument(
        "--mover-duration", type=float, default=None,
        help="how long the target mover streams motion for, in sim seconds "
             "(default: 12.0, or 20.0 under --handoff)",
    )
    parser.add_argument(
        "--fpv", action="store_true",
        help="FPV profile (S1, ADR-0010): bump PX4 params, two-speed closing "
             "law, rescaled terminal ranges for a faster (~6 m/s) target",
    )
    parser.add_argument(
        "--handoff", action="store_true",
        help="S2 two-stage external-cue handoff (ADR-0010 #4/#5): requires "
             "--fpv. CUE_WAIT (external cue) -> DASH (running start) -> "
             "HANDOFF -> the existing camera-only ENGAGE/BREAKOFF logic.",
    )
    parser.add_argument(
        "--dash-speed", type=float, default=None,
        help="override S2 DASH_SPEED (m/s, default %.1f)" % S2["DASH_SPEED"],
    )
    parser.add_argument(
        "--handoff-range", type=float, default=None,
        help="override S2 HANDOFF_RANGE_M (m, default %.1f)" % S2["HANDOFF_RANGE_M"],
    )
    parser.add_argument(
        "--cue-port", type=int, default=None,
        help="override S2 CUE_PORT (default %d)" % S2["CUE_PORT"],
    )
    parser.add_argument(
        "--cue-seed", type=int, default=0,
        help="seed forwarded to scripts/s2_cue_mock.py (reproducibility)",
    )
    parser.add_argument(
        "--kalata", action="store_true",
        help="tracking-refinement PORT 1 (default OFF): recompute the "
             "lambda/range alpha-beta filters' gains from the Kalata "
             "tracking index at the ACTUAL dt since the last correction, "
             "instead of the fixed ALPHA/BETA_GAIN_* constants. Usable with "
             "plain --fpv (no --handoff needed) or under --handoff.",
    )
    parser.add_argument(
        "--cue-latency-comp", action="store_true",
        help="tracking-refinement PORT 2 (default OFF, requires --handoff): "
             "advance a cue reading along the shared tracker's current "
             "velocity estimate by its known sim-time age before correcting "
             "(compensates the cue mock's fixed latency + delivery jitter).",
    )
    parser.add_argument(
        "--cue-velocity", action="store_true",
        help="ADR-0015 table #2 (default OFF, requires --handoff): when the "
             "cue datagrams carry vx/vy/vz (mock --emit-velocity), set the "
             "shared TargetTracker's velocity states DIRECTLY from the cue "
             "velocity (world->NED) instead of differentiating the noisy "
             "position stream. No-ops safely if the mock sends no velocity.",
    )
    parser.add_argument(
        "--coast-search", action="store_true",
        help="ADR-0015 (default OFF, requires --handoff): link-loss-before-"
             "lock branch. If the cue goes stale during DASH, dead-reckon the "
             "track and keep dashing; inside predicted acquisition range hold "
             "a bounded yaw-sweep seeker search; if no camera acquisition "
             "within the sim-time budget, BREAKOFF (outcome=link_lost_no_acq).",
    )
    parser.add_argument(
        "--fuse-midcourse", action="store_true",
        help="P-6 (default OFF, requires --handoff): mid-course sensor fusion "
             "(ADR-0015). Through the window where BOTH sources exist (camera "
             "detecting + cue alive, pre-latch), aim the DASH off a "
             "bearing-weighted fused track (camera owns the LOS angle; the cue "
             "range is folded in inverse-variance-weighted; cue velocity used "
             "when emitted) instead of the raw cue Cartesian track. POST-latch "
             "the cue stays closed one-way and the terminal is camera-only "
             "(unchanged). Lab: helps track continuity under EXPECTED realism, "
             "neutral/sub-noise on mean miss, inert under an early jammer "
             "cutoff -- Gazebo A/B decides (NEXT.md P-6).",
    )
    parser.add_argument(
        "--warm-handoff", action="store_true",
        help="P-6 (default OFF, requires --handoff): at the HANDOFF latch, "
             "warm-initialize the terminal filters (lambda, lambda_dot, R, "
             "Rdot, v_perp) and the shared target track from the fused (or, "
             "without --fuse-midcourse, the dash) track -- a WARM lock-transfer "
             "(ADR-0015) instead of letting the camera-only terminal filters "
             "converge from cold. Rates are set geometrically from the "
             "(cue-emitted when fresh) target velocity.",
    )
    parser.add_argument(
        "--early-handoff", action="store_true",
        help="ADR-0023 Tier-1 lever A (default OFF, requires --handoff): "
             "trigger the one-way HANDOFF latch on a %d-detection solid "
             "streak instead of %d -- engage the camera-only terminal "
             "earlier (and close the cue channel SOONER; the ADR-0010 #5 "
             "one-way latch semantics are unchanged)."
             % (EARLY_HANDOFF_STREAK_MIN, S2["HANDOFF_STREAK_MIN"]),
    )
    parser.add_argument(
        "--split-freeze", action="store_true",
        help="ADR-0023 Tier-1 lever B / ADR-0014 lever 2 (default OFF, "
             "requires --fpv and --law pronav): freeze ONLY the v_perp "
             "magnitude in the terminal (v_close/yaw/lambda_hat stay live), "
             "cap |lambda_dot| at %.0f deg/s in both a_cmd and the filter "
             "predict, and move the freeze later (%.1f -> %.1f m)."
             % (SPLIT_FREEZE_LAMBDA_DOT_CAP_DEG_S,
                FPV["TERMINAL_FREEZE_RANGE_M"], SPLIT_FREEZE_RANGE_M),
    )
    parser.add_argument(
        "--terminal-gain-scale", type=float, default=1.0,
        help="ADR-0043 lever B (default 1.0 = byte-identical): scale the "
             "lambda-filter correction gain during ENGAGE only -- rolls off "
             "the markerless box-center bearing-noise throughput (measured "
             "p90 2.8 deg/tick vs the tag's subpixel corners, ADR-0042). "
             "alpha-beta only; the EKF channel view ignores gain_scale by "
             "design (it weights by covariance).")
    parser.add_argument(
        "--terminal-freeze-range", type=float, default=None,
        help="ADR-0043 lever C (default None = FPV profile value %.1f m, "
             "requires --fpv): override TERMINAL_FREEZE_RANGE_M -- freeze "
             "the terminal velocity vector EARLIER so the endgame is "
             "ballistic through the bearing-noise band."
             % FPV["TERMINAL_FREEZE_RANGE_M"],)
    parser.add_argument(
        "--accel-boost", action="store_true",
        help="ADR-0028 Gazebo-confirm (default OFF, requires --fpv): ~2x "
             "the FPV PX4 param bundle's MPC_ACC_HOR_MAX (%.1f -> %.1f m/s^2) "
             "and MPC_TILTMAX_AIR (%.1f -> %.1f deg) -- the 'airframe "
             "agility' lever, separate from the running-start engagement-"
             "geometry lever (--dash-speed + mc_batch.sh --y0-mag/--x0). "
             "The x500 may be tilt/thrust-limited below the commanded "
             "ceiling; report ACHIEVED accel from telemetry, not the setpoint."
             % (FPV["PX4_PARAMS"]["MPC_ACC_HOR_MAX"], ACCEL_BOOST_MPC_ACC_HOR_MAX,
                FPV["PX4_PARAMS"]["MPC_TILTMAX_AIR"], ACCEL_BOOST_MPC_TILTMAX_AIR),
    )
    parser.add_argument(
        "--dash-unclamp", action="store_true",
        help="ADR-0028 addendum follow-up (default OFF, requires --fpv): "
             "raise FPV['V_TOTAL_MAX'] %.1f -> %.1f m/s. The DASH phase "
             "commands a unit vector scaled to S2['DASH_SPEED'] then clamps "
             "it to V_TOTAL_MAX -- at --dash-speed 16 the default 13 m/s "
             "ceiling silently clamps the dash below its intended speed "
             "while the lead solve still assumes 16. This flag removes that "
             "clamp for the running-start dash (does not touch V_PERP_MAX, "
             "the separate terminal pro-nav lateral clamp)."
             % (FPV["V_TOTAL_MAX"], DASH_UNCLAMP_V_TOTAL_MAX),
    )
    parser.add_argument(
        "--tracker", choices=["alphabeta", "ekf"], default="alphabeta",
        help="pro-nav LOS/range estimator (default alphabeta = the frozen "
             "M4/M5 fixed-gain g-h pair, byte-identical to HEAD). 'ekf' swaps "
             "in the Cartesian constant-velocity EKF (scripts/ekf_tracker.py, "
             "ADR-0033 item 3): a covariance-carrying drop-in for the two "
             "scalar filters, gated for the pre-registered paired-seed A/B in "
             "docs/ekf_design_brief.md (expect NULL on miss / possible signal "
             "on LOS-rate error). Alpha-beta-only knobs (--kalata, "
             "--split-freeze rate cap) do not apply under --tracker ekf.",
    )
    parser.add_argument(
        "--seeker", choices=["apriltag", "markerless"], default="apriltag",
        help="detection SOURCE for the guidance Measurement (default apriltag = "
             "byte-identical to HEAD: pupil-apriltags on the fiducial). "
             "'markerless' (ADR-0033 item 2) swaps ONLY the detection source to "
             "the two-stage markerless seeker (scripts/seeker/two_stage_seeker.py: "
             "classical dark-blob proposal -> NN verify) reading the target BODY, "
             "not a marker; the guidance/tracker/pro-nav math is unchanged. Needs "
             "onnxruntime + opencv-contrib from .venv-seeker (NOT the main .venv) "
             "and RE-EARNS the no-cheat audit at the live Gazebo A/B (NEXT step).",
    )
    args = parser.parse_args()

    if args.handoff and not args.fpv:
        parser.error("--handoff requires --fpv (S2 is an FPV-speed sub-step, ADR-0010)")
    if args.cue_latency_comp and not args.handoff:
        parser.error("--cue-latency-comp requires --handoff (there is no cue channel without it)")
    if args.cue_velocity and not args.handoff:
        parser.error("--cue-velocity requires --handoff (there is no cue channel without it)")
    if args.coast_search and not args.handoff:
        parser.error("--coast-search requires --handoff (there is no cue link to lose without it)")
    if args.fuse_midcourse and not args.handoff:
        parser.error("--fuse-midcourse requires --handoff (P-6 fuses the mid-course cue)")
    if args.warm_handoff and not args.handoff:
        parser.error("--warm-handoff requires --handoff (P-6 warm-transfers at the cue handoff)")
    if args.early_handoff and not args.handoff:
        parser.error("--early-handoff requires --handoff (there is no HANDOFF latch without it)")
    if args.split_freeze and not args.fpv:
        parser.error("--split-freeze requires --fpv (it retunes the FPV terminal-freeze semantics)")
    if not (0.0 < args.terminal_gain_scale <= 1.0):
        parser.error("--terminal-gain-scale must be in (0, 1] (1.0 = off/byte-identical)")
    if args.terminal_freeze_range is not None and not args.fpv:
        parser.error("--terminal-freeze-range requires --fpv (it overrides the FPV freeze constant)")
    if args.terminal_freeze_range is not None and args.terminal_freeze_range <= 0:
        parser.error("--terminal-freeze-range must be positive")
    if args.split_freeze and args.law != "pronav":
        parser.error("--split-freeze requires --law pronav (it freezes the pro-nav v_perp "
                     "term specifically; the lab A/B was pure_pn-only)")
    if args.accel_boost and not args.fpv:
        parser.error("--accel-boost requires --fpv (it retunes the FPV PX4 param bundle)")
    if args.dash_unclamp and not args.fpv:
        parser.error("--dash-unclamp requires --fpv (it retunes the FPV V_TOTAL_MAX clamp)")

    # ADR-0033 (M5 finish): remember whether the caller EXPLICITLY passed
    # --target-start BEFORE we fill in a default just below. The internal tag
    # pre-placement (main(), before any gz-transport init) fires ONLY for an
    # explicitly-provided pose so it never invents one for the default
    # M4/S1/S2 gated callers -- their behavior stays byte-for-byte unchanged.
    args.target_start_provided = args.target_start is not None
    if args.target_start is None:
        args.target_start = S2["TARGET_START_DEFAULT"] if args.handoff else "6.5,-4,0.5"
    if args.target_vel is None:
        args.target_vel = S2["TARGET_VEL_DEFAULT"] if args.handoff else "0,2.0"
    if args.mover_duration is None:
        args.mover_duration = S2["MOVER_DURATION_DEFAULT_S"] if args.handoff else 12.0

    return args


async def run_acquire_and_engage(
    drone, state, meas_holder, tracker, writer, log_file, started, args, s2params=None,
    sim_clock=None,
):
    """Non-handoff (default): ACQUIRE (hover + yaw-center on the static tag)
    -> ENGAGE (spawn the mover, run the chosen guidance law) -> BREAKOFF
    (climb off, keep logging) or an abort.

    Handoff (S2, --handoff, ADR-0010 #4/#5): CUE_WAIT (hover, wait on the
    external cue mock instead of the camera) -> DASH (lead-pursuit toward
    the cue-fed shared TargetTracker at S2_SPEED, a "running start") ->
    HANDOFF (camera acquires within range; cue channel latched closed) ->
    the SAME, UNMODIFIED ENGAGE/BREAKOFF code as the non-handoff path (the
    shared TargetTracker -- fed by the cue pre-handoff, camera-only after
    -- is what carries the "running start" through into ENGAGE; PIP's law
    reads that same tracker either way, now warm-started instead of cold).

    See the module docstring for the guidance law and the CLI spec for the
    exact phase/trigger semantics. Returns a dict of results consumed by
    main() to print the summary and decide the process exit code.
    """
    dt = 1.0 / CONTROL_RATE_HZ

    # --kalata (tracking-refinement PORT 1, default OFF): swap the lambda/
    # range channels' FIXED alpha/beta for gains recomputed every
    # correction from the Kalata tracking index at the actual dt (see
    # kalata_alpha_beta()). kalata_kwargs stays empty (both filters fall
    # back to fixed ALPHA/BETA_GAIN_* exactly as before) unless --kalata.
    lambda_kalata_kwargs = {}
    range_kalata_kwargs = {}
    if args.kalata:
        lambda_kalata_kwargs = dict(
            kalata_sigma_process=math.radians(KALATA_LAMBDA_SIGMA_PROCESS_DEG_S2),
            kalata_sigma_meas=math.radians(KALATA_LAMBDA_SIGMA_MEAS_DEG),
            label="lambda",
        )
        range_kalata_kwargs = dict(
            kalata_sigma_process=KALATA_RANGE_SIGMA_PROCESS_M_S2,
            kalata_sigma_meas=KALATA_RANGE_SIGMA_MEAS_M,
            label="range",
        )
    # --split-freeze (Tier-1 lever B): the lambda_dot cap lives IN the
    # filter (rate_cap), so a_cmd's read and predict()'s forward integration
    # are capped in one place -- see AlphaBetaFilter.rate_cap's docstring
    # for why capping only a_cmd recreates the ADR-0009 whipsaw. None
    # (flag off) => byte-identical baseline filter.
    lambda_rate_cap = (
        math.radians(SPLIT_FREEZE_LAMBDA_DOT_CAP_DEG_S) if args.split_freeze else None
    )
    lambda_filter = AlphaBetaFilter(
        ALPHA, BETA_GAIN_LAMBDA, angular=True, rate_cap=lambda_rate_cap,
        **lambda_kalata_kwargs,
    )
    range_filter = AlphaBetaFilter(ALPHA, BETA_GAIN_RANGE, angular=False, **range_kalata_kwargs)
    # --tracker ekf (ADR-0033 item 3, docs/ekf_design_brief.md): swap the
    # fixed-gain g-h pair for the Cartesian constant-velocity EKF, which
    # exposes lambda_filter/range_filter VIEWS mirroring the AlphaBetaFilter
    # surface (predict/correct/x_hat/xdot_hat/_last_correction_t) so the rest
    # of the loop is unchanged. Default 'alphabeta' leaves the two lines above
    # untouched and this branch a no-op -> the M4/M5/S2 gate path is
    # byte-identical to HEAD. The EKF constructed above is discarded here.
    # ekf_tracker (the EKFTracker instance, not to be confused with the
    # `tracker` parameter -- the gz AprilTag/ground-truth handle) stays None
    # under alphabeta so downstream P1/P2 fusion-capstone code (cue routing,
    # the DASH aim gate, the handoff latch, CSV/S2_RESULT logging) can guard
    # on `ekf_tracker is not None` without a NameError.
    ekf_tracker = None
    if args.tracker == "ekf":
        from ekf_tracker import EKFTracker
        ekf_tracker = EKFTracker()
        lambda_filter, range_filter = ekf_tracker.lambda_filter, ekf_tracker.range_filter
    # PIP (ADR-0011) / S2 dash tracker: absolute target position+velocity
    # track. Used by --law pip's lead solve AND (under --handoff) by the
    # DASH phase's own lead-pursuit -- ONE instance, shared across every
    # phase (spec requirement): predicts every tick, corrects on fresh
    # camera detections (own_ned + camera relative vector, existing PIP
    # code path below) with camera taking priority, and -- pre-handoff
    # only, --handoff mode only -- on ticks with a new cue datagram and no
    # fresh camera detection this tick (see the ext-cue block below).
    target_tracker = TargetTracker(PIP_TRACK_ALPHA, PIP_TRACK_BETA)

    # P-6 (--fuse-midcourse / --warm-handoff): a bearing-weighted polar fused
    # track maintained through the both-sources mid-course window. None (and
    # every branch below no-ops) unless a P-6 flag is set -> default path
    # byte-identical. Fed camera (angle+range) in the fresh block and cue
    # (range+velocity) in the ext-cue block, PRE-latch only.
    # Fusion capstone (design D2, ADR-0041): under tracker=ekf AND
    # fuse-midcourse, the EKF's OWN covariance-gated correct_cue replaces
    # FusedTrack for the mid-course fusion mechanism ("no double-fusion") --
    # FusedTrack is not constructed in that one combination. Every other
    # combination (alphabeta+fuse/warm, ekf+warm-handoff-only, ekf+neither)
    # is byte-identical to before this change.
    fused = (
        None if (args.tracker == "ekf" and args.fuse_midcourse)
        else (FusedTrack() if (args.fuse_midcourse or args.warm_handoff) else None)
    )

    phase = "CUE_WAIT" if args.handoff else "ACQUIRE"
    acquire_start_mono = time.monotonic()
    consecutive_fresh = 0
    centered_streak = 0
    last_meas_t_mono = None

    v_perp = 0.0
    vh0 = vh1 = 0.0
    frozen_vworld = None
    # --split-freeze (Tier-1 lever B) one-way latch state: once r_hat first
    # dips under the (relocated) freeze range, the v_perp MAGNITUDE freezes
    # (integration stops) but the command keeps being reconstructed each
    # detected tick from the LIVE lambda_hat/v_close. frozen_vworld is never
    # set on this path.
    split_frozen = False
    last_cmd = (0.0, 0.0, 0.0, 0.0)
    mover_proc = None
    engage_t0 = None

    breakoff_reason = ""
    breakoff_entry_mono = None
    breakoff_armed = False
    range_increase_streak = 0
    last_fresh_range = None
    dropout_start_mono = None
    lost_since_mono = None

    aborted = False
    abort_reason = ""
    n_ticks = 0
    n_detected_ticks = 0
    min_gt_range_running = None

    # --- S2 (--handoff) state. cue_proc is the s2_cue_mock.py subprocess
    # (left running past HANDOFF -- audit evidence, killed at run end like
    # the mover); cue_reader is this process's UDP receiver, LATCHED to
    # None the instant HANDOFF triggers (ADR-0010 #5, see CueReader's
    # docstring for why nulling the reference is the enforcement). ---
    cue_proc = None
    cue_reader = None
    cue_wait_start_mono = time.monotonic()
    dash_start_mono = None
    last_cue_seq_seen = None
    tracker_correction_count = 0
    handoff_streak = 0
    handoff_done = False
    handoff_t = None
    handoff_range_at_trigger = None
    first_dash_detection_range = None
    # ADR-0015 coast-search (link-loss-before-lock) state. last_cue_recv_sim_t
    # is the sim time of the most recent NEW cue datagram (staleness clock);
    # coast_active latches when the cue goes stale during DASH and resets if
    # the link recovers; outcome is the S2_RESULT tag (ADR-0015).
    last_cue_recv_sim_t = None
    coast_active = False
    coast_loss_sim_t = None
    outcome = "no_handoff"
    if args.handoff:
        cue_reader = CueReader(s2params["CUE_PORT"])
        cue_cmd = [
            sys.executable, CUE_MOCK_SCRIPT,
            "--port", str(s2params["CUE_PORT"]),
            "--seed", str(args.cue_seed),
            "--duration", str(s2params["CUE_MOCK_RUN_DURATION_S"]),
            "--sigma", str(s2params["CUE_MOCK_SIGMA_M"]),
            "--latency-s", str(s2params["CUE_MOCK_LATENCY_S"]),
            "--rate", str(s2params["CUE_MOCK_RATE_HZ"]),
        ]
        # ADR-0015 dev-flight passthrough (env var so it survives check_s2.sh's
        # word-splitting; empty => byte-identical spawn, S2 gate untouched).
        cue_extra = os.environ.get(CUE_MOCK_EXTRA_ENV, "").strip()
        if cue_extra:
            cue_cmd.extend(shlex.split(cue_extra))
            print(f"[s2] cue mock extra ({CUE_MOCK_EXTRA_ENV}): {cue_extra}")
        print(f"[s2] cue mock: {' '.join(cue_cmd)}")
        cue_proc = subprocess.Popen(cue_cmd, stdout=None, stderr=None)

    while True:
        tick_start = time.monotonic()
        detected, meas = sample_measurement(meas_holder)
        # new_meas: the detection thread produced a NEW result since our last
        # tick (whether or not it contains a detection). fresh: that new
        # result IS a detection (usable for filter correction). The two are
        # distinct on purpose: detection takes ~70 ms/frame here (measured
        # 2026-07-05, dev run m4_intercept_pronav_...T011633Z), i.e. ~14 Hz
        # against this 20 Hz loop, so "no new result this tick" is NORMAL
        # cadence, not a miss -- only a new result that failed to detect
        # counts against acquisition.
        new_meas = last_meas_t_mono is None or meas.t_mono != last_meas_t_mono
        fresh = detected and new_meas
        if new_meas:
            last_meas_t_mono = meas.t_mono

        psi_deg = state.yaw_deg
        psi_rad = math.radians(psi_deg) if psi_deg is not None else 0.0
        alt_m = state.relative_altitude_m

        # --- filters: predict every tick, correct only on FRESH ticks ---
        lambda_filter.predict(dt)
        range_filter.predict(dt)
        target_tracker.predict(dt)
        if fused is not None:
            fused.predict(dt)
        if fresh:
            lambda_meas = psi_rad + meas.bearing_rad
            # ADR-0043 lever B: roll off the bearing-noise throughput in the
            # terminal ONLY (gain 1.0 elsewhere and by default). No-op under
            # --tracker ekf (the channel view ignores gain_scale by design).
            lambda_filter.correct(
                lambda_meas, tick_start,
                gain_scale=(args.terminal_gain_scale
                            if phase == "ENGAGE" else 1.0),
            )
            range_filter.correct(meas.range_m, tick_start)
            # PIP absolute target track (ADR-0011): own NED position (EKF,
            # own-state) + the camera relative vector in NED. The relative
            # vector uses the SAME bench-validated LOS azimuth psi+beta the
            # pronav command frame uses: north = range*cos(lambda),
            # east = range*sin(lambda). Target info stays camera-only.
            if state.pos_n is not None and state.pos_e is not None:
                rel_n = meas.range_m * math.cos(lambda_meas)
                rel_e = meas.range_m * math.sin(lambda_meas)
                target_tracker.correct(
                    (state.pos_n + rel_n, state.pos_e + rel_e), tick_start
                )
                tracker_correction_count += 1
            # P-6: camera owns the fused angle + full-gain range (lambda_meas
            # is the inertial LOS psi+beta, exactly as the guidance filter).
            if fused is not None:
                fused.update_camera(lambda_meas, meas.range_m, tick_start)

        # --- S2 (--handoff) external-cue consumption: CAMERA HAS PRIORITY
        # (ADR-0010 #5) -- only feed the shared tracker from the cue on a
        # tick with a genuinely NEW cue datagram (mirrors the new_meas/fresh
        # distinction above) AND no fresh camera correction THIS tick, and
        # only pre-handoff (cue_reader is None once HANDOFF latches it
        # closed -- see the DASH branch below). ext_n/e/z + ext_fresh are
        # for the CSV only: populated exactly on ticks where a cue reading
        # actually corrected the tracker, blank otherwise (CSV_HEADER).
        ext_n = ext_e = ext_z = None
        ext_fresh = 0
        ext_age_s = None
        cue_stale_flag = None   # ADR-0015 coast-search per-tick CSV markers
        coast_phase_tick = None
        if args.handoff and cue_reader is not None:
            cue = cue_reader.read()
            if cue is not None:
                # 9-tuple (ADR-0015): vx/vy/vz are None unless the mock ran
                # with --emit-velocity (CueReader tolerates the old 5-key
                # datagram). WORLD velocity, mapped to NED like position below.
                _t_recv_mono, t_sim_cue, cx, cy, cz, cseq, cvx, cvy, cvz = cue
                cue_is_new = last_cue_seq_seen is None or cseq != last_cue_seq_seen
                if cue_is_new:
                    last_cue_seq_seen = cseq
                    # Cue is alive this tick: reset the staleness clock, and if
                    # we were coasting on a stale link that has now recovered,
                    # drop back to normal cue-guided dash (ADR-0015).
                    if sim_clock is not None and sim_clock.t is not None:
                        last_cue_recv_sim_t = sim_clock.t
                    if coast_active:
                        print("[s2] COAST-SEARCH: cue link recovered -- resuming cue-guided dash")
                        coast_active = False
                        coast_loss_sim_t = None
                    if not fresh:
                        # WORLD -> NED mapping (empirically verified -- see
                        # the module docstring's "S2 WORLD-FRAME MAPPING"
                        # section for the evidence/log filenames): the cue
                        # reports Gazebo WORLD x, y, z; north = world_y,
                        # east = world_x (no sign flip). GOALS.md's world
                        # convention ("origin at the interceptor's start")
                        # means no translation term is needed either.
                        ext_n = cy
                        ext_e = cx
                        ext_z = cz
                        # Tracking-refinement PORT 2 (--cue-latency-comp,
                        # default OFF): the cue's AGE at USE is sim_now -
                        # t_sim (includes the mock's fixed 0.12s latency
                        # PLUS any real delivery jitter -- a genuine measured
                        # quantity, unlike guidance_lab.py's lab, which used
                        # its own known fixed cue_latency constant directly
                        # since it has no wall/sim distinction to model).
                        # Logged (ext_age_s) whenever a clock sample exists,
                        # regardless of the flag, for audit; ONLY applied
                        # (position advanced along the tracker's own current
                        # velocity estimate before correcting -- ported from
                        # guidance_lab.py's
                        # TargetTracker.correct_full(latency_compensate=True))
                        # when --cue-latency-comp is set AND the tracker
                        # already has a velocity estimate to advance along.
                        if sim_clock is not None and sim_clock.t is not None:
                            ext_age_s = _clamp(
                                sim_clock.t - t_sim_cue, CUE_AGE_CLAMP_MIN_S, CUE_AGE_CLAMP_MAX_S
                            )
                        corr_n, corr_e = ext_n, ext_e
                        if args.cue_latency_comp and ext_age_s is not None and ext_age_s > 0.0:
                            vel_hat = target_tracker.vel_hat
                            if vel_hat is not None:
                                corr_n = ext_n + vel_hat[0] * ext_age_s
                                corr_e = ext_e + vel_hat[1] * ext_age_s
                        target_tracker.correct((corr_n, corr_e), tick_start)
                        # ADR-0015 table #2 (--cue-velocity): overwrite the
                        # tracker's velocity states DIRECTLY from the cue's
                        # own filtered velocity (world->NED: vn=world vy,
                        # ve=world vx) instead of trusting the alpha-beta
                        # differentiation of a noisy position stream. Guarded
                        # on the cue actually carrying velocity (old-mock safe).
                        if args.cue_velocity and cvx is not None and cvy is not None:
                            target_tracker.set_velocity(cvy, cvx)
                        # P-6: fold the cue-implied range (bearing-weighted;
                        # the cue NEVER touches the fused angle) + cue velocity
                        # (world->NED: vn=world vy, ve=world vx) into the fused
                        # track. Uses the latency-compensated corr_n/corr_e so
                        # the fused range matches the tracker's own correction.
                        if fused is not None and state.pos_n is not None and state.pos_e is not None:
                            cue_rng = math.hypot(corr_n - state.pos_n, corr_e - state.pos_e)
                            cue_vel_ned = (
                                (cvy, cvx) if (cvx is not None and cvy is not None) else None
                            )
                            fused.update_cue(cue_rng, cue_vel_ned, tick_start)
                        # Fusion capstone (design D2, ADR-0041): under
                        # tracker=ekf + fuse-midcourse, route this SAME
                        # accepted cue datagram into the EKF's own
                        # covariance-gated correct_cue (polar-split -- "the
                        # cue never touches the angle", ekf_tracker.py)
                        # instead of FusedTrack (not constructed in this
                        # combination -- see `fused` above). Only once the
                        # EKF is camera-initialized, mirroring
                        # FusedTrack.update_cue's own initialized-gate (a
                        # biased cue must never define the camera-anchored
                        # track's initial state). World->NED->RELATIVE via
                        # own EKF2 position/velocity (legal, honesty
                        # boundary) -- north=world_y east=world_x, same
                        # mapping already used for ext_n/ext_e above.
                        if (
                            ekf_tracker is not None and args.fuse_midcourse
                            and ekf_tracker.initialized
                            and state.pos_n is not None and state.pos_e is not None
                        ):
                            rel_pos = (corr_n - state.pos_n, corr_e - state.pos_e)
                            rel_vel = None
                            if (
                                cvx is not None and cvy is not None
                                and getattr(state, "vel_n", None) is not None
                            ):
                                rel_vel = (cvy - state.vel_n, cvx - state.vel_e)
                            ekf_tracker.correct_cue(rel_pos, rel_vel, tick_start)
                        tracker_correction_count += 1
                        ext_fresh = 1

        lambda_hat = lambda_filter.x_hat
        lambda_dot_hat = lambda_filter.xdot_hat if lambda_filter.initialized else None
        r_hat = range_filter.x_hat
        rdot_hat = range_filter.xdot_hat if range_filter.initialized else None
        # Vc floor raised 0.3 -> 1.5 after dev run ...T012356Z: the range-rate
        # filter starts at ~0 from hover (and lags under ~50% detection
        # coverage), so a 0.3 m/s floor let a_cmd = N*Vc*lambda_dot build the
        # lead at one-fifth strength for the first ~3 s -- the target's LOS
        # walked out of the FOV before the lead caught up. 1.5 m/s is a
        # defensible prior: the closing law commands a constant V_CLOSE_MAX
        # from engage onward, so true closing speed is never near zero once
        # engaged. Measured -Rdot_hat takes over the
        # moment it exceeds the floor.
        VC_FLOOR_M_S = FPV["VC_FLOOR_M_S"] if args.fpv else 1.5
        vc = max(VC_FLOOR_M_S, -rdot_hat) if rdot_hat is not None else VC_FLOOR_M_S
        a_cmd = (
            N_PRONAV * vc * lambda_dot_hat
            if lambda_dot_hat is not None else 0.0
        )

        if r_hat is not None and r_hat < BREAKOFF_ARM_RANGE_M:
            breakoff_armed = True

        if phase == "ACQUIRE":
            cmd = acquire_command(detected, meas, alt_m, psi_deg)
            # Streak counts consecutive DETECTIONS in the detection stream:
            # a tick with no new detector result leaves it untouched (see
            # new_meas comment above); only a new result WITHOUT a tag
            # resets it. (First version counted per control tick and could
            # never reach 10 -- the 20 Hz loop outpaces ~14 Hz detection.)
            if new_meas:
                consecutive_fresh = consecutive_fresh + 1 if meas.range_m is not None else 0
            acquire_elapsed = tick_start - acquire_start_mono

            # Centered must be SETTLED (a streak of fresh detections), not a
            # momentary zero-crossing mid-slew: dev run ...T012819Z engaged
            # with beta ~4 deg but ~+16 deg/s of residual yaw rate, and the
            # engagement started in a hole. A stable streak of centered
            # detections implies both small beta AND ~zero yaw rate (static
            # tag: beta steady <=> nose steady on the LOS).
            if fresh:
                if abs(math.degrees(meas.bearing_rad)) <= ACQUIRE_MAX_BEARING_DEG:
                    centered_streak += 1
                else:
                    centered_streak = 0
            if (
                consecutive_fresh >= ACQUIRE_MIN_DETECTIONS
                and acquire_elapsed >= ACQUIRE_MIN_S
                and centered_streak >= ACQUIRE_CENTERED_STREAK
            ):
                print(
                    f"[m4] Acquired at t={acquire_elapsed:.2f}s "
                    f"({consecutive_fresh} consecutive fresh detections). "
                    "Spawning mover and engaging..."
                )
                phase = "ENGAGE"
                engage_t0 = tick_start
                mover_args = [
                    sys.executable, MOVER_SCRIPT,
                    f"--start={args.target_start}",
                    # =-attached: a negative leading vx (oblique_close, e.g.
                    # "-4.243,4.243") is otherwise mis-read by the mover's
                    # argparse as a flag. Byte-identical for positive/zero vx.
                    f"--vel={args.target_vel}",
                    "--duration", str(args.mover_duration),
                ]
                print(f"[m4] mover: {' '.join(mover_args)}")
                mover_proc = subprocess.Popen(mover_args, stdout=None, stderr=None)
            elif acquire_elapsed > ACQUIRE_TIMEOUT_S:
                aborted = True
                abort_reason = f"failed to acquire tag within {ACQUIRE_TIMEOUT_S}s"

        elif phase == "CUE_WAIT":
            # CUE_WAIT (S2, replaces ACQUIRE under --handoff): hover, point
            # the nose at wherever the shared tracker THINKS the target is
            # from the cue alone (own-state azimuth to the tracker's
            # position -- NOT a camera bearing; at the S2 start range the
            # tag is normally not yet in the camera's own detection
            # envelope at all, see ADR-0010 seat A's coast-time warning).
            # Leaves once the cue stream is flowing AND the shared tracker
            # has enough corrections to carry a usable position+velocity
            # estimate into DASH.
            v_down = (
                _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
                if alt_m is not None else 0.0
            )
            yaw_deg = psi_deg if psi_deg is not None else 0.0
            tpos = target_tracker.pos_hat
            if tpos is not None and state.pos_n is not None and state.pos_e is not None:
                rel_n = tpos[0] - state.pos_n
                rel_e = tpos[1] - state.pos_e
                if abs(rel_n) > 1e-6 or abs(rel_e) > 1e-6:
                    yaw_deg = math.degrees(math.atan2(rel_e, rel_n))
            cmd = (0.0, 0.0, v_down, yaw_deg)

            n_cue_received = cue_reader.n_received if cue_reader is not None else 0
            cue_wait_elapsed = tick_start - cue_wait_start_mono
            if (
                n_cue_received >= s2params["CUE_WAIT_MIN_DATAGRAMS"]
                and tracker_correction_count >= s2params["CUE_WAIT_MIN_CORRECTIONS"]
            ):
                print(
                    f"[s2] Cue lock at t={cue_wait_elapsed:.2f}s "
                    f"({n_cue_received} datagrams, {tracker_correction_count} tracker "
                    "corrections). Spawning mover and dashing..."
                )
                phase = "DASH"
                dash_start_mono = tick_start
                mover_args = [
                    sys.executable, MOVER_SCRIPT,
                    f"--start={args.target_start}",
                    # =-attached: a negative leading vx (oblique_close, e.g.
                    # "-4.243,4.243") is otherwise mis-read by the mover's
                    # argparse as a flag. Byte-identical for positive/zero vx.
                    f"--vel={args.target_vel}",
                    "--duration", str(args.mover_duration),
                ]
                print(f"[s2] mover: {' '.join(mover_args)}")
                mover_proc = subprocess.Popen(mover_args, stdout=None, stderr=None)
            elif cue_wait_elapsed > s2params["CUE_WAIT_TIMEOUT_S"]:
                aborted = True
                abort_reason = (
                    f"CUE_WAIT: cue stream/tracker not ready within "
                    f"{s2params['CUE_WAIT_TIMEOUT_S']}s ({n_cue_received} datagrams "
                    f"received, {tracker_correction_count} tracker corrections)"
                )

        elif phase == "DASH":
            # DASH (S2): lead-pursuit (the PIP intercept-triangle solve,
            # ADR-0011) toward the shared tracker's position+velocity
            # estimate at the fixed S2 DASH_SPEED -- the "running start"
            # that makes the FPV target band catchable at all (ADR-0011
            # addendum's S1<->S2 coupling finding). Yaw points at the
            # TRACKER's azimuth, not a camera bearing -- there may be no
            # detection yet at DASH's longer ranges.
            dash_elapsed = tick_start - dash_start_mono

            # ADR-0015 coast-search: watch the cue staleness clock during DASH.
            # last_cue_recv_sim_t stops advancing once the cue link dies (mock
            # stops emitting -> CueReader stops seeing new seqs); when the gap
            # exceeds COAST_STALE_S we latch into dead-reckon coast.
            if args.coast_search and sim_clock is not None and sim_clock.t is not None \
                    and last_cue_recv_sim_t is not None:
                cue_age_dash = sim_clock.t - last_cue_recv_sim_t
                cue_stale_flag = 1 if cue_age_dash > COAST_STALE_S else 0
                if cue_age_dash > COAST_STALE_S and not coast_active:
                    coast_active = True
                    coast_loss_sim_t = sim_clock.t
                    print(
                        f"[s2] COAST-SEARCH: cue stale (age {cue_age_dash:.2f}s > "
                        f"{COAST_STALE_S}s) at t_sim={sim_clock.t:.2f} -- dead-reckoning "
                        "the track and continuing the dash (ADR-0015)"
                    )

            # P-6 (--fuse-midcourse): once the camera is in the fused window,
            # aim the dash off the bearing-weighted fused polar state (removes
            # the cue's cross-LOS noise/datum error from the final dash
            # stretch) instead of the raw cue Cartesian track. Falls back to
            # the cue tracker before first detection / when the flag is off.
            own = (state.pos_n, state.pos_e) if (state.pos_n is not None and state.pos_e is not None) else None
            tpos = None
            vt = (0.0, 0.0)
            if args.fuse_midcourse and fused is not None and own is not None and fused.camera_fresh(tick_start):
                own_vel_ned = (state.vel_n, state.vel_e) if getattr(state, "vel_n", None) is not None else None
                st = fused.state(tick_start, own, own_vel_ned, target_tracker.vel_hat)
                if st is not None:
                    tpos = st["pos"]
                    vt = st["vel"]
            elif (
                args.fuse_midcourse and args.tracker == "ekf" and ekf_tracker is not None
                and own is not None and ekf_tracker.initialized
                and ekf_tracker.last_camera_t is not None
                and (tick_start - ekf_tracker.last_camera_t) <= MEAS_STALE_S
            ):
                # Fusion capstone (design D2, ADR-0041): the aim source
                # FusedTrack.state() serves above comes instead from the
                # EKF's own covariance-gated relative state (dn,de -> tpos;
                # dvn,dve -> vt, converted to ABSOLUTE NED velocity via own
                # EKF2 velocity since dvn/dve are target-MINUS-own -- see
                # ekf_tracker.py's module docstring) -- FusedTrack is not
                # constructed in this combination (no double-fusion). Same
                # camera-fresh gating semantics as fused.camera_fresh().
                own_vel_ned = (state.vel_n, state.vel_e) if getattr(state, "vel_n", None) is not None else None
                if own_vel_ned is not None:
                    tpos = (own[0] + ekf_tracker.x[0], own[1] + ekf_tracker.x[1])
                    vt = (ekf_tracker.x[2] + own_vel_ned[0], ekf_tracker.x[3] + own_vel_ned[1])
            if tpos is None:
                tpos = target_tracker.pos_hat
                vt = target_tracker.vel_hat or (0.0, 0.0)
            if tpos is not None and own is not None:
                rel = (tpos[0] - own[0], tpos[1] - own[1])
                t_go = solve_intercept_time(
                    rel, vt, s2params["DASH_SPEED"], s2params["DASH_MAX_LEAD_S"]
                )
                aim = (tpos[0] + vt[0] * t_go, tpos[1] + vt[1] * t_go)
                dirn = (aim[0] - own[0], aim[1] - own[1])
                dn = math.hypot(dirn[0], dirn[1])
                if dn > 1e-6:
                    vh0 = s2params["DASH_SPEED"] * dirn[0] / dn
                    vh1 = s2params["DASH_SPEED"] * dirn[1] / dn
                else:
                    vh0, vh1 = last_cmd[0], last_cmd[1]
                norm = math.hypot(vh0, vh1)
                if norm > V_TOTAL_MAX:  # FPV V_TOTAL_MAX (apply_fpv_profile already ran)
                    scale = V_TOTAL_MAX / norm
                    vh0 *= scale
                    vh1 *= scale
                yaw_deg = math.degrees(math.atan2(rel[1], rel[0]))

                # ADR-0015 coast-search command override on a dead link. Far
                # out: keep dead-reckoning the dash toward the PREDICTED point
                # (the tracker keeps predicting every tick). Inside the
                # predicted acquisition basket: HOLD and run a bounded yaw
                # sweep around the predicted LOS to reacquire with the seeker.
                if coast_active:
                    pred_range = math.hypot(rel[0], rel[1])
                    if pred_range <= COAST_ACQ_RANGE_M:
                        coast_phase_tick = "search"
                        vh0 = vh1 = 0.0
                        t_since_loss = (
                            sim_clock.t - coast_loss_sim_t
                            if (sim_clock is not None and sim_clock.t is not None
                                and coast_loss_sim_t is not None) else 0.0
                        )
                        yaw_deg = math.degrees(math.atan2(rel[1], rel[0])) + \
                            COAST_SEARCH_YAW_AMP_DEG * math.sin(
                                2.0 * math.pi * t_since_loss / COAST_SEARCH_PERIOD_S
                            )
                    else:
                        coast_phase_tick = "coast"
            else:
                vh0, vh1 = 0.0, 0.0
                yaw_deg = psi_deg if psi_deg is not None else 0.0

            v_down = (
                _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
                if alt_m is not None else 0.0
            )
            cmd = (vh0, vh1, v_down, yaw_deg)
            last_cmd = cmd

            if fresh and first_dash_detection_range is None:
                first_dash_detection_range = meas.range_m
                print(
                    f"[s2] First camera detection during DASH at "
                    f"range={meas.range_m:.2f} m (t={dash_elapsed:.2f}s into DASH)"
                )

            # HANDOFF trigger (ADR-0010 #5): >=N consecutive FRESH camera
            # detections, all measuring range <= HANDOFF_RANGE_M. Mirrors
            # ACQUIRE's consecutive_fresh/new_meas pattern (a new detector
            # result with no tag, or one that's out of range, resets the
            # streak; a tick with no new detector result at all leaves it
            # untouched -- normal ~14 Hz-vs-20 Hz cadence, not a miss).
            if new_meas:
                if meas.range_m is not None and meas.range_m <= s2params["HANDOFF_RANGE_M"]:
                    handoff_streak += 1
                else:
                    handoff_streak = 0

            if handoff_streak >= s2params["HANDOFF_STREAK_MIN"]:
                handoff_done = True
                outcome = "handoff_engaged"
                handoff_t = time.monotonic() - started
                handoff_range_at_trigger = meas.range_m
                print(
                    f"[s2] HANDOFF at t={handoff_t:.2f}s, camera range="
                    f"{meas.range_m:.2f} m (streak={handoff_streak}), r_hat="
                    f"{'n/a' if r_hat is None else f'{r_hat:.2f}'} m -- closing "
                    "external cue channel (ADR-0010 #5, latched one-way; the "
                    "cue mock process keeps running/logging as audit evidence)"
                )
                cue_reader.close()
                cue_reader = None  # illegal-state-unrepresentable past this point
                # P-6 (--warm-handoff): WARM lock-transfer (ADR-0015) at the
                # latch instant -- initialize the camera-only terminal filters
                # (lambda, lambda_dot, R, Rdot) + v_perp + the shared PIP track
                # from the best track available (the fused polar state, or the
                # dash tracker without --fuse-midcourse), so the terminal does
                # NOT fly its first seconds on still-converging cold rate
                # states (the ADR-0009 starved-Vc window). Angle stays
                # camera-owned; rates are geometric from the (cue-emitted when
                # fresh) target velocity. Pre-latch cue data only -- legal.
                if args.warm_handoff and state.pos_n is not None and state.pos_e is not None:
                    own_w = (state.pos_n, state.pos_e)
                    own_vel_w = (state.vel_n, state.vel_e) if getattr(state, "vel_n", None) is not None else None
                    st = fused.state(tick_start, own_w, own_vel_w, target_tracker.vel_hat) if fused is not None else None
                    if st is None and target_tracker.pos_hat is not None:
                        # No fused track (warm-handoff without fuse-midcourse):
                        # derive the geometric state from the dash tracker.
                        tp = target_tracker.pos_hat
                        rel_w = (tp[0] - own_w[0], tp[1] - own_w[1])
                        Rw = max(math.hypot(rel_w[0], rel_w[1]), 1e-3)
                        lamw = math.atan2(rel_w[1], rel_w[0])
                        uw = (rel_w[0] / Rw, rel_w[1] / Rw)
                        vtw = target_tracker.vel_hat or (0.0, 0.0)
                        vow = own_vel_w or (0.0, 0.0)
                        vrelw = (vtw[0] - vow[0], vtw[1] - vow[1])
                        st = dict(
                            pos=tp, vel=vtw, lam=lamw, R=Rw,
                            rdot=vrelw[0] * uw[0] + vrelw[1] * uw[1],
                            lamdot=(uw[0] * vrelw[1] - uw[1] * vrelw[0]) / Rw,
                        )
                    if st is not None:
                        # Capture init state BEFORE mutating x_hat (initialized
                        # keys off x_hat). In the normal handoff (>=3 detections)
                        # both are already initialized; these guards are
                        # defensive for a pathological cold handoff.
                        lam_was_init = lambda_filter.initialized
                        # Angle stays CAMERA-owned when already tracking; only
                        # seed it if the terminal angle filter is somehow cold.
                        if not lam_was_init:
                            lambda_filter.x_hat = st["lam"]
                        lambda_filter.xdot_hat = st["lamdot"]
                        if lambda_filter._last_correction_t is None:
                            lambda_filter._last_correction_t = tick_start
                        range_filter.x_hat = st["R"]
                        range_filter.xdot_hat = st["rdot"]
                        if range_filter._last_correction_t is None:
                            range_filter._last_correction_t = tick_start
                        # v_perp = the vehicle's ACTUAL cross-LOS velocity, so
                        # the terminal's first command continues the dash
                        # course rather than jumping from a stale integrator.
                        uw = (math.cos(st["lam"]), math.sin(st["lam"]))
                        pvec = (-uw[1], uw[0])
                        if getattr(state, "vel_n", None) is not None:
                            v_perp = _clamp(
                                state.vel_n * pvec[0] + state.vel_e * pvec[1],
                                -V_PERP_MAX, V_PERP_MAX,
                            )
                        # Re-anchor the shared PIP track to the fused state.
                        target_tracker.fn.x_hat = st["pos"][0]
                        target_tracker.fe.x_hat = st["pos"][1]
                        target_tracker.fn.xdot_hat = st["vel"][0]
                        target_tracker.fe.xdot_hat = st["vel"][1]
                        print(
                            f"[s2] WARM HANDOFF (P-6): terminal seeded "
                            f"lambda={math.degrees(st['lam']):.1f}deg "
                            f"lambda_dot={math.degrees(st['lamdot']):.1f}deg/s "
                            f"R={st['R']:.2f}m Rdot={st['rdot']:.2f}m/s"
                        )
                # Fusion capstone handoff boundary (design D3, ADR-0041).
                # H1 (adversarial review, fixed): latch() runs AFTER the
                # warm-handoff seeding block above, not before -- seeding is
                # a legitimate state warm-transfer, but it also resets P
                # (seed_from_polar), and if latch() ran first that reset
                # would silently UNDO the D3.1 floor (p_diag_at_latch would
                # then record a floor that no longer exists the very next
                # instant). Moving the call here makes latch() the true LAST
                # word on P for this tick -- belt (cue_reader=None above
                # already makes a further correct_cue call unreachable) and
                # suspenders (latch() itself makes correct_cue raise + count,
                # and re-inflates P's position block via max(), never
                # shrinking -- M1 -- if the cue was ever used pre-latch, the
                # F1 confident-bias-lock escape hatch). seed_from_polar()
                # also independently re-floors if it's ever the one called
                # AFTER latch() instead (defense in depth for any other call
                # ordering). Called for every EKF run regardless of
                # --fuse-midcourse -- the D3.2 adaptive gate-recovery is a
                # general post-handoff robustness feature, not
                # fusion-specific.
                if ekf_tracker is not None:
                    ekf_tracker.latch()
                phase = "ENGAGE"
                engage_t0 = tick_start
            elif dash_elapsed > s2params["DASH_TIMEOUT_S"]:
                aborted = True
                abort_reason = (
                    f"DASH: failed to reach handoff range within "
                    f"{s2params['DASH_TIMEOUT_S']}s (best streak {handoff_streak}, "
                    f"first camera detection range="
                    f"{'n/a' if first_dash_detection_range is None else f'{first_dash_detection_range:.2f}'})"
                )

            # ADR-0015 coast-search budget: no camera acquisition within the
            # sim-time budget after cue link-loss -> break off rather than fly
            # blind (docs/decisions.md ADR-0015 "Handoff continuity"). Guarded
            # on phase=="DASH" so a HANDOFF that fired above takes priority.
            if (
                phase == "DASH" and not aborted and coast_active
                and coast_loss_sim_t is not None and sim_clock is not None
                and sim_clock.t is not None
                and (sim_clock.t - coast_loss_sim_t) > COAST_SEARCH_BUDGET_S
            ):
                outcome = "link_lost_no_acq"
                breakoff_reason = (
                    f"coast-search: no camera acquisition within "
                    f"{COAST_SEARCH_BUDGET_S}s of cue link-loss"
                )
                print(f"[s2] COAST-SEARCH BREAKOFF: {breakoff_reason}")
                phase = "BREAKOFF"
                breakoff_entry_mono = tick_start

        elif phase == "ENGAGE":
            engage_elapsed = tick_start - engage_t0

            if detected:
                if args.split_freeze:
                    # Tier-1 lever B: no whole-vector coast, ever. Latch the
                    # v_perp magnitude once (one-way, same permanent-freeze
                    # philosophy as the original -- no un-freeze), keep
                    # everything else live: v_close is compute_v_close(r_hat)
                    # (monotone, never singular), lambda_hat/yaw correct off
                    # every fresh detection, and lambda_dot is rate-capped in
                    # the filter itself (see lambda_rate_cap above).
                    in_terminal_coast = False
                    if (
                        not split_frozen
                        and r_hat is not None
                        and r_hat < TERMINAL_FREEZE_RANGE_M
                    ):
                        split_frozen = True
                        print(
                            f"[m4] Split-freeze latch: v_perp frozen at "
                            f"{v_perp:+.2f} m/s (r_hat={r_hat:.2f} m); "
                            "v_close/yaw/lambda_hat stay live through CPA"
                        )
                else:
                    in_terminal_coast = (
                        frozen_vworld is not None
                        or (r_hat is not None and r_hat < TERMINAL_FREEZE_RANGE_M)
                    )
                if not in_terminal_coast:
                    v_close = compute_v_close(r_hat, args.fpv)  # two-speed under --fpv

                    if args.law == "pip":
                        # Predicted Intercept Point (ADR-0011): aim the whole
                        # closing velocity at the LEAD point where we and the
                        # target's estimated constant-velocity track would
                        # meet -- the direct cure for arriving where the target
                        # WAS. Falls back to aiming at the current estimate
                        # (pursuit) when the tracker isn't ready or the target
                        # outruns v_close (solve_intercept_time -> 0).
                        tpos = target_tracker.pos_hat
                        if tpos is not None and state.pos_n is not None:
                            own = (state.pos_n, state.pos_e)
                            rel = (tpos[0] - own[0], tpos[1] - own[1])
                            vt = target_tracker.vel_hat or (0.0, 0.0)
                            t_go = solve_intercept_time(rel, vt, v_close, PIP_MAX_LEAD_S)
                            aim = (tpos[0] + vt[0] * t_go, tpos[1] + vt[1] * t_go)
                            dirn = (aim[0] - own[0], aim[1] - own[1])
                            dn = math.hypot(dirn[0], dirn[1])
                            if dn > 1e-6:
                                vh0 = v_close * dirn[0] / dn
                                vh1 = v_close * dirn[1] / dn
                            else:
                                vh0, vh1 = last_cmd[0], last_cmd[1]
                        else:
                            # Tracker not ready -> aim up the LOS at v_close.
                            vh0 = v_close * math.cos(lambda_hat)
                            vh1 = v_close * math.sin(lambda_hat)
                        v_perp = 0.0  # not used by PIP; kept for the CSV column
                    else:
                        u = (math.cos(lambda_hat), math.sin(lambda_hat))
                        p = (-math.sin(lambda_hat), math.cos(lambda_hat))
                        if args.law == "pronav":
                            if not split_frozen:
                                v_perp = _clamp(v_perp + a_cmd * dt, -V_PERP_MAX, V_PERP_MAX)
                            # else (--split-freeze, latched): v_perp holds its
                            # frozen SCALAR value -- re-projected onto the
                            # LIVE lambda_hat basis below each tick.
                        else:
                            v_perp = 0.0  # pursuit: no lateral term, ever.
                        vh0 = v_close * u[0] + v_perp * p[0]
                        vh1 = v_close * u[1] + v_perp * p[1]

                    norm = math.hypot(vh0, vh1)
                    if norm > V_TOTAL_MAX:
                        scale = V_TOTAL_MAX / norm
                        vh0 *= scale
                        vh1 *= scale
                else:
                    # TERMINAL COAST (see TERMINAL_FREEZE_RANGE_M comment):
                    # lock the collision-course velocity vector on entry and
                    # fly it straight through CPA -- lambda_dot is singular
                    # here and chasing it un-flies the intercept.
                    if frozen_vworld is None:
                        frozen_vworld = (vh0, vh1)
                        print(
                            f"[m4] Terminal coast: velocity vector frozen at "
                            f"r_hat={r_hat:.2f} m (({vh0:.2f}, {vh1:.2f}) world)"
                        )
                    vh0, vh1 = frozen_vworld

                v_down = (
                    _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
                    if alt_m is not None else 0.0
                )
                # Absolute yaw setpoint at the tag's azimuth (psi + beta) --
                # PX4's attitude controller does the slewing; no rate loop or
                # LOS-rate feedforward needed with yaw-angle setpoints.
                yaw_deg = psi_deg + math.degrees(meas.bearing_rad) if psi_deg is not None else 0.0
                cmd = (vh0, vh1, v_down, yaw_deg)
                last_cmd = cmd
                dropout_start_mono = None
                lost_since_mono = None

                if fresh:
                    if meas.range_m < BREAKOFF_HARD_FLOOR_M:
                        breakoff_reason = (
                            f"measured range {meas.range_m:.2f} m < hard floor "
                            f"{BREAKOFF_HARD_FLOOR_M} m"
                        )
                    if last_fresh_range is not None and meas.range_m > last_fresh_range:
                        range_increase_streak += 1
                    else:
                        range_increase_streak = 0
                    last_fresh_range = meas.range_m
                    if (
                        not breakoff_reason
                        and breakoff_armed
                        and range_increase_streak >= BREAKOFF_RANGE_INCREASES
                    ):
                        breakoff_reason = (
                            f"measured range increased for {range_increase_streak} "
                            "consecutive fresh detections (past closest approach)"
                        )
            else:
                # Dropout: terminal-range holds the last command through a
                # brief loss (endgame FOV loss is EXPECTED, see module
                # docstring); far from the target, a dropout means we
                # genuinely lost the target and should just hover/abort.
                if r_hat is not None and r_hat < TERMINAL_RANGE_M:
                    if dropout_start_mono is None:
                        dropout_start_mono = tick_start
                    cmd = last_cmd
                    if tick_start - dropout_start_mono > TERMINAL_HOLD_MAX_S:
                        breakoff_reason = (
                            f"lost detection for >{TERMINAL_HOLD_MAX_S}s inside "
                            f"terminal range ({TERMINAL_RANGE_M} m)"
                        )
                else:
                    cmd = (0.0, 0.0, 0.0, psi_deg if psi_deg is not None else 0.0)
                    last_cmd = cmd
                    if lost_since_mono is None:
                        lost_since_mono = tick_start
                    if tick_start - lost_since_mono > LOST_TAG_ABORT_S:
                        aborted = True
                        abort_reason = (
                            f"lost tag for more than {LOST_TAG_ABORT_S}s (far from target)"
                        )

            if not breakoff_reason and engage_elapsed > ENGAGE_TIMEOUT_S:
                breakoff_reason = f"engage phase exceeded {ENGAGE_TIMEOUT_S}s timeout"

            if breakoff_reason:
                print(f"[m4] BREAKOFF: {breakoff_reason}")
                phase = "BREAKOFF"
                breakoff_entry_mono = tick_start

        else:  # phase == "BREAKOFF"
            breakoff_elapsed = tick_start - breakoff_entry_mono
            hold_yaw = last_cmd[3]
            if breakoff_elapsed < CLIMB_S:
                cmd = (0.0, 0.0, -CLIMB_V, hold_yaw)
            else:
                cmd = (0.0, 0.0, 0.0, hold_yaw)

        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(*cmd))
        except OffboardError as error:
            aborted = True
            abort_reason = f"set_velocity_ned failed: {error}"

        gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
        if gt_range is not None and (min_gt_range_running is None or gt_range < min_gt_range_running):
            min_gt_range_running = gt_range

        n_ticks += 1
        if detected:
            n_detected_ticks += 1

        tgt_state = None
        if target_tracker.initialized:
            tgt_pos_hat = target_tracker.pos_hat
            tgt_vel_hat = target_tracker.vel_hat or (0.0, 0.0)
            tgt_state = (tgt_pos_hat[0], tgt_pos_hat[1], tgt_vel_hat[0], tgt_vel_hat[1])

        # Fusion capstone (design D2/F4, ADR-0041): the EKF's own relative
        # state + P diagonal, logged only under --tracker ekf once the EKF
        # is initialized (empty otherwise -- see write_row_m4/CSV_HEADER).
        ekf_row_state = None
        if args.tracker == "ekf" and ekf_tracker is not None and ekf_tracker.initialized:
            ekf_row_state = (
                float(ekf_tracker.x[0]), float(ekf_tracker.x[1]),
                float(ekf_tracker.x[2]), float(ekf_tracker.x[3]),
                float(ekf_tracker.P[0, 0]), float(ekf_tracker.P[1, 1]),
                float(ekf_tracker.P[2, 2]), float(ekf_tracker.P[3, 3]),
            )

        write_row_m4(
            writer, log_file, time.monotonic() - started, phase, args.law,
            detected, meas, psi_deg, lambda_hat, lambda_dot_hat, r_hat, rdot_hat,
            vc, a_cmd, v_perp, cmd, alt_m, gt_cam, gt_tag, gt_range,
            ext_xyz=(ext_n, ext_e, ext_z) if ext_fresh else None,
            ext_fresh=ext_fresh, tgt_state=tgt_state, ext_age_s=ext_age_s,
            cue_stale=cue_stale_flag, coast_phase=coast_phase_tick,
            t_sim=sim_clock.t if sim_clock is not None else None,
            ekf_state=ekf_row_state,
        )

        if aborted:
            print(f"[m4] ABORT: {abort_reason}")
            break
        if phase == "BREAKOFF" and (tick_start - breakoff_entry_mono) >= POST_BREAKOFF_LOG_S:
            break

        elapsed = time.monotonic() - tick_start
        await asyncio.sleep(max(0.0, dt - elapsed))

    if cue_reader is not None:
        # Only reached if the run ended WITHOUT ever reaching HANDOFF (e.g.
        # aborted during CUE_WAIT/DASH) -- the normal HANDOFF path already
        # closed and nulled this above. Defensive cleanup, not the audit
        # latch itself.
        cue_reader.close()

    # ADR-0015 outcome tag: HANDOFF/coast-breakoff set it explicitly above;
    # otherwise classify a plain abort vs. an un-handed-off end.
    if outcome == "no_handoff":
        if handoff_done:
            outcome = "handoff_engaged"
        elif aborted:
            outcome = "aborted"

    return {
        "aborted": aborted,
        "abort_reason": abort_reason,
        "breakoff_reason": breakoff_reason,
        "n_ticks": n_ticks,
        "n_detected_ticks": n_detected_ticks,
        "min_gt_range_running": min_gt_range_running,
        "mover_proc": mover_proc,
        "cue_proc": cue_proc,
        # S2 (--handoff): default handoff_done=True when --handoff was never
        # requested, so the non-handoff "clean" calculation in main() (which
        # ANDs this in) is completely unaffected -- see main()'s comment.
        "handoff_done": handoff_done if args.handoff else True,
        "handoff_t": handoff_t,
        "handoff_range_at_trigger": handoff_range_at_trigger,
        "first_dash_detection_range": first_dash_detection_range,
        "outcome": outcome,
        # Fusion capstone (design D3, ADR-0041): None under tracker!=ekf
        # (main()'s S2_RESULT emission only tags these under tracker=ekf).
        "ekf_cue_updates_post_handoff": (
            ekf_tracker.cue_updates_post_handoff if ekf_tracker is not None else None
        ),
        "ekf_camera_gated_post_handoff": (
            ekf_tracker.camera_gated_post_handoff if ekf_tracker is not None else None
        ),
    }


async def run_bench(drone, state, meas_holder, tracker, writer, log_file, started, args, sim_clock=None):
    """--bench: prove the lambda = psi + beta compensation actually works.
    Spins the vehicle in place (yawspeed square wave) over a STATIC tag
    (no mover) while running the lambda filter normally; if the
    compensation is correct, lambda_dot_hat should stay near zero (the tag
    genuinely isn't moving) even though the raw bearing is sweeping hard as
    the vehicle spins under it. See the module docstring's "strapdown
    seeker" section for why this is exactly the failure mode a naive
    d(bearing)/dt would fall into.
    """
    dt = 1.0 / CONTROL_RATE_HZ
    lambda_filter = AlphaBetaFilter(ALPHA, BETA_GAIN_LAMBDA, angular=True)

    wave_total_s = BENCH_SEG_S * 2 * BENCH_CYCLES
    lambda_dot_samples = []
    last_meas_t_mono = None
    phase = "BENCH_WAVE"
    bench_start = time.monotonic()
    recenter_start = None

    while True:
        tick_start = time.monotonic()
        detected, meas = sample_measurement(meas_holder)
        fresh = detected and (last_meas_t_mono is None or meas.t_mono != last_meas_t_mono)
        if fresh:
            last_meas_t_mono = meas.t_mono

        psi_deg = state.yaw_deg
        psi_rad = math.radians(psi_deg) if psi_deg is not None else 0.0
        alt_m = state.relative_altitude_m

        lambda_filter.predict(dt)
        if fresh:
            lambda_filter.correct(psi_rad + meas.bearing_rad, tick_start)

        v_down = (
            _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
            if alt_m is not None else 0.0
        )

        elapsed = tick_start - bench_start
        if phase == "BENCH_WAVE":
            if elapsed >= wave_total_s:
                phase = "BENCH_RECENTER"
                recenter_start = tick_start
                yawspeed_cmd = 0.0
                cmd = (0.0, 0.0, v_down, yawspeed_cmd)
            else:
                cycle_pos = elapsed % (2 * BENCH_SEG_S)
                yawspeed_cmd = BENCH_WAVE_DEG_S if cycle_pos < BENCH_SEG_S else -BENCH_WAVE_DEG_S
                cmd = (0.0, 0.0, v_down, yawspeed_cmd)
                if fresh and lambda_filter.initialized:
                    lambda_dot_samples.append(abs(math.degrees(lambda_filter.xdot_hat)))
        else:  # BENCH_RECENTER
            recenter_elapsed = tick_start - recenter_start
            if recenter_elapsed >= BENCH_RECENTER_S:
                gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
                write_row_m4(
                    writer, log_file, time.monotonic() - started, phase, args.law,
                    detected, meas, psi_deg, lambda_filter.x_hat, lambda_filter.xdot_hat,
                    None, None, None, None, None, (0.0, 0.0, 0.0, 0.0), alt_m,
                    gt_cam, gt_tag, gt_range,
                    t_sim=sim_clock.t if sim_clock is not None else None,
                )
                break
            if detected:
                yawspeed_cmd = _clamp(
                    KYAW_DEG_PER_DEG * math.degrees(meas.bearing_rad),
                    -YAWSPEED_MAX_DEG_S, YAWSPEED_MAX_DEG_S,
                )
            else:
                yawspeed_cmd = 0.0
            cmd = (0.0, 0.0, v_down, yawspeed_cmd)

        try:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(*cmd))
        except OffboardError as error:
            print(f"[m4] BENCH FAILED: set_velocity_body error: {error}")
            break

        gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
        write_row_m4(
            writer, log_file, time.monotonic() - started, phase, args.law,
            detected, meas, psi_deg, lambda_filter.x_hat, lambda_filter.xdot_hat,
            None, None, None, None, None, cmd, alt_m, gt_cam, gt_tag, gt_range,
            t_sim=sim_clock.t if sim_clock is not None else None,
        )

        tick_elapsed = time.monotonic() - tick_start
        await asyncio.sleep(max(0.0, dt - tick_elapsed))

    mean_val = sum(lambda_dot_samples) / len(lambda_dot_samples) if lambda_dot_samples else float("nan")
    max_val = max(lambda_dot_samples) if lambda_dot_samples else float("nan")
    print(f"BENCH mean_abs_lambda_dot={mean_val:.3f} max={max_val:.3f}")
    passed = bool(lambda_dot_samples) and mean_val < BENCH_PASS_MEAN_DEG_S
    return passed, mean_val, max_val


async def main():
    started = time.monotonic()
    args = parse_args()

    # ADR-0023 Tier-1 lever B (--split-freeze): move the freeze later BEFORE
    # apply_fpv_profile() reads the dict, so the coast latch AND
    # compute_v_close's throttle-band floor move together -- exactly the
    # coupling the lab's B variant flew (guidance_lab.py PurePN shares
    # freeze_range between both uses). Flag OFF => dict untouched.
    if args.split_freeze:
        FPV["TERMINAL_FREEZE_RANGE_M"] = SPLIT_FREEZE_RANGE_M

    # ADR-0043 lever C (--terminal-freeze-range): freeze EARLIER for a noisy
    # markerless bearing channel -- same pre-apply dict-patch pattern as
    # --split-freeze above. Flag None => dict untouched, byte-identical.
    if args.terminal_freeze_range is not None:
        _tfr_before = FPV["TERMINAL_FREEZE_RANGE_M"]
        FPV["TERMINAL_FREEZE_RANGE_M"] = args.terminal_freeze_range
        print(
            "[m4] Terminal-freeze-range override (ADR-0043 lever C): "
            f"{_tfr_before} -> {args.terminal_freeze_range} m"
        )

    # ADR-0028 addendum follow-up (--dash-unclamp): patch V_TOTAL_MAX BEFORE
    # apply_fpv_profile() reads FPV into the module-level global (same
    # ordering requirement as --split-freeze above). Flag OFF => dict
    # untouched, byte-identical to every prior FPV/S2 run.
    if args.dash_unclamp:
        _v_total_before = FPV["V_TOTAL_MAX"]
        FPV["V_TOTAL_MAX"] = DASH_UNCLAMP_V_TOTAL_MAX
        print(
            "[m4] Dash-unclamp ON (ADR-0028 addendum follow-up): "
            f"V_TOTAL_MAX {_v_total_before} -> {DASH_UNCLAMP_V_TOTAL_MAX} m/s "
            "(lets a fast --dash-speed actually reach its commanded speed)"
        )

    # ADR-0028 Gazebo-confirm (--accel-boost): patch the PX4-param bundle
    # BEFORE it's read (both by the log line below and by the actual
    # MAVSDK param.set_param_float loop near arming). Flag OFF => dict
    # untouched, byte-identical to every prior FPV run.
    if args.accel_boost:
        FPV["PX4_PARAMS"]["MPC_ACC_HOR_MAX"] = ACCEL_BOOST_MPC_ACC_HOR_MAX
        FPV["PX4_PARAMS"]["MPC_TILTMAX_AIR"] = ACCEL_BOOST_MPC_TILTMAX_AIR
        print(
            "[m4] Accel-boost ON (ADR-0028 Gazebo-confirm): MPC_ACC_HOR_MAX "
            f"-> {ACCEL_BOOST_MPC_ACC_HOR_MAX} m/s^2, MPC_TILTMAX_AIR -> "
            f"{ACCEL_BOOST_MPC_TILTMAX_AIR} deg (x500 may be tilt/thrust-"
            "limited below this -- report ACHIEVED accel from telemetry)"
        )

    if args.fpv:
        apply_fpv_profile()
        print(
            "[m4] FPV profile ON (S1, ADR-0010): two-speed closing "
            f"(run-in {FPV['V_CLOSE_RUNIN']}, terminal {FPV['V_CLOSE_TERMINAL']} m/s), "
            f"terminal-freeze {TERMINAL_FREEZE_RANGE_M} m, V_PERP_MAX {V_PERP_MAX}, "
            f"BETA_GAIN_RANGE {BETA_GAIN_RANGE}"
        )
    if args.split_freeze:
        print(
            "[m4] Split-freeze ON (ADR-0023 Tier-1 lever B / ADR-0014 lever 2): "
            f"v_perp magnitude freezes at r_hat<{TERMINAL_FREEZE_RANGE_M} m; "
            "v_close/yaw/lambda_hat stay LIVE; |lambda_dot| capped at "
            f"{SPLIT_FREEZE_LAMBDA_DOT_CAP_DEG_S:.0f} deg/s in BOTH a_cmd and "
            "the filter predict (rate_cap)"
        )

    if args.kalata:
        print(
            "[m4] Kalata gains ON (tracking-refinement PORT 1): lambda "
            f"sigma_process={KALATA_LAMBDA_SIGMA_PROCESS_DEG_S2} deg/s^2 "
            f"sigma_meas={KALATA_LAMBDA_SIGMA_MEAS_DEG} deg; range "
            f"sigma_process={KALATA_RANGE_SIGMA_PROCESS_M_S2} m/s^2 "
            f"sigma_meas={KALATA_RANGE_SIGMA_MEAS_M} m (sampled gains "
            f"printed every {KALATA_LOG_EVERY} corrections as [kalata] lines)"
        )

    s2params = dict(S2)
    if args.handoff:
        if args.dash_speed is not None:
            s2params["DASH_SPEED"] = args.dash_speed
        if args.handoff_range is not None:
            s2params["HANDOFF_RANGE_M"] = args.handoff_range
        if args.cue_port is not None:
            s2params["CUE_PORT"] = args.cue_port
        # ADR-0023 Tier-1 lever A (--early-handoff): same one-way latch,
        # same streak logic, lower count -- the latch (and the cue-socket
        # close) fires ~1 m earlier at the S2 closing speeds.
        if args.early_handoff:
            s2params["HANDOFF_STREAK_MIN"] = EARLY_HANDOFF_STREAK_MIN
            print(
                "[s2] Early handoff ON (ADR-0023 Tier-1 lever A): HANDOFF "
                f"latch streak {S2['HANDOFF_STREAK_MIN']} -> "
                f"{s2params['HANDOFF_STREAK_MIN']} consecutive fresh in-range "
                "detections (one-way latch semantics unchanged; cue closes "
                "sooner)"
            )
        print(
            "[s2] Handoff profile ON (ADR-0010 #4/#5): CUE_WAIT -> DASH -> "
            f"HANDOFF -> ENGAGE. dash_speed={s2params['DASH_SPEED']} m/s, "
            f"handoff_range={s2params['HANDOFF_RANGE_M']} m, cue_port="
            f"{s2params['CUE_PORT']}, target_start={args.target_start}, "
            f"target_vel={args.target_vel}, mover_duration={args.mover_duration}s"
        )
        if args.cue_latency_comp:
            print(
                "[s2] Cue-latency compensation ON (tracking-refinement PORT "
                "2): cue positions advanced along the shared tracker's own "
                f"velocity estimate by their sim-time age (clamped "
                f"[{CUE_AGE_CLAMP_MIN_S}, {CUE_AGE_CLAMP_MAX_S}] s) before "
                "correcting."
            )

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    suffix = "bench" if args.bench else args.law
    log_path = os.path.join(LOGS_DIR, f"m4_intercept_{suffix}_{timestamp}.csv")

    # ADR-0033 (M5 finish; fixes the ADR-0032 pre-placement race). If the
    # caller gave an EXPLICIT --target-start, teleport the tag there NOW --
    # before Node(), the camera/pose subscriptions, and the detection thread
    # exist -- so the always-on detector cannot lock the world-file
    # default-position board (~5 m away) before the mover's own pre-warm
    # relocation fires deep into CUE_WAIT/DASH (the confirmed race signature:
    # r_hat frozen near 5 m while gt_range diverges to 25-30 m). This runs as a
    # SHORT-LIVED SUBPROCESS (gz service CLI, see preplace_target) precisely
    # because this process will hold a /clock subscription (SimClockHolder) and
    # must make ZERO in-process gz service calls. Non-fatal; --no-preplace opts
    # out, and mc_batch.sh's own external pre-place remains a valid idempotent
    # guard.
    if args.no_preplace:
        print(
            "[m4] Tag pre-placement DISABLED (--no-preplace): relying on the "
            "mover / external pre-place (ADR-0032 race can recur if neither "
            "pre-places the tag before detection starts)."
        )
    elif args.target_start_provided:
        preplace_target(args.target_start)

    node = Node()

    print(f"[m4] Reading camera intrinsics from {CAMERA_INFO_TOPIC} ...")
    try:
        fx, fy, cx, cy, width, height = get_camera_intrinsics(node, CAMERA_INFO_TIMEOUT_S)
    except (RuntimeError, queue.Empty) as exc:
        print(f"[m4] FAILED: could not get camera intrinsics: {exc}")
        return 1
    print(
        f"[m4] Intrinsics: fx={fx:.3f} fy={fy:.3f} cx={cx:.1f} cy={cy:.1f} "
        f"({width}x{height})"
    )

    tracker = PoseTracker()
    pose_sub = node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v)
    if not pose_sub:
        print(f"[m4] FAILED: could not subscribe to {POSE_TOPIC}")
        return 1

    frame_holder = LatestFrame()
    image_sub = node.subscribe(Image, IMAGE_TOPIC, frame_holder.on_image)
    if not image_sub:
        print(f"[m4] FAILED: could not subscribe to {IMAGE_TOPIC}")
        node.unsubscribe(POSE_TOPIC)
        return 1

    # Tracking-refinement PORT 2 (--cue-latency-comp): sim time, only
    # subscribed under --handoff (SimClockHolder's docstring explains why
    # this subscription is safe -- this process makes zero gz service
    # calls). Logged/used by the ext-cue consumption block regardless of
    # --cue-latency-comp (ext_age_s is informative on its own); only the
    # actual position advancement is gated by that flag.
    sim_clock = None
    clock_sub = False
    if args.handoff:
        sim_clock = SimClockHolder()
        clock_sub = node.subscribe(Clock, CLOCK_TOPIC, sim_clock.on_clock)
        if not clock_sub:
            print(f"[m4] FAILED: could not subscribe to {CLOCK_TOPIC}")
            node.unsubscribe(IMAGE_TOPIC)
            node.unsubscribe(POSE_TOPIC)
            return 1

    print(f"[m4] Subscribed to {IMAGE_TOPIC} and {POSE_TOPIC}")
    print(f"[m4] Logging to {log_path}")
    print("[m4] Waiting for the pose topic to populate...")
    deadline = time.monotonic() + POSE_WAIT_TIMEOUT_S
    while not tracker.latest and time.monotonic() < deadline:
        time.sleep(0.1)
    if not tracker.latest:
        print(f"[m4] FAILED: no pose data received within {POSE_WAIT_TIMEOUT_S}s")
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        if clock_sub:
            node.unsubscribe(CLOCK_TOPIC)
        return 1

    meas_holder = MeasurementHolder()
    stop_event = threading.Event()
    if args.seeker == "markerless":
        # ADR-0033 item 2: swap ONLY the detection SOURCE to the two-stage
        # markerless seeker. Imported LAZILY (and only in this branch) so the
        # apriltag default never touches onnxruntime/opencv-contrib and stays
        # byte-identical. Deps live in .venv-seeker; re-earns the no-cheat audit
        # at the live A/B (module docstring + markerless_loop.py).
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "seeker"))
        from markerless_loop import markerless_detection_loop
        print("[m4] Seeker: MARKERLESS two-stage (--seeker markerless).")
        detector_thread = threading.Thread(
            target=markerless_detection_loop,
            args=(frame_holder, meas_holder, fx, fy, cx, cy, stop_event),
            daemon=True,
        )
    else:
        detector_thread = threading.Thread(
            target=detection_loop,
            args=(frame_holder, meas_holder, fx, fy, cx, cy, stop_event),
            kwargs={"detector_kwargs": {"quad_decimate": 2.0}},
            daemon=True,
        )
    detector_thread.start()
    print("[m4] Detection thread started.")

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(CSV_HEADER)
    log_file.flush()

    drone = System()
    state = M4TelemetryState()
    tracker_tasks = []

    result_code = 1
    mover_proc = None
    cue_proc = None

    try:
        print(f"[m4] Connecting to {SYSTEM_ADDRESS} (timeout {CONNECT_TIMEOUT_S}s)...")
        await drone.connect(system_address=SYSTEM_ADDRESS)
        await wait_for_connection(drone, CONNECT_TIMEOUT_S)
        print("[m4] Connected.")

        tracker_tasks = [
            asyncio.create_task(track_position(drone, state)),
            asyncio.create_task(track_flight_mode(drone, state)),
            asyncio.create_task(track_armed(drone, state)),
            asyncio.create_task(track_landed_state(drone, state)),
            asyncio.create_task(track_attitude(drone, state)),
            asyncio.create_task(track_local_position(drone, state)),
        ]

        print(
            "[m4] Waiting for health (is_global_position_ok, is_home_position_ok), "
            f"timeout {HEALTH_TIMEOUT_S}s..."
        )
        await wait_for_health(drone, HEALTH_TIMEOUT_S)
        print("[m4] Health OK.")

        if args.fpv:
            # Push the FPV envelope params BEFORE arming (ADR-0010 decision #3):
            # runtime MAVSDK param API, never an airframe-file edit. Read back
            # each one so a stale value can't silently contaminate a run (and
            # so the run's own log records exactly what flew).
            print("[m4] FPV: setting PX4 params (read back for the log header)...")
            for name, value in FPV["PX4_PARAMS"].items():
                await drone.param.set_param_float(name, value)
                got = await drone.param.get_param_float(name)
                print(f"[m4]   {name} = {got} (requested {value})")
                if abs(got - value) > 1e-3:
                    raise RuntimeError(
                        f"PX4 param {name} did not take (got {got}, wanted {value})"
                    )

        print(f"[m4] Setting takeoff altitude to {ALT_REF_M} m...")
        await drone.action.set_takeoff_altitude(ALT_REF_M)

        print("[m4] Arming...")
        await drone.action.arm()

        print("[m4] Commanding takeoff...")
        await drone.action.takeoff()

        # NOTE: PX4 may floor very low takeoff altitudes / the relative
        # altitude may wobble a bit above target during takeoff itself --
        # that's fine, the altitude P-loop takes over once we're in
        # OFFBOARD (same note as M3).
        success_altitude = ALT_REF_M * ALTITUDE_SUCCESS_FRACTION
        print(
            f"[m4] Waiting for relative_altitude_m >= {success_altitude:.2f} m, "
            f"timeout {ALTITUDE_TIMEOUT_S}s..."
        )
        altitude_reached = False
        deadline = time.monotonic() + ALTITUDE_TIMEOUT_S
        while time.monotonic() < deadline:
            detected, meas = sample_measurement(meas_holder)
            gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
            write_row_m4(
                writer, log_file, time.monotonic() - started, "TAKEOFF", args.law,
                detected, meas, state.yaw_deg, None, None, None, None, None, None,
                0.0, None, state.relative_altitude_m, gt_cam, gt_tag, gt_range,
                t_sim=sim_clock.t if sim_clock is not None else None,
            )
            if (
                state.relative_altitude_m is not None
                and state.relative_altitude_m >= success_altitude
            ):
                altitude_reached = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)

        if not altitude_reached:
            raise RuntimeError(
                f"altitude target not reached within {ALTITUDE_TIMEOUT_S}s "
                f"(last relative_altitude_m={state.relative_altitude_m})"
            )
        print(f"[m4] Altitude reached: {state.relative_altitude_m:.2f} m")

        print("[m4] Entering OFFBOARD (streaming a zero setpoint first)...")
        offboard_ok = True
        try:
            if args.bench:
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            else:
                yaw0 = state.yaw_deg if state.yaw_deg is not None else 0.0
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw0))
            await drone.offboard.start()
        except OffboardError as error:
            print(f"[m4] FAILED: offboard start failed: {error}")
            offboard_ok = False

        if offboard_ok:
            print(f"[m4] OFFBOARD active. Mode={'BENCH' if args.bench else args.law}.")
            if args.bench:
                passed, mean_val, max_val = await run_bench(
                    drone, state, meas_holder, tracker, writer, log_file, started, args,
                    sim_clock=sim_clock,
                )
                result_code = 0 if passed else 1
                print(
                    f"[m4] BENCH {'PASS' if passed else 'FAIL'} "
                    f"(mean {mean_val:.3f} deg/s, bar < {BENCH_PASS_MEAN_DEG_S} deg/s)"
                )
            else:
                result = await run_acquire_and_engage(
                    drone, state, meas_holder, tracker, writer, log_file, started, args,
                    s2params=s2params, sim_clock=sim_clock,
                )
                mover_proc = result["mover_proc"]
                cue_proc = result["cue_proc"]

                min_gt_running = result["min_gt_range_running"]
                miss_from_csv = recompute_min_gt_range_from_csv(log_path)
                if min_gt_running is not None and not math.isnan(miss_from_csv):
                    if abs(miss_from_csv - min_gt_running) > 1e-2:
                        print(
                            f"[m4] WARNING: running-min miss ({min_gt_running:.3f} m) "
                            f"and CSV-recomputed miss ({miss_from_csv:.3f} m) disagree "
                            "by more than 0.01 m"
                        )
                miss_distance = (
                    miss_from_csv if not math.isnan(miss_from_csv)
                    else (min_gt_running if min_gt_running is not None else float("nan"))
                )

                coverage = (
                    result["n_detected_ticks"] / result["n_ticks"]
                    if result["n_ticks"] else 0.0
                )
                engaged_close = (
                    min_gt_running is not None and min_gt_running < BREAKOFF_ARM_RANGE_M
                )
                # handoff_done defaults to True when --handoff was never
                # requested (see run_acquire_and_engage's return dict), so
                # this AND has ZERO effect on the non-handoff "clean"
                # calculation -- byte-for-byte the same as before S2.
                clean = bool(
                    (not result["aborted"]) and engaged_close and result["handoff_done"]
                )

                print(
                    f"[m4] law={args.law} miss_distance_m={miss_distance:.3f} "
                    f"n_ticks={result['n_ticks']} coverage={coverage:.3f} "
                    f"breakoff_reason={result['breakoff_reason'] or 'n/a'} "
                    f"aborted={result['aborted']} log={log_path}"
                )
                engaged = mover_proc is not None
                print(
                    f"M4_RESULT law={args.law} miss={miss_distance:.3f} "
                    f"clean={int(clean)} engaged={int(engaged)}"
                )

                if args.handoff:
                    handoff_t = result["handoff_t"]
                    handoff_range = result["handoff_range_at_trigger"]
                    first_dash_range = result["first_dash_detection_range"]
                    print(
                        "[s2] handoff_done="
                        f"{int(result['handoff_done'])} handoff_t="
                        f"{'n/a' if handoff_t is None else f'{handoff_t:.2f}'}s "
                        f"handoff_range={'n/a' if handoff_range is None else f'{handoff_range:.2f}'}m "
                        "first_dash_detection_range="
                        f"{'n/a' if first_dash_range is None else f'{first_dash_range:.2f}'}m"
                    )
                    # cue_reads_post_handoff is hardcoded 0, not measured: the
                    # cue_reader reference is nulled the instant HANDOFF
                    # triggers (ADR-0010 #5), so any read attempt after that
                    # point is structurally impossible (AttributeError on
                    # None), not merely "didn't happen to occur" -- there is
                    # nothing left to count. Tagged [structural] (design D3,
                    # ADR-0041) to distinguish it from the two EKF counters
                    # below, which ARE measured at runtime (belt vs.
                    # suspenders -- see EKFTracker.latch()/correct_cue()).
                    # check_s2.sh parses this line by lookbehind grep on the
                    # `cue_reads_post_handoff=0` substring (verified against
                    # scripts/check_s2.sh 2026-07-08); the trailing
                    # `[structural]` tag is appended AFTER that exact
                    # substring so the existing parse is unaffected.
                    #
                    # ADR-0015: S2_RESULT gains a trailing outcome tag
                    # (handoff_engaged / link_lost_no_acq / aborted /
                    # no_handoff). Appended LAST so check_s2.sh's existing
                    # lookbehind greps for miss=/clean=/handoff= are unaffected.
                    # Fusion capstone (design D3, ADR-0041): under
                    # tracker=ekf ONLY, two MEASURED post-handoff counters
                    # are appended after outcome= (same "appended last"
                    # discipline) -- ekf_cue_updates_post_handoff must read 0
                    # on every honest flight (the structural belt above
                    # backed by a live count); ekf_camera_gated_post_handoff
                    # is the F1 canary (consecutive post-latch camera
                    # rejections, design D3.2).
                    ekf_tags = ""
                    if args.tracker == "ekf":
                        ekf_cue_uph = result.get("ekf_cue_updates_post_handoff")
                        ekf_gate_ph = result.get("ekf_camera_gated_post_handoff")
                        ekf_tags = (
                            f" ekf_cue_updates_post_handoff="
                            f"{0 if ekf_cue_uph is None else ekf_cue_uph}[measured]"
                            f" ekf_camera_gated_post_handoff="
                            f"{0 if ekf_gate_ph is None else ekf_gate_ph}[measured]"
                        )
                    print(
                        f"S2_RESULT law={args.law} miss={miss_distance:.3f} "
                        f"clean={int(clean)} handoff={int(result['handoff_done'])} "
                        f"handoff_range={'nan' if handoff_range is None else f'{handoff_range:.3f}'} "
                        f"handoff_t={'nan' if handoff_t is None else f'{handoff_t:.3f}'} "
                        f"cue_reads_post_handoff=0[structural] outcome={result['outcome']}{ekf_tags}"
                    )

                exit_ok = clean
                if exit_ok and args.require_miss is not None:
                    exit_ok = miss_distance < args.require_miss
                result_code = 0 if exit_ok else 1
        else:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
            if args.handoff:
                print(
                    f"S2_RESULT law={args.law} miss=nan clean=0 handoff=0 "
                    "handoff_range=nan handoff_t=nan cue_reads_post_handoff=0[structural] outcome=error"
                )
            result_code = 1

        if mover_proc is not None and mover_proc.poll() is None:
            print("[m4] Terminating mover subprocess...")
            mover_proc.terminate()
            try:
                mover_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                print("[m4] Mover did not exit in time, killing...")
                mover_proc.kill()
                mover_proc.wait(timeout=3.0)

        if cue_proc is not None and cue_proc.poll() is None:
            # Left running (not killed) through HANDOFF by design -- its log
            # is the audit evidence that cue data stayed available-but-
            # unread post-latch (ADR-0010 #5). Terminated here at run end,
            # same lifecycle as the mover.
            print("[s2] Terminating cue mock subprocess...")
            cue_proc.terminate()
            try:
                cue_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                print("[s2] Cue mock did not exit in time, killing...")
                cue_proc.kill()
                cue_proc.wait(timeout=3.0)

        print("[m4] Guidance phase complete. Stopping offboard and landing...")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"[m4] WARNING: offboard.stop() failed (may not have been active): {error}")

        await drone.action.land()

        print(f"[m4] Waiting for landed/disarmed, timeout {LAND_TIMEOUT_S}s...")
        landed = False
        deadline = time.monotonic() + LAND_TIMEOUT_S
        while time.monotonic() < deadline:
            if state.landed_state == LandedState.ON_GROUND or state.armed is False:
                landed = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)
        if not landed:
            print(
                f"[m4] WARNING: vehicle did not report landed/disarmed within "
                f"{LAND_TIMEOUT_S}s (last landed_state={state.landed_state}, "
                f"armed={state.armed})"
            )
        else:
            print("[m4] Landed / disarmed.")

        return result_code

    except asyncio.TimeoutError as exc:
        print(f"[m4] FAILED: timed out waiting for a stage: {exc}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
            if args.handoff:
                print(
                    f"S2_RESULT law={args.law} miss=nan clean=0 handoff=0 "
                    "handoff_range=nan handoff_t=nan cue_reads_post_handoff=0[structural] outcome=error"
                )
        return 1
    except ActionError as exc:
        print(f"[m4] FAILED: MAVSDK action error: {exc}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
            if args.handoff:
                print(
                    f"S2_RESULT law={args.law} miss=nan clean=0 handoff=0 "
                    "handoff_range=nan handoff_t=nan cue_reads_post_handoff=0[structural] outcome=error"
                )
        return 1
    except RuntimeError as exc:
        print(f"[m4] FAILED: {exc}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
            if args.handoff:
                print(
                    f"S2_RESULT law={args.law} miss=nan clean=0 handoff=0 "
                    "handoff_range=nan handoff_t=nan cue_reads_post_handoff=0[structural] outcome=error"
                )
        return 1
    except Exception as exc:  # noqa: BLE001 - top-level gate script, log and exit
        print(f"[m4] FAILED: unexpected error: {exc!r}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
            if args.handoff:
                print(
                    f"S2_RESULT law={args.law} miss=nan clean=0 handoff=0 "
                    "handoff_range=nan handoff_t=nan cue_reads_post_handoff=0[structural] outcome=error"
                )
        return 1
    finally:
        if mover_proc is not None and mover_proc.poll() is None:
            mover_proc.kill()
        if cue_proc is not None and cue_proc.poll() is None:
            cue_proc.kill()
        for task in tracker_tasks:
            task.cancel()
        for task in tracker_tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass
        stop_event.set()
        detector_thread.join(timeout=2.0)
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        if clock_sub:
            node.unsubscribe(CLOCK_TOPIC)
        log_file.close()
        print(f"[m4] Log written to {log_path}")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
