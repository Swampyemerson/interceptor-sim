#!/usr/bin/env python3
"""PRE-REGISTERED verdict script for the ADR-0059 comms-denied jam Monte-Carlo
(task #31). Written 2026-07-10, BEFORE any jam arm flew, per the overnight audit
(docs/audit_forward_2026-07-10.md section 1, "jam MC: GO to fly, NO-GO to
conclude without a pre-registered verdict script").

Scores the four paired arms produced by scripts/mc_jam_arm.sh (master-seed 42,
n=16, identical geometry; the ONLY differences are the cue cutoff and whether
the ADR-0059 staleness fix is live):

    control_strict : no cutoff, fix INERT  (--cue-stale-horizon 1e9)
    jam_fixoff     : cutoff,    fix INERT  -- the fail-closed WITNESS
    jam_fixon      : cutoff,    fix LIVE   -- the FIX under validation
    control_active : no cutoff, fix LIVE   -- realistic Markov dropout, no jam

Usage:
    check_jam_mc.py --strict CSV --fixoff CSV --fixon CSV --active CSV
                    [--r2 CSV] [--cutoff-range-m 15] [--expect-n 16]

Exit 0  = fix VALIDATED (fixon recovers, fixoff witnesses, controls equivalent).
Exit 1  = NOT validated, OR the batch is INCOMPLETE (infra) -- the printed
          verdict distinguishes the two; an infra-incomplete arm must be
          re-flown, it is NOT evidence the fix failed.

PRE-REGISTERED GATES (frozen before data exists; numbers from the audit):
  G1  COMPLETENESS   every arm: exactly N=16 rows; per flight the flight CSV
                     exists, has >=50 ticks and reaches DASH; the run log
                     exists; miss_m is finite. A crashed / missing-CSV flight
                     is an INFRA FAILURE, flagged explicitly -- it is NEVER
                     silently classified "never" (analyze_track_ab drops
                     NaN-miss from Pk denominators, which CONFLATES infra
                     failure with the fail-closed signature this MC measures).
  G2  JAM TRIPWIRE   every jam-arm flight's run log contains the RUNTIME
                     "[s2-cue] LINK CUTOFF at t_sim=" line (the startup banner
                     "link cutoff ON:" does NOT count) -- the cutoff FIRED.
  G3a FIXOFF WITNESS REAL(-ish) handoffs <= 6/16 AND never(fixoff) >=
                     never(strict) + 6  ("never >> control").
  G3b FIXON RECOVERY REAL(-ish) handoffs >= 11/16 (r2 baseline 14/16) AND
                     PHANTOM == 0.
  G3c HEADLINE JOINT per-flight joint success := REAL(-ish) handoff AND
                     post-handoff min gt_range <= 2.5 m, denominator = ALL 16
                     flights (NOT conditioned on handing off -- flights that
                     hand off under jam are the geometrically easy survivors,
                     so raw Pk(iii) cross-arm comparisons condition on
                     outcome-dependent subsets). Gate: joint(fixon) >= 8/16
                     AND joint(fixon) > joint(fixoff).
  G4  HARD STRATUM   stratify fixon by first_det_range_m vs the nominal cutoff
                     radius R: first-det >= R = camera saw the target BEFORE
                     the jam (easy, ~6/16 in r2, up to 23 m); first-det < R =
                     detected AFTER the cutoff (the genuinely denied case).
                     Gate: REAL(-ish) handoffs in the detected-AFTER-cutoff
                     stratum >= ceil(hard_n/2)  (if hard_n < 4 the stratum is
                     too thin: gate degrades to >=1 and the verdict is
                     annotated WEAK-STRATUM). The CUTOFF_RANGE_M=18 sweep arm
                     is the genuinely pre-detection follow-up.
  G5  STRICT ~ r2    inert-fix equivalence, PAIRED by cue_seed vs the r2
                     headline: median|dmiss| < 0.7 m AND two-sided sign test
                     p >= 0.05 AND Wilcoxon signed-rank p >= 0.05 AND per-seed
                     handoff-outcome agreement >= 14/16.  max|dmiss| is
                     DIAGNOSTIC ONLY -- P(max|d|<1 m over 16 pairs under the
                     null) ~ 3e-5, so gating on it falsely fails an inert fix.
                     MDE ~0.5 m is printed with the verdict language.
  G6  ACTIVE ~ STRICT arm-level ONLY (identical code, only the horizon
                     differs): REAL(-ish) handoffs >= strict - 2 AND
                     PHANTOM == 0. NEVER per-seed vs r2 (trajectories
                     legitimately diverge once the fallback engages). There is
                     NO "expect 16/16" bar (the harness's own 15/16-noise
                     caveat). Fallback ENGAGEMENT is counted via pre-handoff
                     cue_stale=1 ticks in the flight CSVs (expected ~70-80% of
                     flights under mean-1.5s Markov bursts vs the 1.0 s
                     horizon); engaged < 6/16 => the arm is WEAK EVIDENCE
                     (soft warning that scopes the claim; not a hard fail).
  R7  DiD            (jam_fixon - jam_fixoff) - (control_active -
                     control_strict), on joint successes and on REAL(-ish)
                     handoffs -- isolates the jam-specific effect. Report only.
  R8  PAIRED TABLE   per-seed fixoff-vs-fixon (which seeds recovered).
                     analyze_track_ab.paired() only auto-triggers on
                     'track'/'plain' labels, so it is built here.
  R9  ATTRIBUTION    per jam flight: LINK-CUTOFF t_sim, STALE-notice t_sim
                     (their gap ~ the 1.0 s staleness horizon = the transient
                     window), handoff_cue_rejects -- separates the 1.0 s
                     staleness-window transient from real fix failure. The
                     arms carry no --coast-search, so a fixon NON-handoff is
                     AMBIGUOUS between cue-gate failure and FOV/steering loss:
                     annotated, never over-concluded. A non-handoff cluster
                     points at a horizon sweep (0.5 s), not fix rejection.
  R10 RTF PARITY     per-flight RTF = d(t_sim)/d(t_wall) from the flight CSV;
                     arm medians printed. Arms may fly hours apart and
                     ADR-0015's load confound is why mc_jam_arm.sh carries a
                     DO-NOT-RUN banner. Warn if arm medians spread > 0.15 or
                     any median < 0.35. Report only.

SCOPE: any CLOSED verdict is scoped to speed-12 / gate-8 EXPLICITLY -- at low
target speed the frozen cue stays inside the 8 m gate and the fail-closed
witness silently fails to bite (audit section 1, final line).

Handoff-outcome classification (REAL / RANGE_BIASED_REAL / PHANTOM /
never_terminal / never) is FROZEN here as a copy of scripts/analyze_track_ab.py
as of 2026-07-10 -- deliberately NOT imported, so later edits to that script
cannot silently move this pre-registered verdict.

gt_* columns are SCORING ONLY (offline analysis of logged runs, never control).
Stdlib only; no sim is booted. Safe to run any time.
"""

import argparse
import csv
import math
import os
import re
import sys
from statistics import mean, median

# ---------------------------------------------------------------------------
# Frozen constants (identical to analyze_track_ab.py @ 2026-07-10)
# ---------------------------------------------------------------------------
PK_M = 2.5
REAL_ERR_M = 5.0      # median terminal range-err below this -> REAL camera track
GOOD_ERR_M = 3.0
FALSE_ERR_M = 8.0
REALISH = ("REAL", "RANGE_BIASED_REAL")

# Pre-registered gate numbers (see module docstring).
EXPECT_N = 16
FIXOFF_MAX_REALISH = 6
FIXOFF_NEVER_MARGIN = 6
FIXON_MIN_REALISH = 11
JOINT_MIN_FIXON = 8
STRICT_EQ_MEDIAN_M = 0.7
STRICT_EQ_ALPHA = 0.05
STRICT_EQ_MIN_AGREE = 14
ACTIVE_REALISH_SLACK = 2
ACTIVE_MIN_ENGAGED = 6
MIN_FLIGHT_ROWS = 50
RTF_SPREAD_WARN = 0.15
RTF_FLOOR_WARN = 0.35

RE_CUTOFF = re.compile(r"\[s2-cue\] LINK CUTOFF at t_sim=([0-9.]+) \(R=([^)]+?) m\)")
RE_STALE = re.compile(
    r"ADR-0059: cue STALE \(silent > ([0-9.]+)s?\s*sim\) at t_sim=([0-9.]+)")
RE_REJECTS = re.compile(r"handoff_cue_rejects=(\d+)")


# ---------------------------------------------------------------------------
# Flight-level scanning
# ---------------------------------------------------------------------------
def scan_flight(flight_csv):
    """One pass over a per-tick flight CSV. Returns None if missing/unreadable.

    Combines analyze_track_ab.flight_terminal (frozen) with the extras this
    verdict needs: row count, DASH-reached, pre-handoff cue_stale ticks, RTF.
    """
    if not flight_csv or not os.path.exists(flight_csv):
        return None
    abs_errs = []
    n_engage_det = 0
    post_min_gt = None
    seen_engage = False
    n_rows = 0
    has_dash = False
    stale_ticks = 0            # cue_stale==1 ticks (pre-handoff by construction:
    #                            m4 gates the predicate on `not handoff_done`)
    t0 = t1 = s0 = s1 = None   # wall / sim spans for RTF
    try:
        with open(flight_csv) as f:
            for r in csv.DictReader(f):
                n_rows += 1
                phase = r.get("phase")
                if phase == "DASH":
                    has_dash = True
                try:
                    tw = float(r["t"]); ts = float(r["t_sim"])
                    if t0 is None:
                        t0, s0 = tw, ts
                    t1, s1 = tw, ts
                except (ValueError, KeyError, TypeError):
                    pass
                if r.get("cue_stale") == "1" and phase not in ("ENGAGE", "BREAKOFF"):
                    stale_ticks += 1
                # --- frozen flight_terminal logic ---
                if phase == "ENGAGE":
                    seen_engage = True
                if seen_engage and phase in ("ENGAGE", "BREAKOFF"):
                    try:
                        g = float(r["gt_range"])
                        if post_min_gt is None or g < post_min_gt:
                            post_min_gt = g
                    except (ValueError, KeyError, TypeError):
                        pass
                if phase != "ENGAGE" or r.get("detected") != "1":
                    continue
                try:
                    mr = float(r["meas_range"]); gr = float(r["gt_range"])
                except (ValueError, KeyError, TypeError):
                    continue
                n_engage_det += 1
                abs_errs.append(abs(mr - gr))
    except OSError:
        return None
    rtf = None
    if t0 is not None and t1 is not None and (t1 - t0) > 1.0:
        rtf = (s1 - s0) / (t1 - t0)
    out = dict(n_rows=n_rows, has_dash=has_dash, stale_ticks=stale_ticks,
               rtf=rtf, n_engage_det=n_engage_det, post_min_gt=post_min_gt,
               med_abs_err=None, n_good=0, n_false=0)
    if abs_errs:
        out.update(med_abs_err=median(abs_errs),
                   n_good=sum(1 for e in abs_errs if e <= GOOD_ERR_M),
                   n_false=sum(1 for e in abs_errs if e > FALSE_ERR_M))
    return out


def classify(handoff, term):
    """FROZEN copy of analyze_track_ab.classify @ 2026-07-10."""
    if not handoff:
        return "never"
    if term is None or term["n_engage_det"] == 0:
        return "never_terminal"
    med = term["med_abs_err"]
    if med <= REAL_ERR_M:
        return "REAL"
    if med <= FALSE_ERR_M and term["n_false"] == 0:
        return "RANGE_BIASED_REAL"
    return "PHANTOM"


def parse_run_log(run_log):
    """Extract runtime evidence lines from one flight's run log."""
    out = dict(exists=False, cutoff_t=None, cutoff_r=None,
               stale_t=None, stale_horizon=None, rejects=None)
    if not run_log or not os.path.exists(run_log):
        return out
    out["exists"] = True
    try:
        with open(run_log, errors="replace") as f:
            for line in f:
                m = RE_CUTOFF.search(line)
                if m and out["cutoff_t"] is None:
                    out["cutoff_t"] = float(m.group(1))
                    r = m.group(2).strip()
                    out["cutoff_r"] = None if r == "n/a" else float(r)
                m = RE_STALE.search(line)
                if m and out["stale_t"] is None:
                    out["stale_horizon"] = float(m.group(1))
                    out["stale_t"] = float(m.group(2))
                m = RE_REJECTS.search(line)
                if m:
                    out["rejects"] = int(m.group(1))
    except OSError:
        out["exists"] = False
    return out


def load_arm(label, agg_path, expect_n, need_runlog):
    """Load one arm: aggregate rows + per-flight scans + infra classification."""
    if not os.path.exists(agg_path):
        return None, [f"aggregate CSV missing: {agg_path}"]
    with open(agg_path) as f:
        rows = list(csv.DictReader(f))
    flights, problems = [], []
    if len(rows) != expect_n:
        problems.append(f"row count {len(rows)} != expected {expect_n}")
    for r in rows:
        try:
            miss = float(r["miss_m"])
        except (ValueError, KeyError, TypeError):
            miss = float("nan")
        fd = None
        try:
            fd = float(r["first_det_range_m"])
        except (ValueError, KeyError, TypeError):
            pass
        term = scan_flight(r.get("flight_csv_path"))
        log = parse_run_log(r.get("run_log_path"))
        handoff = r.get("handoff") == "1"
        # --- infra-failure classification (G1). py_exit!=0 ALONE is not infra:
        # half of r2's rows carry python_exit_1 with complete data. Infra =
        # the evidence needed to score the flight is missing or truncated.
        infra = None
        if term is None:
            infra = "flight CSV missing/unreadable"
        elif term["n_rows"] < MIN_FLIGHT_ROWS:
            infra = f"flight CSV truncated ({term['n_rows']} rows < {MIN_FLIGHT_ROWS})"
        elif not term["has_dash"]:
            infra = "flight never reached DASH"
        elif math.isnan(miss):
            infra = "miss_m not finite"
        elif need_runlog and not log["exists"]:
            infra = "run log missing (jam tripwire unverifiable)"
        flights.append(dict(
            run=r.get("run_idx"), seed=r.get("cue_seed"), dir=r.get("direction"),
            miss=miss, handoff=handoff, hr=r.get("handoff_range_m"),
            ht=r.get("handoff_t_s"), first_det=fd, py_exit=r.get("py_exit_code"),
            term=term, log=log, infra=infra,
            outcome="INFRA" if infra else classify(handoff, term),
        ))
    return flights, problems


# ---------------------------------------------------------------------------
# Stats (stdlib-only, exact where feasible)
# ---------------------------------------------------------------------------
def sign_test(deltas):
    """Two-sided exact sign test on paired deltas (zeros dropped)."""
    nz = [d for d in deltas if d != 0.0]
    n = len(nz)
    if n == 0:
        return 1.0, 0, 0
    k = sum(1 for d in nz if d > 0)
    lo = min(k, n - k)
    p = 2.0 * sum(math.comb(n, i) for i in range(lo + 1)) / 2 ** n
    return min(1.0, p), k, n


def wilcoxon_signed_rank(deltas):
    """Two-sided Wilcoxon signed-rank, normal approx with tie correction
    (n=16 is fine for the approximation; zeros dropped)."""
    nz = [d for d in deltas if d != 0.0]
    n = len(nz)
    if n == 0:
        return 1.0, 0.0
    order = sorted(range(n), key=lambda i: abs(nz[i]))
    ranks = [0.0] * n
    i = 0
    tie_term = 0.0
    while i < n:
        j = i
        while j + 1 < n and abs(nz[order[j + 1]]) == abs(nz[order[i]]):
            j += 1
        avg = (i + j) / 2.0 + 1.0
        t = j - i + 1
        tie_term += t ** 3 - t
        for k in range(i, j + 1):
            ranks[order[k]] = avg
        i = j + 1
    w_plus = sum(rk for rk, d in zip(ranks, nz) if d > 0)
    mu = n * (n + 1) / 4.0
    var = n * (n + 1) * (2 * n + 1) / 24.0 - tie_term / 48.0
    if var <= 0:
        return 1.0, w_plus
    z = (w_plus - mu - math.copysign(0.5, w_plus - mu)) / math.sqrt(var)
    p = math.erfc(abs(z) / math.sqrt(2.0))
    return min(1.0, p), w_plus


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def valid(flights):
    return [f for f in flights if f["infra"] is None]


def count_outcomes(flights):
    oc = {}
    for f in flights:
        oc[f["outcome"]] = oc.get(f["outcome"], 0) + 1
    return oc


def realish_count(flights):
    return sum(1 for f in flights if f["outcome"] in REALISH)


def joint_success(f):
    """HEADLINE per-flight metric: REAL(-ish) handoff AND post-handoff
    min gt_range <= PK_M. Denominator = every flight (no conditioning)."""
    return (f["outcome"] in REALISH and f["term"] is not None
            and f["term"]["post_min_gt"] is not None
            and f["term"]["post_min_gt"] <= PK_M)


def joint_count(flights):
    return sum(1 for f in flights if joint_success(f))


def arm_summary(name, flights, n_total):
    v = valid(flights)
    oc = count_outcomes(flights)
    misses = [f["miss"] for f in v if not math.isnan(f["miss"])]
    post = [f["term"]["post_min_gt"] for f in v
            if f["term"] and f["term"]["post_min_gt"] is not None]
    rpost = [f["term"]["post_min_gt"] for f in v
             if f["outcome"] in REALISH and f["term"]
             and f["term"]["post_min_gt"] is not None]
    jc = joint_count(v)
    lo, hi = wilson(jc, n_total)
    print(f"\n### ARM {name}  (n={len(flights)}, valid={len(v)})")
    order = ["REAL", "RANGE_BIASED_REAL", "PHANTOM", "never_terminal",
             "never", "INFRA"]
    print("  HANDOFF OUTCOME: "
          + "  ".join(f"{k}={oc.get(k, 0)}" for k in order if oc.get(k, 0)))
    if misses:
        hits = sum(1 for m in misses if m <= PK_M)
        print(f"  (i)   pooled miss_m Pk@{PK_M}: {hits}/{len(misses)}  "
              f"median {median(misses):.2f} max {max(misses):.2f} m  "
              f"[whole-flight running min -- credits cue-dash-banked CPAs]")
    if post:
        hits = sum(1 for m in post if m <= PK_M)
        print(f"  (ii)  post-handoff min-gt Pk@{PK_M}: {hits}/{len(post)}  "
              f"median {median(post):.2f} m  [camera-only terminal delivery]")
    if rpost:
        hits = sum(1 for m in rpost if m <= PK_M)
        print(f"  (iii) REAL-handoff-conditioned Pk@{PK_M}: {hits}/{len(rpost)}  "
              f"median {median(rpost):.2f} m  [CONDITIONS on an "
              f"outcome-dependent subset -- NOT the cross-arm headline]")
    print(f"  HEADLINE joint P(REAL-ish handoff AND post-handoff min-gt <= "
          f"{PK_M} m): {jc}/{n_total}  Wilson95 [{lo:.2f}, {hi:.2f}]")
    print("   run dir     miss  postGT  hand  firstDet  outcome")
    for f in flights:
        pg = f["term"]["post_min_gt"] if f["term"] else None
        pg_s = f"{pg:6.2f}" if pg is not None else "   -  "
        fd_s = f"{f['first_det']:5.1f}" if f["first_det"] is not None else "  -  "
        miss_s = f"{f['miss']:6.2f}" if not math.isnan(f["miss"]) else "   nan"
        print(f"   {str(f['run']):>3} {str(f['dir']):<4} {miss_s}  {pg_s}  "
              f"{'Y' if f['handoff'] else 'n'}    {fd_s}    {f['outcome']}")
    return dict(realish=realish_count(v), oc=oc, joint=jc, valid=v)


# ---------------------------------------------------------------------------
# Gate bookkeeping
# ---------------------------------------------------------------------------
GATES = []      # (gate_id, hard, passed, detail)
WARNINGS = []


def gate(gid, passed, detail, hard=True):
    GATES.append((gid, hard, passed, detail))
    tag = "PASS" if passed else ("FAIL" if hard else "WARN")
    print(f"  [{tag}] {gid}: {detail}")
    if not passed and not hard:
        WARNINGS.append(f"{gid}: {detail}")
    return passed


def main():
    ap = argparse.ArgumentParser(
        description="Pre-registered verdict for the ADR-0059 jam MC (task #31).")
    ap.add_argument("--strict", required=True, help="control_strict aggregate CSV")
    ap.add_argument("--fixoff", required=True, help="jam_fixoff aggregate CSV")
    ap.add_argument("--fixon", required=True, help="jam_fixon aggregate CSV")
    ap.add_argument("--active", required=True, help="control_active aggregate CSV")
    ap.add_argument("--r2", default="logs/mc_t21_trackgate_weave12_r2.csv",
                    help="ADR-0058 r2 headline CSV (strict-equivalence baseline)")
    ap.add_argument("--cutoff-range-m", type=float, default=15.0,
                    help="nominal jam cutoff radius R for stratification")
    ap.add_argument("--expect-n", type=int, default=EXPECT_N)
    args = ap.parse_args()

    print("=" * 78)
    print("check_jam_mc.py -- PRE-REGISTERED verdict, ADR-0059 comms-denied jam MC")
    print("Pre-registered 2026-07-10 BEFORE any arm flew "
          "(docs/audit_forward_2026-07-10.md section 1).")
    print("SCOPE: speed-12 / gate-8 ONLY. At low target speed the frozen cue")
    print("stays inside the 8 m gate and the fail-closed witness silently fails")
    print("to bite -- do NOT generalize a CLOSED verdict beyond this point.")
    print("=" * 78)

    specs = [("control_strict", args.strict, False),
             ("jam_fixoff", args.fixoff, True),
             ("jam_fixon", args.fixon, True),
             ("control_active", args.active, False)]
    arms, load_problems = {}, {}
    for name, path, need_log in specs:
        flights, problems = load_arm(name, path, args.expect_n, need_log)
        arms[name] = flights
        load_problems[name] = problems
        if flights is None:
            print(f"\nFATAL: {name}: {problems[0]}")
            print("VERDICT: INCOMPLETE (infra) -- arm CSV missing. Re-fly / "
                  "re-point; this is NOT evidence about the fix.")
            return 1

    n = args.expect_n

    # ------------------------------------------------------------------ G1
    print("\n--- G1 ARM COMPLETENESS (infra failures are NOT 'never') ---")
    infra_total = 0
    for name in arms:
        fl = arms[name]
        infra = [f for f in fl if f["infra"]]
        infra_total += len(infra)
        for f in infra:
            print(f"  INFRA {name} run={f['run']} seed={f['seed']} "
                  f"py_exit={f['py_exit']}: {f['infra']}")
        exit_nonzero = [f for f in fl if f["infra"] is None
                        and f["py_exit"] not in ("0", 0, None, "")]
        if exit_nonzero:
            print(f"  note  {name}: {len(exit_nonzero)} flights exited nonzero "
                  f"but are DATA-COMPLETE (scored normally; r2 itself had "
                  f"7/16 such rows)")
        probs = load_problems[name]
        ok = not probs and not infra
        detail = (f"{name}: {len(fl)} rows, {len(infra)} infra failures"
                  + (f", {'; '.join(probs)}" if probs else ""))
        gate(f"G1[{name}]", ok, detail)

    # ------------------------------------------------------------------ G2
    print("\n--- G2 JAM-FIRED TRIPWIRE (runtime 'LINK CUTOFF at t_sim=' line) ---")
    for name in ("jam_fixoff", "jam_fixon"):
        fl = valid(arms[name])
        fired = [f for f in fl if f["log"]["cutoff_t"] is not None]
        not_fired = [f for f in fl if f["log"]["cutoff_t"] is None]
        for f in not_fired:
            print(f"  NO-CUTOFF {name} run={f['run']} seed={f['seed']} -- the "
                  f"jam never fired (startup banner does not count)")
        gate(f"G2[{name}]", len(not_fired) == 0 and len(fired) == len(fl),
             f"{name}: cutoff fired in {len(fired)}/{len(fl)} valid flights")
    # No-jam arms must NOT show a cutoff (wiring check, hard: a cutoff in a
    # control arm means the arms were mislabeled/mis-pointed).
    for name in ("control_strict", "control_active"):
        stray = [f for f in valid(arms[name]) if f["log"]["cutoff_t"] is not None]
        gate(f"G2[{name}-nojam]", len(stray) == 0,
             f"{name}: {len(stray)} stray LINK CUTOFF lines (must be 0)")

    # ------------------------------------------------------------- summaries
    S = {name: arm_summary(name, arms[name], n) for name in arms}

    # ------------------------------------------------------------------ G3
    print("\n--- G3 PRE-REGISTERED THRESHOLDS ---")
    strict_never = S["control_strict"]["oc"].get("never", 0)
    fixoff_never = S["jam_fixoff"]["oc"].get("never", 0)
    gate("G3a[fixoff-witness]",
         S["jam_fixoff"]["realish"] <= FIXOFF_MAX_REALISH
         and fixoff_never >= strict_never + FIXOFF_NEVER_MARGIN,
         f"REAL-ish {S['jam_fixoff']['realish']}/{n} (need <= {FIXOFF_MAX_REALISH}) "
         f"AND never {fixoff_never} >= strict never {strict_never} + "
         f"{FIXOFF_NEVER_MARGIN} (fail-closed collapse reproduced)")
    fixon_phantom = S["jam_fixon"]["oc"].get("PHANTOM", 0)
    gate("G3b[fixon-recovery]",
         S["jam_fixon"]["realish"] >= FIXON_MIN_REALISH and fixon_phantom == 0,
         f"REAL-ish {S['jam_fixon']['realish']}/{n} (need >= {FIXON_MIN_REALISH}; "
         f"r2 baseline 14/16) AND PHANTOM={fixon_phantom} (need 0)")
    jf_on, jf_off = S["jam_fixon"]["joint"], S["jam_fixoff"]["joint"]
    gate("G3c[headline-joint]",
         jf_on >= JOINT_MIN_FIXON and jf_on > jf_off,
         f"joint P(REAL-ish AND post-gt<= {PK_M}) fixon {jf_on}/{n} "
         f"(need >= {JOINT_MIN_FIXON} and > fixoff {jf_off}/{n}) -- "
         f"unconditioned denominator kills the easy-survivor bias")
    js, ja = S["control_strict"]["joint"], S["control_active"]["joint"]
    print(f"  headline joint, all arms: strict {js}/{n}  active {ja}/{n}  "
          f"fixoff {jf_off}/{n}  fixon {jf_on}/{n}")

    # ------------------------------------------------------------------ G4
    print("\n--- G4 STRATIFY BY FIRST-DET vs CUTOFF "
          f"(R_nominal={args.cutoff_range_m:.0f} m) ---")
    R = args.cutoff_range_m
    fon = S["jam_fixon"]["valid"]
    easy = [f for f in fon if f["first_det"] is not None and f["first_det"] >= R]
    hard_s = [f for f in fon if f["first_det"] is not None and f["first_det"] < R]
    nodet = [f for f in fon if f["first_det"] is None]
    hard_recovered = sum(1 for f in hard_s if f["outcome"] in REALISH)
    easy_recovered = sum(1 for f in easy if f["outcome"] in REALISH)
    print(f"  fixon strata: easy(first-det >= {R:.0f} m, camera saw target "
          f"BEFORE jam)={len(easy)} (recovered {easy_recovered})  "
          f"hard(detected AFTER cutoff)={len(hard_s)} (recovered "
          f"{hard_recovered})  no-det={len(nodet)}")
    st_easy = sum(1 for f in S["control_strict"]["valid"]
                  if f["first_det"] is not None and f["first_det"] >= R)
    print(f"  (control_strict easy-stratum size {st_easy}/{n}; r2 showed 6/16 "
          f"first-det >= 15 m, up to 23.04 m -- ~40% of pairs are jammed AFTER "
          f"the camera already saw the target)")
    if len(hard_s) >= 4:
        need = math.ceil(len(hard_s) / 2)
        ok4 = hard_recovered >= need
        det4 = (f"detected-AFTER-cutoff stratum: {hard_recovered}/{len(hard_s)} "
                f"REAL-ish recovered (need >= {need})")
    else:
        need = 1
        ok4 = hard_recovered >= need
        det4 = (f"hard stratum too thin (n={len(hard_s)} < 4): degraded gate "
                f">= 1 recovered ({hard_recovered} seen) -- verdict annotated "
                f"WEAK-STRATUM")
        if ok4:
            WARNINGS.append("G4: hard stratum n < 4 -- recovery evidence in the "
                            "genuinely denied case is WEAK at this n")
    gate("G4[hard-stratum]", ok4, det4)
    print("  NOTE: the CUTOFF_RANGE_M=18 sweep arm (mc_jam_arm.sh env hook) is "
          "the genuinely PRE-detection follow-up; R=15 only guarantees the jam "
          "precedes HANDOFF, not first detection.")

    # ------------------------------------------------------------------ G5
    print("\n--- G5 control_strict EQUIVALENCE vs r2 (inert-fix check, "
          "paired by cue_seed) ---")
    r2_flights, _ = load_arm("r2", args.r2, args.expect_n, False)
    if r2_flights is None:
        gate("G5[strict~r2]", False, f"r2 baseline CSV missing: {args.r2}")
    else:
        r2map = {f["seed"]: f for f in r2_flights}
        pairs = [(f, r2map[f["seed"]]) for f in S["control_strict"]["valid"]
                 if f["seed"] in r2map
                 and not math.isnan(f["miss"])
                 and not math.isnan(r2map[f["seed"]]["miss"])]
        deltas = [f["miss"] - g["miss"] for f, g in pairs]
        agree = sum(1 for f, g in pairs if f["handoff"] == g["handoff"])
        print("   seed      dir   strict     r2   delta   hand(strict/r2)")
        for (f, g), d in zip(pairs, deltas):
            print(f"   {str(f['seed']):>7} {str(f['dir']):<4} {f['miss']:6.2f}  "
                  f"{g['miss']:6.2f}  {d:+6.2f}   "
                  f"{'Y' if f['handoff'] else 'n'}/{'Y' if g['handoff'] else 'n'}")
        if deltas:
            ad = [abs(d) for d in deltas]
            med_ad = median(ad)
            p_sign, k_pos, n_nz = sign_test(deltas)
            p_wil, w_plus = wilcoxon_signed_rank(deltas)
            print(f"  n={len(deltas)} pairs  median|d|={med_ad:.3f} m  "
                  f"mean_signed={mean(deltas):+.3f} m")
            print(f"  sign test: {k_pos}+/{n_nz} nonzero, two-sided p={p_sign:.3f}   "
                  f"Wilcoxon signed-rank: W+={w_plus:.1f}, p={p_wil:.3f}")
            print(f"  max|d|={max(ad):.3f} m -- DIAGNOSTIC ONLY, NOT a gate "
                  f"(P(max|d|<1 m over 16 pairs under the null) ~ 3e-5; gating "
                  f"on max|d| falsely fails an inert fix)")
            print("  MDE ~0.5 m: n=16 pairs with ~1 m single-flight terminal "
                  "noise gives sigma_d~1.4 m, SE(median)~1.25*1.4/sqrt(16)"
                  "~0.44 m -- shifts below ~0.5 m are UNDETECTABLE at this n; "
                  "'equivalent' is scoped above that floor.")
            gate("G5[strict~r2]",
                 med_ad < STRICT_EQ_MEDIAN_M and p_sign >= STRICT_EQ_ALPHA
                 and p_wil >= STRICT_EQ_ALPHA and agree >= STRICT_EQ_MIN_AGREE,
                 f"median|d| {med_ad:.3f} < {STRICT_EQ_MEDIAN_M} m AND "
                 f"sign p {p_sign:.3f} >= {STRICT_EQ_ALPHA} AND Wilcoxon p "
                 f"{p_wil:.3f} >= {STRICT_EQ_ALPHA} AND handoff agreement "
                 f"{agree}/{len(pairs)} >= {STRICT_EQ_MIN_AGREE}")
        else:
            gate("G5[strict~r2]", False, "no valid seed pairs vs r2")

    # ------------------------------------------------------------------ G6
    print("\n--- G6 control_active vs control_strict (ARM-LEVEL ONLY) ---")
    print("  Per-seed deltas vs r2 are deliberately NOT computed here: the fix")
    print("  is identical code with a different horizon, and trajectories")
    print("  legitimately diverge once the fallback engages on a dropout burst.")
    print("  There is NO 'expect 16/16' bar (the harness's own 15/16-noise "
          "caveat).")
    act = S["control_active"]["valid"]
    engaged = [f for f in act if f["term"] and f["term"]["stale_ticks"] > 0]
    eng_ticks = sorted((f["term"]["stale_ticks"] for f in engaged), reverse=True)
    print(f"  fallback ENGAGEMENT (pre-handoff cue_stale=1 ticks): "
          f"{len(engaged)}/{len(act)} flights engaged "
          f"(tick counts: {eng_ticks if eng_ticks else 'none'})")
    print("  expected ~70-80% of flights (Markov mean burst 1.5 s > 1.0 s "
          "horizon)")
    gate("G6a[active-engagement]", len(engaged) >= ACTIVE_MIN_ENGAGED,
         f"engaged {len(engaged)}/{len(act)} (need >= {ACTIVE_MIN_ENGAGED} or "
         f"the arm is WEAK EVIDENCE for always-on safety)", hard=False)
    act_phantom = S["control_active"]["oc"].get("PHANTOM", 0)
    need_realish = S["control_strict"]["realish"] - ACTIVE_REALISH_SLACK
    gate("G6b[active~strict]",
         S["control_active"]["realish"] >= need_realish and act_phantom == 0,
         f"REAL-ish {S['control_active']['realish']}/{n} >= strict "
         f"{S['control_strict']['realish']} - {ACTIVE_REALISH_SLACK} "
         f"AND PHANTOM={act_phantom} (need 0)")

    # ------------------------------------------------------------------ R7
    print("\n--- R7 DIFFERENCE-IN-DIFFERENCES (report only) ---")
    did_joint = (jf_on - jf_off) - (ja - js)
    did_realish = ((S["jam_fixon"]["realish"] - S["jam_fixoff"]["realish"])
                   - (S["control_active"]["realish"]
                      - S["control_strict"]["realish"]))
    print(f"  joint successes: (fixon {jf_on} - fixoff {jf_off}) - "
          f"(active {ja} - strict {js}) = {did_joint:+d} /{n}")
    print(f"  REAL-ish handoffs: DiD = {did_realish:+d} /{n}")
    print("  isolates the jam-specific effect of the fix from any always-on "
          "cost/benefit under realistic dropout")

    # ------------------------------------------------------------------ R8
    print("\n--- R8 PER-SEED PAIRED fixoff vs fixon (which seeds recovered) ---")
    print("  (analyze_track_ab.paired() only auto-triggers on 'track'/'plain' "
          "labels -- built here)")
    offmap = {f["seed"]: f for f in arms["jam_fixoff"]}
    n_rec = n_both = n_lost = 0
    print("   seed      dir   off_out              on_out               "
          "off_miss on_miss  recovered?")
    for f in arms["jam_fixon"]:
        g = offmap.get(f["seed"])
        if g is None:
            continue
        rec = (g["outcome"] not in REALISH and f["outcome"] in REALISH)
        both = (g["outcome"] in REALISH and f["outcome"] in REALISH)
        lost = (g["outcome"] in REALISH and f["outcome"] not in REALISH)
        n_rec += rec; n_both += both; n_lost += lost
        gm = f"{g['miss']:6.2f}" if not math.isnan(g["miss"]) else "   nan"
        fm = f"{f['miss']:6.2f}" if not math.isnan(f["miss"]) else "   nan"
        tag = "RECOVERED" if rec else ("both-ok" if both
                                       else ("LOST-BY-FIX" if lost else "-"))
        print(f"   {str(f['seed']):>7} {str(f['dir']):<4} {g['outcome']:<20} "
              f"{f['outcome']:<20} {gm}  {fm}   {tag}")
    print(f"  recovered-by-fix={n_rec}  ok-under-both={n_both}  "
          f"lost-by-fix={n_lost}")

    # ------------------------------------------------------------------ R9
    print("\n--- R9 ATTRIBUTION (staleness transient vs real fix failure) ---")
    print("   arm         run  cutoff_t  stale_t  gap_s  rejects  handoff_t  "
          "outcome")
    for name in ("jam_fixoff", "jam_fixon", "control_active"):
        for f in arms[name]:
            lg = f["log"]
            if name == "control_active" and lg["stale_t"] is None:
                continue
            ct = f"{lg['cutoff_t']:8.2f}" if lg["cutoff_t"] is not None else "     -  "
            st = f"{lg['stale_t']:7.2f}" if lg["stale_t"] is not None else "    -  "
            gap = (f"{lg['stale_t'] - lg['cutoff_t']:5.2f}"
                   if lg["stale_t"] is not None and lg["cutoff_t"] is not None
                   else "  -  ")
            rj = str(lg["rejects"]) if lg["rejects"] is not None else "-"
            ht = f["ht"] or "-"
            print(f"   {name:<11} {str(f['run']):>3}  {ct}  {st}  {gap}  "
                  f"{rj:>7}  {str(ht):>9}  {f['outcome']}")
    print("  reading: stale_t - cutoff_t ~ the 1.0 s staleness horizon = the")
    print("  TRANSIENT window (frozen cue still gating; handoff_cue_rejects "
          "banked")
    print("  there are the transient, not a fix defect). fixoff never goes "
          "stale")
    print("  (horizon 1e9) -- rejects there are the BUG signature itself.")
    fon_nonhand = [f for f in S["jam_fixon"]["valid"]
                   if f["outcome"] not in REALISH]
    if fon_nonhand:
        print(f"  AMBIGUITY NOTE: {len(fon_nonhand)} fixon non-REAL-ish "
              f"flight(s) (runs "
              f"{[f['run'] for f in fon_nonhand]}). The arms carry no "
              f"--coast-search, so each is AMBIGUOUS between cue-gate failure "
              f"and FOV/steering loss -- annotate, don't over-conclude. A "
              f"cluster here triggers a --cue-stale-horizon sweep (0.5 s), "
              f"NOT fix rejection.")

    # ------------------------------------------------------------------ R10
    print("\n--- R10 RTF PARITY (ADR-0015 load confound; report only) ---")
    med_rtfs = {}
    for name in arms:
        rtfs = [f["term"]["rtf"] for f in valid(arms[name])
                if f["term"] and f["term"]["rtf"] is not None]
        if rtfs:
            med_rtfs[name] = median(rtfs)
            print(f"  {name:<15} median RTF {median(rtfs):.3f}  "
                  f"min {min(rtfs):.3f}  max {max(rtfs):.3f}  (n={len(rtfs)})")
        else:
            print(f"  {name:<15} RTF unavailable")
    if med_rtfs:
        spread = max(med_rtfs.values()) - min(med_rtfs.values())
        if spread > RTF_SPREAD_WARN:
            WARNINGS.append(f"R10: arm median RTFs spread {spread:.2f} > "
                            f"{RTF_SPREAD_WARN} -- arms flew under unequal "
                            f"load (ADR-0015); treat cross-arm deltas with "
                            f"suspicion")
        low = [k for k, v in med_rtfs.items() if v < RTF_FLOOR_WARN]
        if low:
            WARNINGS.append(f"R10: arm(s) {low} median RTF < {RTF_FLOOR_WARN} "
                            f"-- flown under load, idle-load rule violated?")
        print(f"  arm-median spread {spread:.3f} "
              f"(warn > {RTF_SPREAD_WARN}); arms may fly hours apart around "
              f"other jobs -- this is why mc_jam_arm.sh carries the "
              f"DO-NOT-RUN-UNDER-LOAD banner")

    # ---------------------------------------------------------------- verdict
    print("\n" + "=" * 78)
    hard_fails = [(g, d) for g, h, p, d in GATES if h and not p]
    infra_failed = any(g.startswith("G1") for g, _ in hard_fails)
    for w in WARNINGS:
        print(f"WARNING: {w}")
    print("-" * 78)
    if not hard_fails:
        print("VERDICT: FIX VALIDATED -- jam_fixon recovers handoffs under the")
        print("mid-dash cue jam, jam_fixoff reproduces the ADR-0059 fail-closed")
        print("witness, and both controls are statistically equivalent (inert")
        print("when fresh, safe under realistic dropout).")
        print(f"SCOPE: speed-12 / gate-8 / CUTOFF_RANGE_M as flown ONLY. The "
              f"R=18 sweep remains the genuinely pre-detection case.")
        if WARNINGS:
            print("Scoping warnings above apply -- carry them into the ADR.")
        print("=" * 78)
        return 0
    if infra_failed:
        print("VERDICT: INCOMPLETE (INFRA) -- one or more arms have missing/")
        print("truncated evidence. Re-fly or repair the arm; this is NOT")
        print("evidence about the fix (do not read infra rows as 'never').")
    else:
        print("VERDICT: NOT VALIDATED -- pre-registered gate(s) failed:")
    for g, d in hard_fails:
        print(f"  FAILED {g}: {d}")
    print("=" * 78)
    return 1


if __name__ == "__main__":
    sys.exit(main())
