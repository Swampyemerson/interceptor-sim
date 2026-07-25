# Error-handling policy — how this repo fails LOUDLY

> **Codified 2026-07-25** from the two same-day silent-failure reviews
> (`docs/review2_silent_failure_findings.md`: ~15 defects sharing one shape —
> *a confident, plausible, WRONG answer with nothing raised, behind green
> checks*). Nothing here is invented: every rule below is the pattern already
> proven in `scripts/field_score.py` (the model citizen) and
> `scripts/field/common.sh`, promoted to policy. One page; follow it.

## The one-sentence rule

**If a quantity should have been measured, matched, or scored and wasn't,
the tool must say so and exit non-zero — never substitute, never default,
never print a verdict.**

## 1. Raise vs. exit vs. warn

- **Helpers raise.** A library/helper function that hits bad input raises
  `ValueError` **naming the file/quantity** (`field_score.py:372` —
  `f"{path}: empty CSV"`). Helpers never call `sys.exit()`, never print-and-
  continue, never return a guessed value.
- **`main()` catches ONCE and exits 1.** One `try/except` at the entry point
  turns the raised message into a clean one-line error. No other layer
  swallows exceptions.
- **Warnings ride in the report, not to stderr-and-gone.** Non-fatal caveats
  go into the JSON/CSV artifact the run writes (`field_score.py` `warnings=[]`
  threaded into the report at `:627/:633`), so the caveat travels WITH the
  number it qualifies. If a verdict is uncertain, print it on the console
  line that shows the verdict — a caveat buried in a JSON nobody opens is a
  silent failure with paperwork.

## 2. Shell exit-code contract (scripts/field/common.sh — use it everywhere)

| code | meaning | notes |
|---|---|---|
| 0 | **PASS** | ran against the real thing and passed |
| 1 | **FAIL** | the thing is there and it's wrong — read the hint |
| 2 | **USAGE** | operator error / missing file — not a finding |
| 3 | **ABSENT / UNCERTAIN** | could not measure — advisory; `FLD_REQUIRE=1` upgrades to FAIL on field day |

Python verdict tools mirror it (`audit_per_tick.py`: 0/1/2 + **3 = VACUOUS**).
Never encode "could not decide" and "passed" (or "failed"!) in the same code.

## 3. No vacuous verdicts

**A verdict computed on zero units is UNCERTAIN/VACUOUS, never PASS.**
Every verdict line prints *the n it was computed from and the n it required*
("PASS: 12/12 audited flights…"). If a join/filter matched zero rows, or n is
below the stated floor, print VACUOUS/UNDERPOWERED and exit non-zero (code 3).
This class has bitten ≥4 times (tripod zero-row join, CODED_DASH phase filter,
rebal-arm mirage, tags.csv schema); it is now pinned by
`tests/test_no_vacuous_verdicts.py`. Reserve UNCERTAIN for data that was
matched and measured and still cannot decide.

## 4. FAIL-CLOSED on measured quantities

Never substitute a default for something that should have been measured — the
30-fps bug: the money gate divided by an INVENTED frame rate and flipped a
$740 buy decision. If the measurement is missing, return UNCERTAIN **naming
the missing measurement and the one-line fix** (`tripod_score.gate_verdict`'s
`stream_fps=None` path is the template). Config defaults are fine for
*preferences*; they are forbidden for *measurements*, calibrations, and rates.

## 5. Typed sentinels, not bare `None`

A bare `None` return gets absorbed by the next `if x:` and becomes a silent
drop. When a function can decline, return a labeled outcome the caller must
branch on (verdict strings `"PASS"/"FAIL"/"UNCERTAIN"/"VACUOUS"`, a
`(value, source_tag)` pair like `resolve_stream_fps`, or a raised
`ValueError`). If `None` is unavoidable, COUNT the Nones and print the count
("scored N; M dropped: K off-frame, J unreadable") — a denominator that
shrinks without saying so is the whole failure class.

## 6. Producer→consumer pairs get a golden contract test

Any script that reads another script's artifact gets one test that generates
the fixture **from the producer's own writer/header** (never hand-typed) and
runs the REAL consumer on it. Templates: `tests/test_tripod_pipeline_join.py`
(recorder→joiner→scorer), `tests/test_flight_csv_contract.py` (flight CSV /
arm CSV → the three verdict-bearing auditors). Both sides passing their own
tests proves nothing about the seam.

## 7. Green must mean RAN

A check that can skip its own substance must assert a minimum executed count
or preflight its dependencies and FAIL if they're gone (`run_tests.sh` stage 2:
a pytest skip in the parity stage is a failure; stage 4 runs the previously
unreachable `--self-test` entry points). Never cite an unrun self-test as
evidence. The dashboard `--check` states exactly what it checked ("prose
wording NOT checked") — an OK line may not claim more scope than the check has.

## 8. A fix isn't done until its EFFECT is observed end-to-end

Two fixes shipped on 2026-07-24 were inert (the bench still forced the old
weights; the doc still taught the wrong command). After any fix: **run the
real entry point and watch the number/verdict actually change** — the
pre-fix behavior reproduced, then the post-fix behavior demonstrated, both in
the commit message or test docstring. A green suite is corroboration, not
proof; the proof is the observed effect.
