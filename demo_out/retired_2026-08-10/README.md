# RETIRED demo cuts — do not publish any file in this directory

Builder ruling, 2026-08-10: **retire the onboard demo assets** (GitHub issue #2,
contract queue item `stale-demo-assets`).

## Why

Every cut in here was rendered **2026-07-07** and burns this caption into the
closing frame:

> `CPA 0.63 m  <  1.5 m lethal radius`

After ADR-0084 that reads as a **ram-kill claim, which it is not**. The ram /
contact radius is **0.35 m**; the 1.5 m figure is a *net-class* criterion, a
different weapon concept entirely. Nothing this project has flown has landed
inside 0.35 m — the honest count is 0/16 — so a portfolio-facing asset implying
a contact kill is exactly the overclaim the honesty rules exist to prevent.

The generator was fixed on **2026-07-25** (commit `759e9b0`) and now renders
`NET-CLASS lethal radius` explicitly. Everything rendered *before* that date is
stale; everything after is fine.

## What is in here, and what was NOT retired

Retired (all rendered 2026-07-07, all carry the caption):

| file | what it was |
|---|---|
| `interceptor_onboard.mp4` / `.gif` | the primary seeker-POV hero cut |
| `interceptor_demo.mp4` / `.gif` | the earlier pre-re-cut version |
| `interceptor_chase.mp4` | the external chase view |

Also deleted from git in the same commit: `docs/images/demo_onboard_final.png`
and `docs/images/demo_chase_final.png`, the two tracked stills carrying the same
caption.

**Deliberately kept:** `demo_out/t25/` (re-rendered 2026-07-25, *after* the fix,
and pixel-verified against the ram-radius claim) and `*_raw.mp4` (raw captures
with no banner — source material, not a claim).

## Why these were not simply re-rendered

They cannot be. The hero flight CSV
(`logs/m4_intercept_pronav_20260707T211601Z.csv`, miss 0.632 m) was rotated off
disk and never committed. Re-flying produces a different CPA and desynchronises
the 700+ already-captured frames, so a "re-render" is really a re-shoot. The
result itself survives in the ADR-0032 addendum in `docs/decisions.md`.

## If you want a demo video again

Re-shoot with `scripts/build_demo.py`, which is already correct. Better: shoot it
off a **real** flight once tripod day and the airframe order are through — a real
one is worth more than a re-cut sim anyway.
