# The hardware model (SysML v2)

Every part that is actually in the build — target drone, interceptor, Tier-1
tripod seeker rig, ground gear — and every wire, cable, radio path and mount
between them, written as a **SysML v2 textual model**. The diagrams in
`views/` and view 8 of the MBSE sheet (`docs/mbse.html`, embedded in the
dashboard Artifact) are drawn from these files. **Edit the text, never the
pictures.**

## 30-second primer

**SysML** (Systems Modeling Language) is the standard notation systems engineers
use to describe a system as a model instead of as loose drawings. **v2** (OMG,
2025) adds a real text syntax, so a model can live in git and be diffed like code.
Five ideas cover everything in this folder:

| Idea | Here | Example |
|---|---|---|
| **part def** — a kind of thing | a catalogue entry for one board | `part def Pixhawk6CMini { … }` |
| **part** — one used in a system | where it sits in an aircraft | `part fc : Pixhawk6CMini { … }` |
| **port** — a plug, socket or pad | typed, so mismatches are catchable | `port telem2 : UARTPort;` |
| **~port** — the *conjugate* | the receiving side of the same plug | `port gpioUart : ~UARTPort;` |
| **interface** — a wire between ports | typed, with build state | `interface companion : UARTLink connect fc.telem2 to pi.gpioUart { … }` |

A UART host port wired to a conjugate UART port **is** "TX/RX crossed" — the
model encodes the crossing in the types. A power supplier owns `DCPowerPort`, the
consumer owns `~DCPowerPort`; a wire drawn the other way round fails the check.

## Files

| File | What's in it |
|---|---|
| `HardwareLibrary.sysml` | port types, wire types (each one is power / data / radio / mechanical / optical), annotations (`@BOM`, `@Layout`, `@External`, `@NotModelled`), `LinkState` |
| `Components.sysml` | one `part def` per kind of hardware, with its real ports |
| `Target.sysml` | the target drone |
| `Interceptor.sysml` | the interceptor (two-rail power: PM02 → FC, Matek BEC → Pi) |
| `SeekerRig.sysml` | the Tier-1 tripod rig — same camera/Pi/card as the interceptor |
| `GroundSegment.sysml` | transmitters, charger, laptop, hotspot + the list of build-sheet rows that are deliberately not drawn |
| `Engagement.sysml` | the context: everything crossing between aircraft, ground and sky |
| `views/*.svg`, `views/*.png` | generated — 9 views; `manifest.json` ties each PNG to the SVG it came from |

## What lives where (single source of truth)

- **Part status** (in hand / ordered / built / print / must-add) is **not in the
  model**. Each part carries `@BOM { tab = "<build_tab id>"; bomName = "<substring>"; }`
  and the renderer reads the live status from `docs/project_state.json → build_tab`.
  Change a status there, re-render, and the diagrams follow.
- **Wiring progress** *is* in the model, on each wire:
  `linkState = planned | wired | verified`. `wired` and `verified` must name the
  build step(s) that prove them: `evidence = "brn-03, brn-05"` — ids that must
  exist in `build_tab` steps.
- **Layout** is the only presentation data: `@Layout { col; row; }` per part. SysML
  carries no layout of its own.

## Common edits

**A wire got soldered / tested.** Change its `linkState` and add `evidence`:
```sysml
attribute :>> linkState = LinkState::verified;
attribute :>> evidence = "afr-03";
```

**A new part.** (1) Add its row to `build_tab` in `project_state.json` (the
renderer will refuse a `@BOM` that matches no row). (2) If it is a new kind of
hardware, add a `part def` with its ports in `Components.sysml`. (3) Add the
`part` with `@BOM` and `@Layout` to the system. (4) Wire it with `interface`s.

**A part swapped for a different model** (e.g. another ESC). New `part def`,
change the `part`'s type, fix its `@BOM`, re-check every wire to it — the port
names and types are exactly what changes between models.

**A build-sheet row that should not be drawn** (a tool, safety gear). Add it to the
`@NotModelled` list at the bottom of `GroundSegment.sysml` with a `reason`.

Then run the ritual below.

## The ritual (same turn as the edit)

```bash
python3 scripts/render_sysml.py --png     # validate + redraw views/ (PNGs need Chromium)
python3 scripts/render_mbse.py            # view 8 of the MBSE sheet
python3 scripts/render_dashboard.py       # the dashboard embeds the MBSE sheet
python3 scripts/render_dashboard.py --artifact /tmp/dash.html   # then republish to artifact_url
scripts/sysml/check_sysml_official.sh     # optional: OMG reference validation (Java)
```
Commit the `.sysml` change together with `views/`, `docs/mbse.html` and
`docs/dashboard.html`. `scripts/run_tests.sh` (stage 3) and CI run
`render_sysml.py --check`, which fails if anything is stale.

## The two validators, and why both

| | `scripts/render_sysml.py --check` | `scripts/sysml/check_sysml_official.sh` |
|---|---|---|
| Is it real SysML v2? | reads a documented **subset** only, fails on anything else | **yes** — the OMG reference implementation (Pilot 0.62.0) |
| Wire into the wrong kind of port / backwards | **caught** | not caught (tested 2026-10-02: accepts a battery wired backwards) |
| Every build-sheet row drawn or excluded | **caught** | n/a |
| `verified` without a real build step | **caught** | n/a |
| Needs | Python stdlib | Java 17+, one-time ~124 MB download (cached) |
| Runs in | `run_tests.sh`, CI, every cloud clone | on demand |

### The subset the renderer accepts

`package` · `import` · `doc /* */` · `// notes` · `abstract`? `part|port|item|attribute|enum|metadata|interface def Name (:> Super, …)?` ·
`enum lit;` · `end name : ~?Type;` · `attribute (:>>)? name (: T | :> T)? (= value)?;` ·
`in|out|inout item name : T;` · `port name : ~?Type (body)?` · `part name : Type ([n])? (body)?` ·
`interface name : Type connect a.b(.c) to d.e(.f) (body)?` · `@Meta { k = v; }` / `@Meta;`.
Values: strings, numbers (optionally `[unit]`), `true/false`, `Enum::literal`.
Anything else fails with file:line — the model is never silently half-read.

**SysML keywords that bit while writing this** (don't use them as names): `ref`,
`ordered`, `state`, `out`, `frame`.

## Open wiring decisions the model surfaced (2026-10-02)

The renderer lists any wire whose text says `TBD` / `UNRESOLVED` / `check at`
as an open decision on the MBSE sheet. At creation there were three:

1. **Interceptor buzzer** — the 6C Mini has no buzzer pad; which pin drives the
   Flywoo Finder is undecided (afr-01).
2. **Interceptor ESC signal lead** — the contract says "JST-SH 8-pin", the bench
   wiring card says the 6C Mini's MAIN outputs are 3-pin S/+/− headers; an
   adapter lead is likely (afr-01).
3. **Pi power entry** — USB-C pigtail or header 5 V/GND pins from the BEC (afr-05).

Also flagged in the model, not a wire: the **2nd Pocket's battery** is not on the
build sheet.
