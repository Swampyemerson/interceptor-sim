# Plan: a clean GPS and compass on the interceptor (written 2026-11-22)

Three hand flights on 2026-11-22 all failed the same way: the estimator dropped the GPS,
picked it up again, and everything downstream (position hold, height hold, heading,
geofence) went wrong from there. This page is the plan to fix that at the source. Evidence:
`runs/interceptor_flight_20261122/`, `runs/interceptor_flight2_20261122/`,
`runs/interceptor_flight3_20261122/`, `runs/interceptor_arming_gps_20261122/`.

## What is known

| Finding | Measured |
|---|---|
| The camera and its ribbon cable are the main jammer | Camera off: 6 to 7 satellites. Camera streaming, bare ribbon: 0, fix lost within a second. Processor load alone: one satellite. |
| Foil round the ribbon removes most of it | Ground: fix held at 7. In flight: 11 satellites (flight 2), 9 with dips to 6 (flight 3). |
| What is left is still too weak | The unit rates its own speed accuracy at 0.5 to 0.9 m/s; a healthy one says 0.2 to 0.3. It had 8 to 10 satellites indoors on the old bench rig, away from the Pi. |
| The compass in the GPS unit hears the battery leads | 0.0036 G per amp: 0.28 G at a 77 A punch, half the Earth's field. |
| The height estimate follows GPS altitude | 8 m of wander against the barometer in flight 3. |

Not known: how much the Pi and the AI board cost with the camera fully shielded; whether
the motors and speed controllers add noise (never tested apart); how good this GPS unit is
at this site with nothing near it; whether noise also travels up the GPS unit's own cable,
which runs down past the Pi (foil does nothing for that).

**Layers are not the lever (added 2026-11-24).** Radio current at the GPS frequency only
flows in the outer 2 microns of aluminium (skin depth = sqrt(resistivity / (pi x frequency
x permeability)) = 2.06 microns at 1575 MHz), and kitchen foil is several times thicker, so
one layer already stops what it covers. What leaks is the gaps: the open ends of the ribbon
wrap at both connectors, any seam, and a shield that is not connected to ground and so
carries the noise along instead of draining it. More layers only help if they close a gap.

## Why a mount helps, in three sentences

GPS signals arrive from above and are extremely weak; the noise comes from below. A flat
metal plate under the unit, wider than the unit, improves what it hears from above and
blocks what comes from below; about 8 to 10 cm across is the useful size (half the 19 cm
wavelength). Distance does the rest for the radio noise: the stray fields close to a cable fall off
very fast.

**Corrected 2026-11-25: the compass gain is smaller than this page first said, and not
yet known.** The first version said 3 cm to 10 cm cuts the compass disturbance 37 times.
That is true 3 cm from a small magnet, and it is the wrong case here. The disturbance
measured is the battery leads, which are further away, and the mast adds 51 mm to
whatever that distance is today. A pair of leads falls with the square of distance, a
small loop with the cube. Worked from the measured 0.0036 G per amp
(`hardware/avionics_pod/FIELD_STUDY_BRIEF.md`, study A): 3.5 to 6 times better if the
leads are 60 mm below the compass today, 2.2 to 3.2 times if they are 110 mm below. The
0.001 G per amp target needs 3.6 times.

**The builder's description (2026-11-25, corrected the same day after looking again):**
the leads are inside the body and below it, red and black within 5 mm of each other
except the last 20 mm at the speed controller. A pair that tight gives 0.0010 to
0.0025 G per amp at 100 to 63 mm, so the leads explain a quarter to two thirds of the
measured 0.0036 and **the rest of the source is not identified** (candidates: the split
at the capacitor, the speed controller board, the battery pack's internal path, the
power module). The builder then placed the capacitor split about 70 mm below the GPS:
a 20 mm split 10 to 15 mm wide adds 0.0009 to 0.0014 G per amp, so leads and split
together explain most of the measurement, and the mast would bring it to 0.0008 to
0.0013 against the 0.001 target (working: `hardware/avionics_pod/FIELD_STUDY_BRIEF.md`,
study A). The mast's compass gain is therefore 2.8 to 4.8 times and unproven. An
earlier version of this paragraph said taping the leads would give 4 times; that rested
on a 20 mm gap that does not exist. The builder is tidying the wiring (leads, 5 V runs
moved lower, the ribbon's fold moved from under the lid); the compass pickup and the GPS
are remeasured after that. The ribbon, as flown on 2026-11-22, folded back on itself
directly under the lid, 2 to 9 mm beneath the GPS.

## Step 1: measure before printing (one bench session outdoors, props off, about 30 min)

The aircraft sits in the open, camera streaming the whole time, 90 s per row. I read
satellites used, the unit's speed accuracy and its jamming figure from the log, and the
compass field strength from the estimator. I also record the signal strength of each
satellite (C/N0, in dB-Hz: how far each signal stands above the noise). It moves before the
satellite count does, so it shows a small gain or loss that the count hides. It needs one
diagnostic setting on the flight controller (`GPS_SAT_INFO` = 1, then a restart), on the
builder's yes. It is recorded for information; the pass mark below is unchanged.

| Row | Arrangement |
|---|---|
| R | **Reference:** GPS unit as far from the aircraft as its cable allows, on a non-metal stand, plate under it. This is the best this unit can do here. |
| 0 | As flown today (on the lid, foil on the ribbon). |
| 1 | On the lid, plus a flat plate about 9 cm across under the unit. |
| 2 | 5 cm above the lid on a non-metal spacer, with the plate. |
| 3 | 10 cm above the lid, with the plate. |
| 3g | Row 3 with the plate joined to ground by one short wire. Tells us whether the deck's foil needs a ground wire at all. |
| 4 | Row 3 with the ribbon shield properly made: insulated, one layer wrapped with no gaps right up to both connectors, and connected to a ground point at the Pi end. |
| 4c | Row 4 with the GPS cable routed clear of the ribbon and a clip-on ferrite on it **at the flight-controller end only**. Ferrite is magnetic and the compass is inside the GPS unit, so none goes within 3 cm of the unit. |
| 5 | The best row so far with the motors running, props off, at about hover throttle. |

**Pass mark, fixed now:** camera streaming, the row is within 2 satellites of row R and its
speed accuracy is under 0.35 m/s (median), and starting the camera costs no more than one
satellite. **The smallest arrangement that passes is the one to print.** If no row passes,
the unit goes on a longer cable to the tail or the camera link needs a properly shielded
ribbon before any mount will fix it; that would be the finding, and printing waits.
**If row R itself is poor** (under 12 satellites), the site or the unit is the limit and
the test moves somewhere more open before anything is concluded.

Second number from the same session: with a pack on and a motor run, how far the compass
field moves per amp at each height. Target: under 0.001 G per amp, which is 0.08 G at 77 A.

## Step 2: the printed mount

**Status 2026-11-22: drawn and analysed at the builder's go-ahead, not printed.** It
replaces the pod's lid with two parts, `pod_lid_mast` and `gps_deck`
(`hardware/avionics_pod/README.md`, section "GPS mast and deck"). As drawn: GPS antenna
94.6 mm above the Pi board, an 88 mm foil ground plane under it, 21.7 g more than the
plain lid, first natural frequency 130 Hz (114 Hz with soft plastic). The mast height
is one number in the generator, so step 1's result can still change it before a print.
The points below are what the drawing was held to.

- **Stiff, not tall.** A short tripod or a wide-based column, not a thin stalk. The GPS and
  plate weigh about 25 g; the mount's first wobble should sit above 80 Hz, clear of the
  20 to 60 Hz band where the airframe already vibrates. I will compute that before
  printing and it gets a tap test after.
- **A plate bay under the unit**, flat, sized from step 1, for copper or aluminium foil
  tape or thin sheet (not steel).
- **No steel within 3 cm of the unit**: nylon or brass screws, brass inserts.
- **Breakaway base** (nylon screws) so a crash takes the mount, not the pod lid.
- **Cable strain relief**, the unit's arrow forward, and the cable kept away from the
  camera ribbon.
- One current set of print files only: the plain lid's files were deleted the day the
  mast was drawn.

## Step 3: settings, each on the builder's yes

| Setting | Now | Proposed | Why |
|---|---|---|---|
| Height reference (`EKF2_HGT_REF`) | GPS | Barometer | The aircraft followed 8 m of GPS altitude wander in flight 3. **Approved 2026-11-22; written and read back 2026-11-25; not yet flown.** |
| Position-mode response (`MPC_ACC_HOR`, `MPC_JERK_MAX`) | 3 m/s2, 8 m/s3 | 5 m/s2, 20 m/s3 | Stick-to-tilt delay was 0.36 to 0.60 s against 0.14 s in Altitude mode. **Approved 2026-11-22; written and read back 2026-11-25; not yet flown.** |
| GPS speed-accuracy limit (`EKF2_REQ_SACC`) | 1.0 m/s | back to 0.5 | Only once the mount is fitted and the unit reports under 0.35. |
| Pack capacity (`BAT1_CAPACITY`) | unset | 1350 | Only once position is dependable (it misfired a failsafe in flight 2). |

## Step 4: checks around it

1. **Compass:** recalibrate outdoors, away from cars, steel tables and reinforced concrete.
   Afterwards I read the estimator's field strength: it should be within 0.03 G of 0.513
   (it reads 0.39 to 0.42 after today's recalibration; it was 0.49 before).
2. **Pod fasteners and a tap test**: lateral vibration grew across the three flights.
3. **One flight to prove it**: Position mode held for 60 s, estimator height within 1 m of
   the barometer, no "blind land", no compass fault. Then straight runs for lean angle.
4. **Charger reading** after a flight (mAh put back) to settle the pack's real capacity.
