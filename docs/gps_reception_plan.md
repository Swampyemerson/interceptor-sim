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
at this site with nothing near it.

## Why a mount helps, in three sentences

GPS signals arrive from above and are extremely weak; the noise comes from below. A flat
metal plate under the unit, wider than the unit, improves what it hears from above and
blocks what comes from below; about 8 to 10 cm across is the useful size (half the 19 cm
wavelength). Distance does the rest: the stray fields close to a cable fall off very
fast, and a magnetic field from a wire or magnet falls with the cube of distance, so
going from 3 cm to 10 cm cuts the compass disturbance by about 37 times.

## Step 1: measure before printing (one bench session outdoors, props off, about 30 min)

The aircraft sits in the open, camera streaming the whole time, 90 s per row. I read
satellites used, the unit's speed accuracy and its jamming figure from the log, and the
compass field strength from the estimator.

| Row | Arrangement |
|---|---|
| R | **Reference:** GPS unit as far from the aircraft as its cable allows, on a non-metal stand, plate under it. This is the best this unit can do here. |
| 0 | As flown today (on the lid, foil on the ribbon). |
| 1 | On the lid, plus a flat plate about 9 cm across under the unit. |
| 2 | 5 cm above the lid on a non-metal spacer, with the plate. |
| 3 | 10 cm above the lid, with the plate. |
| 4 | Row 3 with the ribbon shield properly made: insulated, wrapped, and connected to a ground point at the Pi end. |
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

## Step 2: the printed mount (after step 1 picks the geometry)

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
- One current set of print files only; nothing is drawn until step 1 is in.

## Step 3: settings, each on the builder's yes

| Setting | Now | Proposed | Why |
|---|---|---|---|
| Height reference (`EKF2_HGT_REF`) | GPS | Barometer | The aircraft followed 8 m of GPS altitude wander in flight 3. Can be done now. |
| Position-mode response (`MPC_ACC_HOR`, `MPC_JERK_MAX`) | 3 m/s2, 8 m/s3 | 5 m/s2, 20 m/s3 | Stick-to-tilt delay was 0.36 to 0.60 s against 0.14 s in Altitude mode. A matter of feel; sharper values are possible. |
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
