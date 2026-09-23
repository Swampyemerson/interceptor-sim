# LinkedIn copy (2026-09-22, red-teamed)

Process: head draft, then a Fable red-team pass verified every number against
the repo and rewrote both drafts. Fixes adopted: no present-tense capability
claims (nothing intercepted outside simulation); "no GPS" corrected because
the GPS cue relay (2026-09-22) feeds a launch cue, so the honest line is
autonomy AFTER launch; the flying detection path is the AprilTag marker, the
markerless NN is in-work; the 345 fitted flights are simulator flights; the
96.6 fps figure is the marker-tracking pipeline only.

Voice rules applied: no em dashes, first person, short declarative sentences,
no marketing language, AI-assisted software disclosed. Terms avoided in
public copy: kill, lethal, warhead. "Intercept" and "contact radius" instead.

Number provenance: parity grid isim/specs/parity_trace_2026-09-22.md,
re-run fresh logs/linkedin_media_20260922/; 6,600-run study plain_log
2026-09-17; 345-flight fit isim/fits/vehicle_gazebo_x500.json; 96.6 fps
runs/skr07_tagged/qd2_uncapped/soak.json (ADR-0090).

## A. Projects-section description (~150 words)

Autonomous camera-guided drone interceptor. Guidance validated in simulation,
hardware now on the bench.

I am building a small quadcopter designed to intercept a fast target drone.
After launch it is on its own: no datalink, no outside tracking. An onboard
Raspberry Pi 5 camera reads a visual marker on the practice target, a Kalman
filter tracks it, and a pursuit law flies the final approach. A markerless
neural-net detector is in work.

I built two simulators to get here: a PX4/Gazebo pipeline, and a fast
Monte-Carlo harness that runs the actual flight code against a vehicle model
fitted to 345 logged simulator flights. A 6,600-run study selected the
guidance concept. A registered tick-level trace then found four estimator
defects in the flight code. After the fix, 50 of 50 seeded nominal runs end
inside the 0.35 m contact radius, median miss 0.068 m. All simulation; flight
test is next. The marker-tracking pipeline runs at a measured 96.6 fps on the
real Pi 5.

Software written with Claude Code under my direction. I specify the tests and
verify every number against logged runs.

## B. Post copy (~200 words)

I am building a drone that can catch another drone. It has to do it with its
own camera. After launch it is on its own: no datalink, no outside tracking.

Two rules govern the project. Every published number must trace to a logged
run. And the simulator's ground truth is firewalled from the guidance code,
with tests that fail if anything reaches through.

Where it stands, all in simulation with the real flight code in the loop: the
terminal chase puts 50 of 50 seeded nominal runs inside the 0.35 m contact
radius against a 9 m/s crossing target, median miss 0.068 m. It holds 100%
with a 10 degree launch aim error. It drops to 70% at 20 degrees and to 76%
when the target is 2 m above the cued altitude, because the flight state
machine has no second approach yet. That is the next design decision.

The hardware is on the bench: frame, flight controller, and a Raspberry Pi 5
whose marker-tracking pipeline measures 96.6 fps. Photos below. Flight video
when it earns it.

The systems model in the images is generated from the project's
machine-readable state file and fails CI if it drifts from the build.
Software written with Claude Code under my direction.

## Open items for the builder

1. The application handoff doc and /areas/interceptor-drone.md (framing
   rules, start date, terms to avoid) live outside this machine. These drafts
   follow the repo's public framing. Check them against those rules before
   posting.
2. Red-team flag, out of scope here: the public README should be re-checked
   against the GPS-cue-relay change (the relay postdates the last claims
   scrub, so "no GPS" phrasing may survive there too).
3. Media set: fig1 robustness grid, fig2 miss CDF, fig3 engagement anatomy,
   MBSE functional architecture, MBSE system boundary, plus bench photos.
   Sim intercept video to follow.
