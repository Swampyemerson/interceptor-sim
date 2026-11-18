"""Mutation check: each fix is switched off in turn and the tests that pin it
must FAIL. The source is restored after every mutant (and at exit)."""
import os
import subprocess
import sys

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RF = WT + "/flight/deploy/real_flight.py"
SUP = WT + "/flight/deploy/loop_supervisor.py"
PY = WT + "/.venv/bin/python"
TESTS = "flight/tests/test_real_flight_driver.py"

MUTANTS = [
    ("G7 own_state() drops the age", RF,
     "                        alt_m=self.alt_m, vel_ned=self.vel_ned,\n                        age_s=self.own_state_age_s)",
     "                        alt_m=self.alt_m, vel_ned=self.vel_ned)",
     "test_own_state_age_reaches_the_terminal"),
    ("G7 FAILSAFE 9 never fires", RF,
     "            own_ok, own_why = self.own_state_status_sm(obs)\n            if not own_ok:",
     "            own_ok, own_why = self.own_state_status_sm(obs)\n            if False:",
     "failsafe9 or frozen_attitude"),
    ("G7 driver does not supply the age", RF,
     "                         det_age_s=det_age, own_state_age_s=_own_age)",
     "                         det_age_s=det_age)",
     "frozen_attitude"),
    ("G6 teardown always stops and lands (old behaviour)", RF,
     "    if name in _PROGRAM_OWNED_MODES:\n        return True, action, f\"aircraft is in {name}, which this program commanded\"",
     "    if True:\n        return True, action, f\"aircraft is in {name}, which this program commanded\"",
     "pilot_takeover or px4_failsafe_return or test_teardown_plan"),
    ("G6 never-armed run still sends land", RF,
     "    if not armed_by_program:\n        print(\"[mavsdk] teardown: this program never armed the vehicle -- \"\n              \"sending nothing\")\n        return",
     "    if False:\n        return",
     "never_armed_sends_nothing"),
    ("G1 main() lets --sitl-smoke use a serial link", RF,
     "    if args.sitl_smoke and is_serial_url(args.mavsdk_url):\n        ap.error(",
     "    if False and is_serial_url(args.mavsdk_url):\n        ap.error(",
     "sitl_smoke_refuses_a_serial_link"),
    ("G1 driver has no serial defence", RF,
     "    if is_serial_url(args.mavsdk_url) and not fly:\n        print(f\"[mavsdk] REFUSED:",
     "    if False:\n        print(f\"[mavsdk] REFUSED:",
     "driver_itself_refuses"),
    ("G1 --fly validates nothing", RF,
     "    why: List[str] = []\n    if not getattr(args, \"mavsdk_url\", None):\n        why.append(\"--fly needs --mavsdk-url (on the Pi: \"",
     "    why: List[str] = []\n    return why\n    if not getattr(args, \"mavsdk_url\", None):\n        why.append(\"--fly needs --mavsdk-url (on the Pi: \"",
     "fly_refuses_every_simulator_default"),
    ("G1 arming does not wait for the permit", RF,
     "            if permit_trigger is None or not await _wait_arm_permit(\n                    args, trigger, permit_trigger, state):\n                return 1",
     "            pass",
     "without_a_permit_never_arms or arms_only_after or someone_else_armed"),
    ("G1 permit switch found HIGH counts", RF,
     "        rose = (p == \"high\" and self._permit_prev_high is False)",
     "        rose = (p == \"high\")\n        self.permit_seen_low = True",
     "arm_permit"),
    ("G4 commanded speed above the board is not refused", RF,
     "        if v_cmd > float(v_plat) + 1e-6:\n            refusals.append(",
     "        if False:\n            refusals.append(",
     "limits_commanded_speed or stock_speed_limit"),
    ("G4 transit check uses the commanded speed (old behaviour)", RF,
     "        t_min = approach_transit_lower_bound_s(belief_range, v_eff,",
     "        t_min = approach_transit_lower_bound_s(belief_range, v_cmd,",
     "limits_transit_check"),
    ("G4 geofence is not checked", RF,
     "        if gf_h is not None and float(gf_h) > 0.0 and d_meet > float(gf_h):",
     "        if False:",
     "geofence"),
    ("G4 an unread limit can be overridden", RF,
     "            if hard or (refusals and not override):",
     "            if (hard or refusals) and not override:",
     "cannot_override_a_limit"),
    ("G11 bench does not require disarmed", RF,
     "        if state[\"armed\"] is not False:\n            print(f\"[bench] REFUSED: the vehicle reports armed=",
     "        if False:\n            print(f\"[bench] REFUSED: the vehicle reports armed=",
     "bench_refuses_unless"),
    ("G11 bench does not check the battery", RF,
     "        if state[\"batt_v\"] > max_batt:\n            print(f\"[bench] REFUSED: battery reads",
     "        if False:\n            print(f\"[bench] REFUSED: battery reads",
     "bench_refuses_unless"),
    ("G11 bench loop does not abort when armed", RF,
     "            if state[\"armed\"] is not False:\n                out[\"abort\"] = (f\"vehicle reports armed=",
     "            if False:\n                out[\"abort\"] = (f\"vehicle reports armed=",
     "bench_aborts_the_instant"),
    ("G11 bench path arms", RF,
     "        offboard_ok = await _enter_offboard(drone, cfg, state)\n        if not offboard_ok:\n            rc_exit = 1\n        else:\n            loop_out = await _control_loop(\n                drone, args, cfg, sm, trigger, detector, state, frame_recorder,\n                supervisor, outer_bound_s=args.smoke_duration,",
     "        await drone.action.arm()\n        offboard_ok = await _enter_offboard(drone, cfg, state)\n        if not offboard_ok:\n            rc_exit = 1\n        else:\n            loop_out = await _control_loop(\n                drone, args, cfg, sm, trigger, detector, state, frame_recorder,\n                supervisor, outer_bound_s=args.smoke_duration,",
     "bench_mode_cannot_reach or bench_rehearsal_runs_the_real_loop"),
    ("G2 the loop never starts the watchdog timing", RF,
     "    if supervisor is not None:\n        supervisor.watch()\n    t_prev_tick",
     "    t_prev_tick",
     "every_live_path_feeds or hung_control_loop"),
    ("G2 the loop does not feed the watchdog", RF,
     "        if supervisor is not None:\n            supervisor.beat()\n            if not supervisor.alive():",
     "        if supervisor is not None:\n            if not supervisor.alive():",
     "every_live_path_feeds"),
    ("G2 the loop flies on after its watchdog is gone", RF,
     "            if not supervisor.alive():\n                # The watchdog is gone",
     "            if False:\n                # The watchdog is gone",
     "flight_stops_when_its_watchdog_is_gone"),
    ("G2 supervisor ignores a heartbeat timeout", SUP,
     "        if not ready:\n            gap = now - last",
     "        if not ready:\n            last = now\n            continue\n            gap = now - last",
     "watchdog_ends_the_stream_when_the_loop_stops or stopped_program"),
    ("G2 supervisor ignores the program dying", SUP,
     "            what = kill_server(args.server_pid, args.match)\n            log(f\"TRIPPED: the flight program is gone",
     "            what = 'mutant: not killed'\n            log(f\"TRIPPED: the flight program is gone",
     "program_is_killed"),
    ("G2 supervisor kills without checking what the pid is", SUP,
     "    if match and match not in cmd:",
     "    if False:",
     "will_not_kill_a_process"),
    ("G2 --fly arms with no watchdog", RF,
     "        if not sup_ok:\n            print(\"[fly] REFUSED: the setpoint-stream watchdog could not be \"",
     "        if False:\n            print(\"[fly] REFUSED: the setpoint-stream watchdog could not be \"",
     "refuses_to_arm_without_the_watchdog"),
]


def run(k):
    p = subprocess.run([PY, "-m", "pytest", "-q", "-x", TESTS, "-k", k,
                        "-p", "no:cacheprovider"], cwd=WT, capture_output=True, text=True)
    tail = [ln for ln in p.stdout.strip().splitlines() if ln.strip()][-1:] or [""]
    return p.returncode, tail[0]


def main():
    originals = {RF: open(RF).read(), SUP: open(SUP).read()}
    bad = 0
    try:
        for name, path, old, new, k in MUTANTS:
            src = originals[path]
            if src.count(old) != 1:
                print(f"SETUP-ERROR  {name}: anchor found {src.count(old)} times")
                bad += 1
                continue
            open(path, "w").write(src.replace(old, new))
            try:
                rc, tail = run(k)
            finally:
                open(path, "w").write(src)
            ok = rc != 0 and ("failed" in tail or "error" in tail)
            print(f"{'KILLED ' if ok else 'SURVIVED'}  {name}  [{k}] -> {tail}")
            bad += 0 if ok else 1
    finally:
        for path, src in originals.items():
            open(path, "w").write(src)
    rc, tail = run("test_")
    print(f"restored source: {tail}")
    print(f"{len(MUTANTS) - bad}/{len(MUTANTS)} mutants killed")
    return 1 if bad or rc != 0 else 0


if __name__ == "__main__":
    sys.exit(main())
