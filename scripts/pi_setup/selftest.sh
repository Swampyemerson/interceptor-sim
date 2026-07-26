#!/usr/bin/env bash
# ===========================================================================
# selftest.sh — offline gate for the Pi provisioning pack. Exits 0/1, NO
# hardware, safe on WSL/x86. Two checks, per the deliverable spec:
#   1. `bash -n provision.sh`  — the script parses.
#   2. `provision.sh --dry-run` — asserts it MAKES NO CHANGES and PRINTS THE
#      FULL PLAN. "No changes" is proven by pointing PROVISION_BOOT_DIR at a
#      sandbox with a known config.txt/cmdline.txt and confirming both files are
#      byte-identical afterwards, and that the target venv dir was never created.
# Also validates the sibling deliverables (requirements-pi.txt, capture.service).
#
# Run:  bash scripts/pi_setup/selftest.sh
# ===========================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROV="$SCRIPT_DIR/provision.sh"
REQ="$SCRIPT_DIR/requirements-pi.txt"
SVC="$SCRIPT_DIR/capture.service"

fails=0
ok()   { printf '[selftest] OK   %s\n' "$*"; }
bad()  { printf '[selftest] FAIL %s\n' "$*"; fails=$((fails+1)); }

# --- 1. syntax ------------------------------------------------------------
if bash -n "$PROV"; then ok "bash -n provision.sh"; else bad "bash -n provision.sh"; fi

# --- 2. dry-run makes no changes + prints the full plan -------------------
SANDBOX="$(mktemp -d)"
trap 'rm -rf "$SANDBOX"' EXIT
BOOT="$SANDBOX/boot"; mkdir -p "$BOOT"
printf 'dtparam=audio=on\nconsole=serial0,115200\n' > "$BOOT/config.txt"
printf 'console=serial0,115200 root=PARTUUID=abcd rootwait\n'   > "$BOOT/cmdline.txt"
CFG_BEFORE="$(md5sum "$BOOT/config.txt"  | cut -d' ' -f1)"
CMD_BEFORE="$(md5sum "$BOOT/cmdline.txt" | cut -d' ' -f1)"
VENV_TARGET="$SANDBOX/should-not-exist-venv"

set +e
OUT="$(PROVISION_BOOT_DIR="$BOOT" bash "$PROV" --dry-run --venv "$VENV_TARGET" --hostname interceptor-seeker 2>&1)"
RC=$?
set -e

[ "$RC" -eq 0 ] && ok "dry-run exited 0" || bad "dry-run exit code $RC"

# 2a. no file mutations
CFG_AFTER="$(md5sum "$BOOT/config.txt"  | cut -d' ' -f1)"
CMD_AFTER="$(md5sum "$BOOT/cmdline.txt" | cut -d' ' -f1)"
[ "$CFG_BEFORE" = "$CFG_AFTER" ] && ok "config.txt unchanged by dry-run"  || bad "config.txt was modified by dry-run"
[ "$CMD_BEFORE" = "$CMD_AFTER" ] && ok "cmdline.txt unchanged by dry-run" || bad "cmdline.txt was modified by dry-run"
[ ! -e "$VENV_TARGET" ]          && ok "venv NOT created by dry-run"      || bad "dry-run created the venv"
# no stray backups / files anywhere in the sandbox beyond the 2 seeds we wrote
STRAY="$(find "$SANDBOX" -type f ! -path "$BOOT/config.txt" ! -path "$BOOT/cmdline.txt" | wc -l)"
[ "$STRAY" -eq 0 ] && ok "dry-run wrote no files (no backups, no venv)" || bad "dry-run left $STRAY stray file(s)"

# 2b. the plan is FULL — assert every managed action is printed
assert_out() { if grep -qF -- "$2" <<<"$OUT"; then ok "plan mentions: $1"; else bad "plan missing: $1"; fi; }
assert_out "DRY-RUN banner"        "DRY-RUN (no changes)"
assert_out "dtoverlay=ov9281"      "dtoverlay=ov9281"
assert_out "camera_auto_detect=0"  "camera_auto_detect=0"
# THE line that creates /dev/ttyAMA0 on a Pi 5. enable_uart=1 alone is a no-op
# for GPIO14/15 there (it targets the debug-header ttyAMA10), and this selftest
# used to certify the dead link by asserting only that string.
assert_out "dtparam=uart0=on (Pi 5 GPIO14/15 UART)" "dtparam=uart0=on"
assert_out "enable_uart=1 (pre-Pi-5 boards)" "enable_uart=1"
# [all] guard: the block is appended at EOF, so it must cancel any model filter
# the file happens to end inside, or every managed line is silently ignored.
assert_out "[all] filter guard"    "[all]"
assert_out "serial console"        "serial console"
# The plan used to claim the cmdline strip "leaves ... the Pi 5 debug ttyAMA10
# console untouched" — false: on a Pi 5 serial0 IS ttyAMA10, so the strip removes
# exactly that console. Pin the corrected claim, not merely the string ttyAMA10.
assert_out "debug-header console honesty" "REMOVES THE DEBUG-HEADER CONSOLE"
assert_out "CSI connector named"   "CAM/DISP"
assert_out "serial-getty disabled" "serial-getty@ttyAMA0"
assert_out "apt install step"      "apt-get install -y"
assert_out "picamera2 apt dep"     "python3-picamera2"
assert_out "venv build step"       "python3 -m venv --system-site-packages"
assert_out "requirements-pi.txt"   "requirements-pi.txt"
assert_out "ssh enabled"           "systemctl enable --now ssh"
assert_out "hostname honored"      "interceptor-seeker"
assert_out "pi_capture smoke hint" "pi_capture.py --self-test"
assert_out "no-op tail line"       "nothing above was executed"

# 2c. --cam-port selects the CSI connector in the emitted overlay line
CAM0_OUT="$(PROVISION_BOOT_DIR="$BOOT" bash "$PROV" --dry-run --cam-port 0 2>&1 || true)"
grep -qF -- 'dtoverlay=ov9281,cam0' <<<"$CAM0_OUT" \
  && ok "--cam-port 0 emits dtoverlay=ov9281,cam0" || bad "--cam-port 0 did not emit the cam0 overlay"
grep -qxF -- '    dtoverlay=ov9281' <<<"$OUT" \
  && ok "default emits the bare dtoverlay=ov9281 (connector 1)" || bad "default overlay line changed"

# --- 2d. APPLY MODE, for real, against a sandbox boot dir -----------------
# The managed-block code (marker match, idempotence, the [all] guard, revert)
# used to be covered ONLY by grepping the PRINTED PLAN — a green check over a
# path nothing ran. --config-only makes it executable offline.
ABOX="$SANDBOX/apply"; mkdir -p "$ABOX"
seed_cfg() {   # $1 = trailing content, to prove the [all] guard
  printf 'dtparam=audio=on\n%s\n' "$1" > "$ABOX/config.txt"
  printf 'console=serial0,115200 root=PARTUUID=abcd rootwait\n' > "$ABOX/cmdline.txt"
}
apply_cfg() { PROVISION_BOOT_DIR="$ABOX" bash "$PROV" --config-only "$@" >/dev/null 2>&1; }
cnt() { grep -cF -- "$1" "$ABOX/config.txt" || true; }

# (i) a config.txt ending inside a MODEL FILTER: the block must land under [all]
seed_cfg '[cm5]'
apply_cfg && ok "apply --config-only exited 0" || bad "apply --config-only failed"
LAST_HDR="$(awk '/^\[/{h=$0} /^dtoverlay=ov9281/{print h; exit}' "$ABOX/config.txt")"
[ "$LAST_HDR" = "[all]" ] \
  && ok "managed block sits under [all] even when config.txt ends in [cm5]" \
  || bad "managed block inherited the '$LAST_HDR' filter — it would be IGNORED on the Pi 5"
[ "$(cnt 'dtparam=uart0=on')" -eq 1 ] && ok "apply wrote dtparam=uart0=on (the Pi 5 GPIO UART enabler)" \
  || bad "apply did not write dtparam=uart0=on"

# (ii) idempotent: three applies == ONE block
apply_cfg && apply_cfg
[ "$(cnt "$(grep -m1 -F '>>> interceptor-sim' "$ABOX/config.txt")")" -eq 1 ] \
  && ok "3 applies leave exactly 1 managed block" || bad "applies duplicated the managed block"

# (iii) a HAND-MUTATED marker (trailing space / CRLF) must NOT duplicate the
#       block: detection and removal now share one normalized predicate.
sed -i 's/^\(# >>> interceptor-sim.*>>>\)$/\1 /' "$ABOX/config.txt"
apply_cfg
[ "$(cnt 'dtoverlay=ov9281')" -eq 1 ] \
  && ok "a padded BEGIN marker does not append a SECOND block" \
  || bad "padded marker duplicated the block ($(cnt 'dtoverlay=ov9281') overlay lines)"

# (iv) an UNTERMINATED block must FAIL CLOSED, not delete to EOF
seed_cfg '[all]'; apply_cfg
printf 'user_setting_after_block=1\n' >> "$ABOX/config.txt"
sed -i 's/^# <<< interceptor-sim.*$/# <<< MANGLED/' "$ABOX/config.txt"
BEFORE_MANGLED="$(md5sum "$ABOX/config.txt" | cut -d' ' -f1)"
if apply_cfg; then bad "apply over an unterminated block exited 0 (should refuse)"
else ok "apply over an unterminated block REFUSES (non-zero)"; fi
[ "$(md5sum "$ABOX/config.txt" | cut -d' ' -f1)" = "$BEFORE_MANGLED" ] \
  && ok "the refused apply left config.txt byte-identical (no delete-to-EOF)" \
  || bad "the refused apply MODIFIED config.txt"
if PROVISION_BOOT_DIR="$ABOX" bash "$PROV" --config-only --revert >/dev/null 2>&1; then
  bad "--revert over an unterminated block exited 0 (should refuse)"
else ok "--revert over an unterminated block REFUSES (non-zero)"; fi

# (v) revert round-trips a normally-applied file back to its pre-provision bytes
seed_cfg '[all]'
CLEAN="$(grep -v '^$' "$ABOX/config.txt" | md5sum | cut -d' ' -f1)"
apply_cfg
PROVISION_BOOT_DIR="$ABOX" bash "$PROV" --config-only --revert >/dev/null 2>&1 \
  && ok "--revert exited 0" || bad "--revert failed"
[ "$(cnt 'dtoverlay=ov9281')" -eq 0 ] && [ "$(cnt '[all]')" -eq 1 ] \
  && ok "--revert removed the block INCLUDING its leading [all]" \
  || bad "--revert left managed lines behind"
# (the one blank separator line apply appends before the block survives revert;
#  every non-blank line must be back exactly as it was)
[ "$(grep -v '^$' "$ABOX/config.txt" | md5sum | cut -d' ' -f1)" = "$CLEAN" ] \
  && ok "--revert restored every non-blank config.txt line" || bad "--revert did not round-trip"

# --- 3. sibling deliverables sane ----------------------------------------
grep -qE '^mavsdk==3\.15\.3'      "$REQ" && ok "requirements pins mavsdk"      || bad "requirements missing mavsdk pin"
grep -qE '^onnxruntime==1\.27\.0' "$REQ" && ok "requirements pins onnxruntime" || bad "requirements missing onnxruntime pin"
grep -qE '^pyapriltags'           "$REQ" && ok "requirements has pyapriltags"  || bad "requirements missing pyapriltags"
! grep -qiE '^numpy|^opencv|^picamera2' "$REQ" && ok "requirements does NOT pip-pin numpy/opencv/picamera2 (apt)" \
                                               || bad "requirements wrongly pip-pins an apt package"
grep -qF 'pi_capture.py' "$SVC"          && ok "capture.service runs pi_capture.py" || bad "capture.service wrong ExecStart"
grep -qF 'source picamera2' "$SVC"       && ok "capture.service uses --source picamera2" || bad "capture.service wrong source"
# PIN THE CONSTANTS, not just their presence: the unit's EXTRA_ARGS example is
# copy-pasted into /etc/default/interceptor-capture, and it carried --tag-size
# 0.30 (14% short of the adopted 0.350 m placard, docs/placard_mount.md §2) long
# after the same error was purged from pi_capture/tripod_score.
grep -qF -- '--tag-size 0.35' "$SVC"     && ok "capture.service example pins the adopted --tag-size 0.35" || bad "capture.service --tag-size is not the adopted 0.35"
grep -qF -- '--quad-decimate 1.0' "$SVC" && ok "capture.service example pins --quad-decimate 1.0 (ADR-0082)" || bad "capture.service example omits --quad-decimate 1.0"
! grep -qE -- '--tag-size 0\.30?([^0-9]|$)' "$SVC" && ok "capture.service carries NO stale --tag-size 0.30" || bad "capture.service still mentions --tag-size 0.30"
# capture.service parses as an ini-ish unit (has the three required sections)
for sec in '[Unit]' '[Service]' '[Install]'; do
  grep -qF "$sec" "$SVC" && ok "capture.service has $sec" || bad "capture.service missing $sec"
done

# --- verdict --------------------------------------------------------------
echo
if [ "$fails" -eq 0 ]; then
  echo "[selftest] PASS — all checks green"
  exit 0
else
  echo "[selftest] FAIL — $fails check(s) failed"
  exit 1
fi
