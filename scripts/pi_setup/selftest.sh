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
assert_out "enable_uart=1 (TELEM2)" "enable_uart=1"
assert_out "serial console freed"  "serial console"
assert_out "serial-getty disabled" "serial-getty@ttyAMA0"
assert_out "apt install step"      "apt-get install -y"
assert_out "picamera2 apt dep"     "python3-picamera2"
assert_out "venv build step"       "python3 -m venv --system-site-packages"
assert_out "requirements-pi.txt"   "requirements-pi.txt"
assert_out "ssh enabled"           "systemctl enable --now ssh"
assert_out "hostname honored"      "interceptor-seeker"
assert_out "pi_capture smoke hint" "pi_capture.py --self-test"
assert_out "no-op tail line"       "nothing above was executed"

# --- 3. sibling deliverables sane ----------------------------------------
grep -qE '^mavsdk==3\.15\.3'      "$REQ" && ok "requirements pins mavsdk"      || bad "requirements missing mavsdk pin"
grep -qE '^onnxruntime==1\.27\.0' "$REQ" && ok "requirements pins onnxruntime" || bad "requirements missing onnxruntime pin"
grep -qE '^pyapriltags'           "$REQ" && ok "requirements has pyapriltags"  || bad "requirements missing pyapriltags"
! grep -qiE '^numpy|^opencv|^picamera2' "$REQ" && ok "requirements does NOT pip-pin numpy/opencv/picamera2 (apt)" \
                                               || bad "requirements wrongly pip-pins an apt package"
grep -qF 'pi_capture.py' "$SVC"          && ok "capture.service runs pi_capture.py" || bad "capture.service wrong ExecStart"
grep -qF 'source picamera2' "$SVC"       && ok "capture.service uses --source picamera2" || bad "capture.service wrong source"
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
