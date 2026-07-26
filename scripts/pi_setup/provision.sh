#!/usr/bin/env bash
# ===========================================================================
# provision.sh — seeker-rig software bring-up for the physical interceptor's
# Raspberry Pi 5 (Raspberry Pi OS Bookworm, 64-bit). Runs ONCE per fresh flash,
# then again any time to re-assert state (idempotent). This is the "software is
# ready before the Pi arrives" half of the Tier-1 seeker build (build_tab
# subsystem `seeker`, steps skr-01..skr-08; docs/tripod_test_protocol.md §1).
#
# WHAT IT DOES (each step is a no-op when already applied):
#   1. apt deps          picamera2 / libcamera / rpicam-apps / opencv / numpy +
#                        the venv/build/ssh tooling.
#   2. venv              --system-site-packages venv + requirements-pi.txt
#                        (mavsdk, onnxruntime, pyapriltags).
#   3. config.txt        MARKER-GUARDED managed block: dtoverlay=ov9281,
#                        camera_auto_detect=0, dtparam=uart0=on (+enable_uart=1)
#                        (docs/camera_paper_check.md §5; hardware_order_list.md
#                        §0 FC<->Pi UART; project_state.json build_tab brain).
#   4. serial console    strip the Linux serial console from cmdline.txt +
#                        disable the serial-getty login. ON A PI 5 THAT CONSOLE
#                        LIVES ON THE 3-PIN DEBUG HEADER (serial0 -> ttyAMA10),
#                        not on GPIO14/15 — so this REMOVES the debug-header
#                        login shell (raspi-config `do_serial_cons 1` parity).
#                        The GPIO UART is freed by step 3, not by this step.
#   5. ssh / mDNS        enable ssh + avahi (headless field access), optional
#                        hostname set.
#
# SAFE TO RUN ON THIS DEV BOX: `--dry-run` prints the full plan and TOUCHES
# NOTHING (no apt, no venv, no file writes) — that is the self-test path
# (scripts/pi_setup/selftest.sh) and it works on WSL/x86 with no Pi.
#
# Every config.txt edit is a single marker-guarded block, so it is idempotent
# (regenerated each run) and reversible (`--revert` strips it; originals are
# also backed up to *.interceptor-sim.bak on first edit).
#
# Usage:
#   sudo bash scripts/pi_setup/provision.sh            # apply (on the Pi, as root)
#   bash scripts/pi_setup/provision.sh --dry-run       # print the plan, change nothing
#   sudo bash scripts/pi_setup/provision.sh --revert   # undo the managed edits
#   Flags: --venv DIR  --hostname NAME  --cam-port 0|1  --allow-non-pi
#          --config-only  --help
#          --cam-port picks the Pi 5 CSI connector the camera ribbon is in
#          (CAM/DISP1 = default; 0 emits `dtoverlay=ov9281,cam0`).
#          --config-only does just the config.txt/cmdline.txt edits (the
#          selftest's sandbox path; no apt, no venv, no systemd).
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# --- defaults / config -----------------------------------------------------
DRY=0
REVERT=0
ALLOW_NON_PI=0
# --config-only: run JUST the config.txt/cmdline.txt edits (no apt, no venv, no
# systemd). It exists so selftest.sh can exercise the managed-block code for
# REAL against a sandbox BOOT dir — the marker/idempotence/[all]/revert bugs all
# lived in a path that only --dry-run's PRINTED PLAN had ever been checked
# against, which is this project's named defect class (a green check over code
# nothing runs). Root is still required when the target is a real /boot.
CONFIG_ONLY=0
HOSTNAME_NEW=""
VENV="${SEEKER_VENV:-$REPO_ROOT/.venv-pi}"
REQ="$SCRIPT_DIR/requirements-pi.txt"

# Boot config dir: Bookworm uses /boot/firmware; older images use /boot.
# PROVISION_BOOT_DIR overrides it (the self-test points it at a sandbox).
if [ -n "${PROVISION_BOOT_DIR:-}" ]; then
  BOOT_DIR="$PROVISION_BOOT_DIR"
elif [ -d /boot/firmware ]; then
  BOOT_DIR="/boot/firmware"
else
  BOOT_DIR="/boot"
fi
CONFIG="$BOOT_DIR/config.txt"
CMDLINE="$BOOT_DIR/cmdline.txt"

# Marker-guard for the config.txt managed block.
BEGIN="# >>> interceptor-sim seeker provision (managed) >>>"
END="# <<< interceptor-sim seeker provision (managed) <<<"

# apt packages: camera stack + venv/build tooling + ssh/mDNS.
APT_PKGS=(
  # --- camera stack (docs/camera_paper_check.md §5: mainline libcamera, no vendor driver) ---
  python3-picamera2   # OV9281 libcamera binding used by pi_capture.py --source picamera2
  python3-libcamera   # libcamera core (pulled by picamera2; listed for clarity)
  rpicam-apps         # rpicam-still/rpicam-hello — the libcamera stills verify in the README
  python3-opencv      # cv2 for pi_capture.py (imwrite / cvtColor / VideoCapture)
  python3-numpy       # the numpy libcamera+picamera2 were built against (do NOT pip-shadow it)
  # --- venv + source-build fallback (some aarch64 wheels build from source) ---
  python3-venv
  python3-pip
  python3-dev
  build-essential
  cmake
  git
  # --- headless field access ---
  openssh-server
  avahi-daemon        # <hostname>.local discovery over the phone-hotspot LAN
)

# Which CSI connector the camera ribbon is in. The Pi 5 has TWO identical
# 22-pin CSI/DSI connectors, silkscreened CAM/DISP0 and CAM/DISP1 (between the
# micro-HDMI connectors and the Ethernet port). `dtoverlay=ov9281` binds camera
# connector 1 by DEFAULT, so a ribbon in CAM/DISP0 enumerates NOTHING and
# `rpicam-hello --list-cameras` prints "No cameras available!" with no hint why
# (raspberrypi.com/documentation/computers/camera_software.html). --cam-port 0
# emits `dtoverlay=ov9281,cam0` so the recovery is a re-provision, not a
# hand-edit of /boot/firmware/config.txt.
CAM_PORT=1

# config.txt managed-block body (comment provenance inline, per house style).
# Built AFTER arg parse so --cam-port can select the connector.
build_config_block() {
CONFIG_BLOCK=(
  # [all] FIRST, INSIDE the markers (2026-07-26). The block is APPENDED at EOF,
  #   and config.txt is section-filtered: if the file happens to end inside a
  #   model conditional ([pi4], [cm5], ...) every managed line below would be
  #   silently filtered out on this Pi 5 while the file still LOOKS provisioned.
  #   Leading [all] cancels any inherited filter wherever the block lands; it is
  #   inside the markers so --revert takes it away again with everything else.
  "[all]"
  "# innomaker OV9281 mono global-shutter -- native mainline overlay, no vendor"
  "#   driver to build (docs/camera_paper_check.md item 5). Binds CSI connector"
  "#   CAM/DISP${CAM_PORT} (see --cam-port; bare = connector 1, the overlay default)."
  "$([ "$CAM_PORT" = "0" ] && echo "dtoverlay=ov9281,cam0" || echo "dtoverlay=ov9281")"
  "# Pi 5 / Bookworm: turn OFF the auto camera probe so the ov9281 overlay is"
  "#   honoured (docs/camera_paper_check.md item 5)."
  "camera_auto_detect=0"
  "# Brings up the GPIO14/15 UART (RP1 uart0) for the TELEM2 MAVLink companion"
  "#   link to the FC: 3.3 V, 921600 baud, TX/RX crossed (docs/hardware_order_list.md"
  "#   FC<->Pi UART; docs/project_state.json build_tab brain, MAV_1_CONFIG=TELEM2)."
  "#   3-wire link, CTS floating -- so do NOT add the ctsrts param (bench.params"
  "#   sets MAV_1_FLOW_CTRL=0)."
  "#   SET EXPLICITLY, not because enable_uart=1 is known to be insufficient:"
  "#   MEASURED 2026-07-26 on the project's own Pi 5 Model B Rev 1.1 running"
  "#   Debian 13 trixie / kernel 6.18.34+rpt-rpi-2712, enable_uart=1 ALONE was"
  "#   enough -- after reboot 'pinctrl get 14,15' read a4 TXD0/RXD0, /dev/ttyAMA0"
  "#   existed and opened at 921600, and /dev/serial0 pointed at it. An earlier"
  "#   review asserted ttyAMA0 'will never appear' without this line; that claim is"
  "#   REFUTED for this firmware. It is kept because it is explicit and correct on"
  "#   firmware revisions where enable_uart=1 does NOT suffice, and is a no-op where"
  "#   it already does -- belt and braces, not the load-bearing line."
  "dtparam=uart0=on"
  "# The correct knob on pre-Pi-5 boards. On a Pi 5 the PRIMARY/console UART is the"
  "#   debug-header ttyAMA10, so this is NOT what nominally targets GPIO14/15 --"
  "#   but see the measurement above: on 6.18.34 it brought ttyAMA0 up regardless."
  "enable_uart=1"
)
}

# --- tiny UI helpers -------------------------------------------------------
banner() { printf '\n== %s ==\n' "$*"; }
step()   { printf '\n[provision] %s\n' "$*"; }
warn()   { printf '[provision] WARN: %s\n' "$*" >&2; }
die()    { printf '[provision] ERROR: %s\n' "$*" >&2; exit 1; }

require_root() {
  [ "$(id -u)" -eq 0 ] || die "apply mode writes /boot + /etc and installs apt packages — run with sudo (or use --dry-run)."
}

guard_pi() {
  # Only enforced in apply mode; --dry-run and --allow-non-pi bypass it.
  local model=""
  [ -r /proc/device-tree/model ] && model="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)"
  case "$model" in
    *"Raspberry Pi"*) return 0 ;;
    *) die "this does not look like a Raspberry Pi (model='${model:-unknown}'). Refusing to edit ${BOOT_DIR} on the wrong machine. Use --dry-run to preview, or --allow-non-pi to force." ;;
  esac
}

usage() {
  sed -n '2,44p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit 0
}

# --- arg parse -------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run)      DRY=1 ;;
    --revert)       REVERT=1 ;;
    --allow-non-pi) ALLOW_NON_PI=1 ;;
    --venv)         VENV="${2:?--venv needs a path}"; shift ;;
    --venv=*)       VENV="${1#*=}" ;;
    --hostname)     HOSTNAME_NEW="${2:?--hostname needs a name}"; shift ;;
    --hostname=*)   HOSTNAME_NEW="${1#*=}" ;;
    --cam-port)     CAM_PORT="${2:?--cam-port needs 0 or 1}"; shift ;;
    --cam-port=*)   CAM_PORT="${1#*=}" ;;
    --config-only)  CONFIG_ONLY=1 ;;
    -h|--help)      usage ;;
    *)              die "unknown argument: $1 (see --help)" ;;
  esac
  shift
done
case "$CAM_PORT" in
  0|1) ;;
  *)   die "--cam-port must be 0 or 1 (the Pi 5's CAM/DISP0 / CAM/DISP1 connectors), got '$CAM_PORT'" ;;
esac
build_config_block

# --- managed-block plumbing -----------------------------------------------
render_block() {
  printf '%s\n' "$BEGIN"
  printf '%s\n' "${CONFIG_BLOCK[@]}"
  printf '%s\n' "$END"
}

# Detection and removal MUST use the SAME predicate, and it must tolerate the
# ways a marker line gets mutated by hand (trailing whitespace; a CRLF, because
# /boot/firmware is FAT32 and this SD card is routinely editable from Windows).
# They used to disagree — grep did a SUBSTRING match while awk did an exact-line
# match — so a padded marker made every apply APPEND A SECOND BLOCK and made
# --revert print "removed managed block" while removing nothing.
_AWK_NORM='{ L=$0; sub(/\r$/,"",L); sub(/[ \t]+$/,"",L) }'

has_managed_block() {   # $1=file — exit 0 when a normalized BEGIN marker exists
  awk -v b="$BEGIN" "$_AWK_NORM"'
    L==b {found=1}
    END { exit found?0:1 }
  ' "$1"
}

strip_managed_block() {  # $1=file -> stdout. exit 3 = BEGIN with no END.
  awk -v b="$BEGIN" -v e="$END" "$_AWK_NORM"'
    L==b {skip=1; nb++; next}
    L==e {skip=0; ne++; next}
    skip!=1 {print}
    # FAIL CLOSED on an unterminated block: with END mutated, the old awk
    # skipped to EOF and silently DELETED every user line after the marker.
    END { if (nb>0 && ne==0) exit 3 }
  ' "$1"
}

# Replace $CONFIG with the contents of $1, as durably as a FAT32 boot partition
# allows: sibling temp on the SAME filesystem (a /tmp temp is a different fs, so
# `mv` would silently degrade to copy+unlink = the truncate-then-write we are
# trying to avoid), data flushed BEFORE the rename, then the directory entry.
# NOT claimed to be crash-ATOMIC — vfat gives no such guarantee — but the crash
# window shrinks from "the whole rewrite" to "the rename", and the new bytes are
# already on the card when the rename happens.
replace_config() {  # $1 = file holding the new contents
  local new="$1" dst="$CONFIG.tmp.$$"
  cat "$new" > "$dst"
  chmod --reference="$CONFIG" "$dst" 2>/dev/null || true   # mv would take the umask
  sync "$dst" 2>/dev/null || sync
  mv -f "$dst" "$CONFIG"
  sync "$BOOT_DIR" 2>/dev/null || sync
}

apply_config_block() {
  step "camera + UART overlay -> $CONFIG  (marker-guarded managed block; idempotent)"
  echo "    camera CSI connector: CAM/DISP$CAM_PORT   (--cam-port 0|1)"
  printf '    ---8<--- managed block ---\n'
  render_block | sed 's/^/    /'
  printf '    --------------------------\n'
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: no file written)"; return 0; fi
  if [ ! -f "$CONFIG" ]; then
    # Create path: nothing to lose, and a temp+rename here would be WORSE — an
    # interruption before the rename leaves NO config.txt at all.
    warn "$CONFIG not present — creating it with the managed block only"
    mkdir -p "$(dirname "$CONFIG")"
    render_block > "$CONFIG"
    return 0
  fi
  [ -f "$CONFIG.interceptor-sim.bak" ] || cp -a "$CONFIG" "$CONFIG.interceptor-sim.bak"
  local tmp; tmp="$(mktemp)"
  # shellcheck disable=SC2064
  trap "rm -f '$tmp'" RETURN
  if has_managed_block "$CONFIG"; then
    # regenerate the block: drop the old one, then re-append at EOF (the leading
    # [all] keeps it filter-safe wherever it lands)
    strip_managed_block "$CONFIG" > "$tmp" || die \
      "$CONFIG has a '$BEGIN' marker with NO matching end marker — refusing to \
edit it, because stripping an unterminated block would delete everything after \
the marker. Repair it by hand (or restore $CONFIG.interceptor-sim.bak) and re-run."
  else
    cat "$CONFIG" > "$tmp"
    printf '\n' >> "$tmp"
  fi
  render_block >> "$tmp"
  replace_config "$tmp"
  # POST-CONDITION, not a hope: exactly one managed block landed, and the file
  # is not the truncated/empty shape a mid-write interruption leaves behind.
  local nb; nb="$(grep -cF -- "$BEGIN" "$CONFIG" || true)"
  if [ ! -s "$CONFIG" ] || [ "$nb" != "1" ]; then
    cp -a "$CONFIG.interceptor-sim.bak" "$CONFIG"
    die "$CONFIG came back wrong after the write ($nb begin-marker(s), $(wc -c <"$CONFIG") bytes) — restored from $CONFIG.interceptor-sim.bak. Nothing was changed."
  fi
}

apply_cmdline() {
  step "strip the Linux serial console from $CMDLINE  (raspi-config do_serial_cons 1)"
  echo "    strip any 'console=serial0,*' / 'console=ttyAMA0,*' token"
  echo "    ON A PI 5 THIS REMOVES THE DEBUG-HEADER CONSOLE: serial0 -> ttyAMA10 is"
  echo "     the 3-pin debug connector, so you lose the boot log AND the login shell"
  echo "     there. The GPIO14/15 UART (ttyAMA0) never had a console on a Pi 5 — it"
  echo "     is freed by 'dtparam=uart0=on' above, not by this strip."
  echo "    KEPT: the HDMI+keyboard console (tty1) and SSH/avahi. To get the debug"
  echo "     console back: sudo raspi-config nonint do_serial_cons 0   (or restore"
  echo "     $CMDLINE.interceptor-sim.bak), then reboot."
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: no file written)"; return 0; fi
  if [ ! -f "$CMDLINE" ]; then warn "$CMDLINE absent — skipping cmdline edit"; return 0; fi
  [ -f "$CMDLINE.interceptor-sim.bak" ] || cp -a "$CMDLINE" "$CMDLINE.interceptor-sim.bak"
  sed -i -E 's/console=(serial0|ttyAMA0),[0-9]+ ?//g' "$CMDLINE"
}

disable_serial_getty() {
  step "disable the serial-getty login on ttyAMA0  (companion link, not a console)"
  echo "    systemctl disable --now serial-getty@ttyAMA0.service"
  echo "    systemctl mask serial-getty@ttyAMA0.service"
  echo "    (belt-and-braces: on a Pi 5 this unit does not run anyway — the console"
  echo "     getty is generated on serial0 -> ttyAMA10 from the cmdline console="
  echo "     token, which the step above removes. Kept for pre-Pi-5 boards / a"
  echo "     future image that puts a getty on the GPIO UART. NOT extended to"
  echo "     ttyAMA10: raspi-config masks nothing, and one way to kill the debug"
  echo "     console is enough.)"
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: systemd not touched)"; return 0; fi
  systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
  systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true
}

enable_ssh() {
  step "SSH + mDNS niceties  (headless field access over a phone hotspot)"
  echo "    systemctl enable --now ssh"
  echo "    systemctl enable --now avahi-daemon        # <hostname>.local discovery"
  if [ -n "$HOSTNAME_NEW" ]; then
    echo "    hostnamectl set-hostname $HOSTNAME_NEW      (+ /etc/hosts 127.0.1.1)"
  else
    echo "    (hostname unchanged — pass --hostname NAME to set it)"
  fi
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: systemd/hostname not touched)"; return 0; fi
  systemctl enable --now ssh 2>/dev/null || true
  systemctl enable --now avahi-daemon 2>/dev/null || true
  if [ -n "$HOSTNAME_NEW" ]; then
    hostnamectl set-hostname "$HOSTNAME_NEW"
    if [ -f /etc/hosts ] && grep -qE '^127\.0\.1\.1' /etc/hosts; then
      sed -i -E "s/^127\.0\.1\.1.*/127.0.1.1\t$HOSTNAME_NEW/" /etc/hosts
    else
      printf '127.0.1.1\t%s\n' "$HOSTNAME_NEW" >> /etc/hosts
    fi
  fi
}

install_apt() {
  step "APT dependencies  (idempotent — apt-get install is a no-op when current)"
  echo "    apt-get update"
  echo "    apt-get install -y ${APT_PKGS[*]}"
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: apt not touched)"; return 0; fi
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y "${APT_PKGS[@]}"
}

build_venv() {
  step "Pi-side venv -> $VENV  (--system-site-packages: sees apt picamera2/libcamera/opencv/numpy)"
  echo "    python3 -m venv --system-site-packages $VENV"
  echo "    $VENV/bin/pip install --upgrade pip"
  echo "    $VENV/bin/pip install -r $REQ"
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: venv not created)"; return 0; fi
  [ -f "$REQ" ] || die "requirements file missing: $REQ"
  [ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV"
  "$VENV/bin/pip" install --upgrade pip
  "$VENV/bin/pip" install -r "$REQ"
}

do_revert() {
  banner "REVERT — undo the managed provisioning"
  step "strip the managed block from $CONFIG (originals kept at *.interceptor-sim.bak)"
  step "restore $CMDLINE from backup; unmask serial-getty@ttyAMA0"
  echo "    (apt packages + the venv are left in place — nothing to break by keeping them)"
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: nothing changed)"; return 0; fi
  case "$CONFIG_ONLY:$BOOT_DIR" in
    1:/boot*) require_root ;;
    1:*)      : ;;                # sandbox revert (selftest.sh) needs no root
    *)        require_root ;;
  esac
  if [ -f "$CONFIG" ] && has_managed_block "$CONFIG"; then
    local tmp; tmp="$(mktemp)"
    # shellcheck disable=SC2064
    trap "rm -f '$tmp'" RETURN
    strip_managed_block "$CONFIG" > "$tmp" || die \
      "$CONFIG has a '$BEGIN' marker with NO matching end marker — refusing to \
strip it, because that would delete everything after the marker. Repair it by \
hand (or restore $CONFIG.interceptor-sim.bak) and re-run --revert."
    replace_config "$tmp"
    # A revert that removed NOTHING is UNCERTAIN, never success (no vacuous
    # verdicts): the old code printed "removed managed block" unconditionally.
    if grep -qF -- "$BEGIN" "$CONFIG" || grep -qF -- "$END" "$CONFIG"; then
      die "revert FAILED: marker lines are STILL in $CONFIG — the managed settings were NOT removed. Inspect it by hand (backup: $CONFIG.interceptor-sim.bak)."
    fi
    echo "    removed managed block from $CONFIG"
  fi
  if [ -f "$CMDLINE.interceptor-sim.bak" ]; then
    cp -a "$CMDLINE.interceptor-sim.bak" "$CMDLINE"
    echo "    restored $CMDLINE from backup"
  fi
  systemctl unmask serial-getty@ttyAMA0.service 2>/dev/null || true
  echo "    reboot to complete the revert."
}

# --- main ------------------------------------------------------------------
MODE="APPLY"; [ "$DRY" -eq 1 ] && MODE="DRY-RUN (no changes)"
banner "interceptor-sim seeker Pi provisioning  [$MODE]"
cat <<EOF
  repo root : $REPO_ROOT
  boot dir  : $BOOT_DIR
  config    : $CONFIG
  cmdline   : $CMDLINE
  venv      : $VENV
  reqs      : $REQ
  hostname  : ${HOSTNAME_NEW:-<unchanged>}
EOF

if [ "$REVERT" -eq 1 ]; then
  do_revert
  banner "REVERT done"
  exit 0
fi

if [ "$DRY" -eq 0 ]; then
  # A sandbox BOOT_DIR (--config-only + PROVISION_BOOT_DIR, i.e. selftest.sh)
  # writes nothing privileged, so it needs no root; a real /boot always does.
  case "$CONFIG_ONLY:$BOOT_DIR" in
    1:/boot*) require_root; [ "$ALLOW_NON_PI" -eq 1 ] || guard_pi ;;
    1:*)      : ;;
    *)        require_root; [ "$ALLOW_NON_PI" -eq 1 ] || guard_pi ;;
  esac
fi

if [ "$CONFIG_ONLY" -eq 0 ]; then
  install_apt
  build_venv
fi
apply_config_block
apply_cmdline
if [ "$CONFIG_ONLY" -eq 0 ]; then
  disable_serial_getty
  enable_ssh
fi

banner "plan complete  [$MODE]"
cat <<EOF
  Next (after applying on the Pi, then a REBOOT — config.txt/cmdline changes need one):
    0. OBSERVE THE EFFECT, don't assume it (both are one-liners):
         ls -l /dev/ttyAMA*     # MUST now list ttyAMA0 (GPIO14/15) as well as
                                #   ttyAMA10 (debug header). No ttyAMA0 => the
                                #   dtparam=uart0=on line did not take.
         ls -l /dev/serial0     # READ where it points — on a Pi 5 it is NOT
                                #   ttyAMA0. Always give MAVLink /dev/ttyAMA0
                                #   explicitly, never /dev/serial0.
    1. verify the camera:   rpicam-hello --list-cameras   then
                            rpicam-still -t 1000 -o /tmp/ov9281.jpg   (skr-02)
       "No cameras available!" => the ribbon is in the OTHER CSI port. Camera is
       in CAM/DISP$CAM_PORT per this run; re-run with --cam-port $((1-CAM_PORT)) and reboot.
    2. smoke pi_capture:     $VENV/bin/python scripts/seeker/pi_capture.py --self-test
    3. live sensor pass:     $VENV/bin/python scripts/seeker/pi_capture.py \\
                               --source picamera2 --out sessions/smoke --n-frames 30 --exposure-us 1000
       then confirm meta.json: exposure_meets_spec=true, applied_exposure_us <= 1000   (skr-03/skr-05)
       (that check is valid ONLY for a --source picamera2 session — a v4l2 session
        reports exposure in DEVICE units and publishes exposure_meets_spec=null)
    If config.txt ever looks wrong:
         sudo cp -a $CONFIG.interceptor-sim.bak $CONFIG && sudo reboot
    Full first-boot walkthrough + the skr-01..skr-08 bench items: scripts/pi_setup/README.md
EOF
[ "$DRY" -eq 1 ] && echo "  (DRY-RUN: nothing above was executed.)"
exit 0
