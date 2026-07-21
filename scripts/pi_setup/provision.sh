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
#                        camera_auto_detect=0, enable_uart=1
#                        (docs/camera_paper_check.md §5; hardware_order_list.md
#                        §0 FC<->Pi UART; project_state.json build_tab brain).
#   4. serial console    free ttyAMA0 for the TELEM2 MAVLink link (strip the
#                        GPIO serial console from cmdline.txt + disable the
#                        serial-getty login).
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
#   Flags: --venv DIR  --hostname NAME  --allow-non-pi  --help
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# --- defaults / config -----------------------------------------------------
DRY=0
REVERT=0
ALLOW_NON_PI=0
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

# config.txt managed-block body (comment provenance inline, per house style).
CONFIG_BLOCK=(
  "# innomaker OV9281 mono global-shutter -- native mainline overlay, no vendor"
  "#   driver to build (docs/camera_paper_check.md item 5)."
  "dtoverlay=ov9281"
  "# Pi 5 / Bookworm: turn OFF the auto camera probe so the ov9281 overlay is"
  "#   honoured (docs/camera_paper_check.md item 5)."
  "camera_auto_detect=0"
  "# Enable the PL011 UART on GPIO14/15 for the TELEM2 MAVLink companion link to"
  "#   the FC: 3.3 V, 921600 baud, TX/RX crossed (docs/hardware_order_list.md"
  "#   FC<->Pi UART; docs/project_state.json build_tab brain, MAV_1_CONFIG=TELEM2)."
  "enable_uart=1"
)

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
  sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
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
    -h|--help)      usage ;;
    *)              die "unknown argument: $1 (see --help)" ;;
  esac
  shift
done

# --- managed-block plumbing -----------------------------------------------
render_block() {
  printf '%s\n' "$BEGIN"
  printf '%s\n' "${CONFIG_BLOCK[@]}"
  printf '%s\n' "$END"
}

apply_config_block() {
  step "camera + UART overlay -> $CONFIG  (marker-guarded managed block; idempotent)"
  printf '    ---8<--- managed block ---\n'
  render_block | sed 's/^/    /'
  printf '    --------------------------\n'
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: no file written)"; return 0; fi
  if [ ! -f "$CONFIG" ]; then
    warn "$CONFIG not present — creating it with the managed block only"
    mkdir -p "$(dirname "$CONFIG")"
    render_block > "$CONFIG"
    return 0
  fi
  [ -f "$CONFIG.interceptor-sim.bak" ] || cp -a "$CONFIG" "$CONFIG.interceptor-sim.bak"
  local tmp; tmp="$(mktemp)"
  if grep -qF "$BEGIN" "$CONFIG"; then
    # regenerate the block in place (drop the old one, then re-append)
    awk -v b="$BEGIN" -v e="$END" '
      $0==b {skip=1; next}
      $0==e {skip=0; next}
      skip!=1 {print}
    ' "$CONFIG" > "$tmp"
  else
    cat "$CONFIG" > "$tmp"
    printf '\n' >> "$tmp"
  fi
  render_block >> "$tmp"
  cat "$tmp" > "$CONFIG"          # keep the original inode/owner
  rm -f "$tmp"
}

apply_cmdline() {
  step "free the GPIO serial console on $CMDLINE  (MAVLink owns ttyAMA0)"
  echo "    strip any 'console=serial0,*' / 'console=ttyAMA0,*' token"
  echo "    (leaves tty1 + the Pi 5 debug ttyAMA10 console untouched;"
  echo "     mirrors 'raspi-config nonint do_serial_cons 1')"
  if [ "$DRY" -eq 1 ]; then echo "    (dry-run: no file written)"; return 0; fi
  if [ ! -f "$CMDLINE" ]; then warn "$CMDLINE absent — skipping cmdline edit"; return 0; fi
  [ -f "$CMDLINE.interceptor-sim.bak" ] || cp -a "$CMDLINE" "$CMDLINE.interceptor-sim.bak"
  sed -i -E 's/console=(serial0|ttyAMA0),[0-9]+ ?//g' "$CMDLINE"
}

disable_serial_getty() {
  step "disable the serial-getty login on ttyAMA0  (companion link, not a console)"
  echo "    systemctl disable --now serial-getty@ttyAMA0.service"
  echo "    systemctl mask serial-getty@ttyAMA0.service"
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
  require_root
  if [ -f "$CONFIG" ] && grep -qF "$BEGIN" "$CONFIG"; then
    local tmp; tmp="$(mktemp)"
    awk -v b="$BEGIN" -v e="$END" '
      $0==b {skip=1; next}
      $0==e {skip=0; next}
      skip!=1 {print}
    ' "$CONFIG" > "$tmp"
    cat "$tmp" > "$CONFIG"; rm -f "$tmp"
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
  require_root
  [ "$ALLOW_NON_PI" -eq 1 ] || guard_pi
fi

install_apt
build_venv
apply_config_block
apply_cmdline
disable_serial_getty
enable_ssh

banner "plan complete  [$MODE]"
cat <<EOF
  Next (after applying on the Pi, then a REBOOT — config.txt/cmdline changes need one):
    1. verify the camera:   rpicam-still -t 1000 -o /tmp/ov9281.jpg   (skr-02)
    2. smoke pi_capture:     $VENV/bin/python scripts/seeker/pi_capture.py --self-test
    3. live sensor pass:     $VENV/bin/python scripts/seeker/pi_capture.py \\
                               --source picamera2 --out sessions/smoke --n-frames 30 --exposure-us 1000
       then confirm meta.json: exposure_meets_spec=true, applied_exposure_us <= 1000   (skr-03/skr-05)
    Full first-boot walkthrough + the skr-01..skr-08 bench items: scripts/pi_setup/README.md
EOF
[ "$DRY" -eq 1 ] && echo "  (DRY-RUN: nothing above was executed.)"
exit 0
