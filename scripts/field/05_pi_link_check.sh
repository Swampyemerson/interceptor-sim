#!/usr/bin/env bash
# ===========================================================================
# 05_pi_link_check.sh -- "can this laptop actually reach the Pi, on the network
# it will have AT THE FIELD?" -- the PRE-PACK networking dry run
# (runbook: docs/field_bringup.md section 3.0).
#
# WHY IT EXISTS: every capture is started by ssh'ing from the laptop to the Pi
# (01_camera_live_check.sh, and 04_pull_logs.sh pulls the session back the same
# way). At the field the only network is a phone hotspot, and three things have
# to be true that are NOT true out of the box:
#   1. the Pi has to JOIN that hotspot -- the Pi Imager writes ONE Wi-Fi SSID
#      (your home one). Add the hotspot as a second profile at the desk.
#   2. the laptop has to RESOLVE and REACH it -- `interceptor-seeker.local` is
#      mDNS, and mDNS does not resolve from NAT-mode WSL2. Use the IP.
#   3. ssh has to work NON-INTERACTIVELY -- every field script uses
#      BatchMode=yes (a password prompt inside a gate script would hang), so a
#      key must exist here AND be installed on the Pi.
# It also measures the LAPTOP<->PI CLOCK OFFSET, because the hotspot is the Pi's
# NTP source and the frame<->log time join that produces every range number
# depends on it (docs/tripod_test_protocol.md section 6.1).
#
# It changes NOTHING on either machine (one temp file under the Pi's sessions/
# dir, removed immediately) and needs no hardware beyond the Pi.
#
# Usage:
#   scripts/field/05_pi_link_check.sh                  # uses FIELD_PI_HOST
#   scripts/field/05_pi_link_check.sh --pi pi@192.168.43.17
#   scripts/field/05_pi_link_check.sh --require        # field day: must pass
#
# Exit: 0 PASS (the link is proven end to end)
#       1 FAIL (the Pi answers but something on it is wrong -> read the hints)
#       2 usage (missing ssh/rsync on THIS machine)
#       3 NOT CONNECTED (no Pi answered -- advisory until field day)
# ===========================================================================
set -uo pipefail          # NOT -e: this script reports, it must not abort early

FLD_TAG="05-pilink"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/common.sh"

PI_HOST="$FIELD_PI_HOST"
# Protocol section 6.1: one 2 m range bin is 0.22 s of flight at 9 m/s, and the
# frame<->log sync error must stay WELL under that. This is the clock bar.
CLOCK_BAR_S=0.22

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pi)      PI_HOST="${2:-$FIELD_PI_DEFAULT_HOST}"; shift 2 ;;
        --require) FLD_REQUIRE=1; shift ;;
        -h|--help) fld_usage_from_header "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) fld_bad "unknown arg: $1"; fld_finish "$FLD_USAGE" "run with -h for usage" ;;
    esac
done

LOG_DIR="$(fld_log_dir 05_pilink)"
LOG="$LOG_DIR/report.txt"
exec > >(tee "$LOG") 2>&1
fld_say "log -> $LOG"

PI_TRY="${PI_HOST:-$FIELD_PI_DEFAULT_HOST}"
FAILS=0
HINTS=()

# ------------------------------------------------------- 1. this laptop's half
fld_head "1. this machine (ssh client, key, rsync)"
for tool in ssh rsync; do
    if command -v "$tool" >/dev/null 2>&1; then
        fld_ok "$tool present"
    else
        fld_bad "$tool is NOT installed here -- $([[ $tool == rsync ]] && echo '04_pull_logs cannot fetch the session' || echo 'nothing can reach the Pi')"
        fld_finish "$FLD_USAGE" "sudo apt install $tool"
    fi
done

KEY=""
for k in "$HOME/.ssh/id_ed25519" "$HOME/.ssh/id_rsa" "$HOME/.ssh/id_ecdsa"; do
    [[ -f "$k" ]] && { KEY="$k"; break; }
done
if [[ -n "$KEY" ]]; then
    fld_ok "ssh key: $KEY"
else
    fld_miss "NO ssh key on this laptop (~/.ssh/id_ed25519 etc.)"
    fld_hint 'make one now (no passphrase, it lives on your own laptop):'
    fld_hint '  ssh-keygen -t ed25519 -C interceptor-field -N "" -f ~/.ssh/id_ed25519'
    fld_hint '  ssh-copy-id pi@<Pi IP>      # once, over ANY network the Pi is on'
    HINTS+=("generate + copy the ssh key (above) -- every field script uses BatchMode=yes and cannot type a password")
fi

# ---------------------------------------------------- 2. the address it will use
fld_head "2. the address: $PI_TRY"
HOSTPART="${PI_TRY#*@}"
if [[ "$HOSTPART" == *.local ]]; then
    fld_warn "'$HOSTPART' is an mDNS name"
    if fld_is_wsl; then
        fld_bad "mDNS (.local) does NOT resolve from NAT-mode WSL2 -- this address will fail at the field"
        HINTS+=("use the IP: --pi pi@<IP>  (the Pi prints it with 'hostname -I'; your phone's hotspot client list also shows it)")
        HINTS+=("or pin it once:  echo '<IP>  interceptor-seeker' | sudo tee -a /etc/hosts   (inside WSL)")
        HINTS+=("or export FIELD_PI_HOST=pi@<IP> in your shell profile so every 0*.sh picks it up")
        FAILS=$((FAILS + 1))
    fi
fi
if getent hosts "$HOSTPART" >/dev/null 2>&1; then
    fld_ok "resolves: $(getent hosts "$HOSTPART" | head -1)"
else
    fld_miss "'$HOSTPART' does not resolve from here"
fi

# --------------------------------------------------------- 3. the link itself
fld_head "3. non-interactive ssh (BatchMode -- what every field script uses)"
if ! fld_pi_reachable "$PI_TRY"; then
    fld_miss "no answer from $PI_TRY"
    fld_finish "$FLD_ABSENT" \
        "THIS IS THE FIELD-DAY SINGLE POINT OF FAILURE -- clear it at the desk, not in a field" \
        "1. put the FIELD hotspot on the Pi as a SECOND Wi-Fi profile (over your home link):" \
        "     sudo nmcli connection add type wifi ifname wlan0 con-name field-hotspot ssid '<SSID>' \\" \
        "         wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<PASSWORD>'" \
        "     sudo nmcli connection modify field-hotspot connection.autoconnect yes" \
        "2. turn the phone hotspot ON, reboot the Pi, read its IP ('hostname -I' or the phone's client list)" \
        "3. ssh-copy-id pi@<IP>   then re-run:  scripts/field/05_pi_link_check.sh --pi pi@<IP>" \
        "if the hotspot has AP/CLIENT ISOLATION the laptop can never reach the Pi -- turn it off, or drive the Pi from an ssh app on the PHONE (the hotspot host always reaches its clients)" \
        "${HINTS[@]}"
fi
fld_ok "ssh $PI_TRY works with no password prompt"

# ------------------------------------------- 4. what the field scripts need there
fld_head "4. the Pi's half (repo, venv, recorder, rsync, disk, clock)"
# ONE round trip, KEY=VALUE out, so the clock sample below is not polluted by
# extra ssh handshakes. Nothing here writes outside the repo's sessions/ dir.
REMOTE_PROBE='
printf "host=%s\n" "$(hostname)"
printf "ips=%s\n" "$(hostname -I 2>/dev/null | tr -s " ")"
printf "rsync=%s\n" "$(command -v rsync || echo MISSING)"
printf "repo=%s\n" "$([ -d '"$FIELD_PI_REPO"' ] && echo OK || echo MISSING)"
printf "venv=%s\n" "$([ -x '"$FIELD_PI_PYTHON"' ] && echo OK || echo MISSING)"
printf "capture=%s\n" "$([ -f '"$FIELD_PI_REPO"'/scripts/seeker/pi_capture.py ] && echo OK || echo MISSING)"
printf "ntp=%s\n" "$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown)"
printf "tz=%s\n" "$(timedatectl show -p Timezone --value 2>/dev/null || echo unknown)"
printf "freemb=%s\n" "$(df -Pm '"$FIELD_PI_REPO"' 2>/dev/null | awk "NR==2{print \$4}")"
printf "sessions=%s\n" "$(mkdir -p '"$FIELD_PI_REPO"'/sessions 2>/dev/null && touch '"$FIELD_PI_REPO"'/sessions/.fieldcheck 2>/dev/null && rm -f '"$FIELD_PI_REPO"'/sessions/.fieldcheck 2>/dev/null && echo WRITABLE || echo READONLY)"
'
PROBE_OUT="$(ssh -o BatchMode=yes -o ConnectTimeout="${FIELD_SSH_TIMEOUT:-6}" "$PI_TRY" "$REMOTE_PROBE" 2>/dev/null)"
get() { printf '%s\n' "$PROBE_OUT" | sed -n "s/^$1=//p" | head -1; }

fld_say "Pi hostname: $(get host)   addresses: $(get ips)"
for pair in "repo:$FIELD_PI_REPO" "venv:$FIELD_PI_PYTHON" \
            "capture:$FIELD_PI_REPO/scripts/seeker/pi_capture.py"; do
    k="${pair%%:*}"; v="${pair#*:}"
    if [[ "$(get "$k")" == "OK" ]]; then
        fld_ok "$k present: $v"
    else
        fld_bad "$k MISSING on the Pi: $v"
        FAILS=$((FAILS + 1))
        HINTS+=("provision the Pi: scripts/pi_setup/README.md (clone + provision.sh) -- missing: $v")
    fi
done
if [[ "$(get rsync)" == "MISSING" ]]; then
    fld_bad "rsync is NOT installed on the Pi -- 04_pull_logs.sh cannot fetch the session"
    FAILS=$((FAILS + 1))
    HINTS+=("on the Pi: sudo apt install rsync")
else
    fld_ok "rsync on the Pi: $(get rsync)"
fi
if [[ "$(get sessions)" == "WRITABLE" ]]; then
    fld_ok "sessions/ is writable (that is where pi_capture.py writes a pass)"
else
    fld_bad "cannot write $FIELD_PI_REPO/sessions on the Pi"
    FAILS=$((FAILS + 1))
    HINTS+=("fix ownership on the Pi: sudo chown -R \$USER $FIELD_PI_REPO")
fi
FREE="$(get freemb)"
if [[ -n "$FREE" ]]; then
    # A 1280x800 mono PNG runs ~0.4-0.8 MB; a 300-frame pass is a few hundred MB
    # and the matrix is ~28 passes (docs/tripod_test_protocol.md section 4.6).
    if ((FREE < 4096)); then
        fld_warn "only ${FREE} MB free on the Pi -- a ~28-pass day of PNGs will not fit"
        HINTS+=("free space on the Pi (or bring a bigger card) before the field day")
    else
        fld_ok "free space on the Pi: ${FREE} MB"
    fi
fi

# ------------------------------------------------------------- 5. the clock
fld_head "5. clock offset (the frame<->log join depends on this)"
T0="$(date +%s.%N)"
PI_EPOCH="$(ssh -o BatchMode=yes -o ConnectTimeout="${FIELD_SSH_TIMEOUT:-6}" "$PI_TRY" 'date +%s.%N' 2>/dev/null)"
T1="$(date +%s.%N)"
if [[ -n "$PI_EPOCH" ]]; then
    read -r OFFSET RTT <<<"$(awk -v a="$T0" -v b="$T1" -v p="$PI_EPOCH" \
        'BEGIN{printf "%.3f %.3f", p-(a+b)/2.0, b-a}')"
    fld_say "Pi clock - laptop clock = ${OFFSET} s   (measured over a ${RTT} s ssh round trip)"
    fld_say "NTP synchronised on the Pi: $(get ntp)   timezone: $(get tz)"
    if awk -v o="$OFFSET" -v bar="$CLOCK_BAR_S" 'BEGIN{exit !(((o<0)?-o:o) > bar)}'; then
        fld_bad "clock offset exceeds ${CLOCK_BAR_S} s = one 2 m range bin at 9 m/s (protocol 6.1)"
        FAILS=$((FAILS + 1))
        HINTS+=("NTP-sync the Pi on the hotspot: sudo timedatectl set-ntp true; timedatectl status")
        HINTS+=("a wrong Pi clock does NOT lose the day -- range_truth_join.py --clock-offset-s recovers it from the 6.1 sync event -- but fix it now, it is free")
    else
        fld_ok "within the ${CLOCK_BAR_S} s protocol 6.1 budget"
    fi
    if [[ "$(get ntp)" != "yes" ]]; then
        fld_warn "the Pi is not NTP-synchronised (systemd-timesyncd off, or no internet on the hotspot)"
        HINTS+=("the hotspot is the Pi's ONLY time source at the field -- keep phone data on, and re-check this within minutes of pass 1")
    fi
    fld_say "NOTE: this ssh-based measurement is only good to about half the round"
    fld_say "trip (${RTT} s). The AUTHORITATIVE clock link stays the protocol 6.1"
    fld_say "marked sync event -> range_truth_join.py --clock-offset-s."
else
    fld_warn "could not read the Pi's clock"
fi

# ---------------------------------------------------------------- 6. verdict
fld_head "SUMMARY"
HINTS+=("prove the whole chain now, at the desk:  scripts/field/01_camera_live_check.sh --pi $PI_TRY   then   scripts/field/04_pull_logs.sh --pi $PI_TRY --no-score")
if ((FAILS > 0)); then
    fld_finish "$FLD_FAIL" "$FAILS problem(s) on a Pi that IS reachable -- fix them at the desk" "${HINTS[@]}"
fi
fld_finish "$FLD_OK" \
    "the laptop<->Pi link works non-interactively, and the Pi has everything 01/04 need" \
    "${HINTS[@]}"
