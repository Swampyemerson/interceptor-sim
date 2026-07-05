#!/usr/bin/env bash
# Kill any running PX4 SITL + Gazebo processes. Exists as a FILE so callers
# never carry these pkill patterns in their own command line — `pkill -f`
# matches full command lines, and an inline `pkill -f "gz sim"` typed into
# an interactive shell (or a bash -c one-liner) matches ITSELF and kills the
# caller's shell (exit 144). Hit three times during M3/M4 GUI sessions
# before this script existed. Check scripts embed their own copies (their
# cmdline is just the script path, so they're immune).
pkill -f "bin/px4" 2>/dev/null
pkill -f "px4_sitl" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "gz-sim" 2>/dev/null
sleep 2
exit 0
