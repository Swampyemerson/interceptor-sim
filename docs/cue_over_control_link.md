# Sending the target's GPS down the control link

Written 2026-11-18. Desk research plus a laptop program and its tests. Nothing
here has been tried on the real transmitter yet; the section "Not checked yet"
lists exactly what that leaves open.

## The question

"Can we not run it over the control link into a USB that goes into the laptop
that goes to the Pi over Wi-Fi?"

## The short answer

Yes. The target's autopilot already sends its GPS position back down the
ExpressLRS radio link to the handset, and the handset can copy those bytes out
of its USB port. No firmware has to change mode: two menu settings on the
handset and one on the radio link. The laptop program for it is written and
tested against made-up data: `scripts/field/crsf_cue_source.py`.

The catch is quality. Over this route the autopilot sends its position **at
most twice a second**, as speed and direction of travel (not a full velocity),
with height in whole metres. The Pi's cue receiver wants at least 8 position
reports inside its 5.5 second window and the newest one under 1 second old.
Twice a second gives 11, so it passes, with a margin of three lost reports.

Try this route first because it cannot hurt the control link. If twice a
second turns out not to be good enough, the next step is a choice between two
things described below, and one of them changes how the kill switch reaches
the autopilot.

## The three routes

### Route A: ordinary telemetry, copied out of the handset's USB port (recommended first)

How it works: ArduPilot sends small telemetry packets (GPS, attitude, battery,
flight mode) to the receiver on the same wires that carry stick commands. The
receiver radios them to the handset. EdgeTX, the handset's software, has a
setting called "Telem Mirror" that copies every telemetry byte it receives to
a serial port, and the USB port can be that serial port.

What comes down, per GPS packet:

| Field | Step size | Note |
|---|---|---|
| Latitude, longitude | 0.0000001 degree (about 1 cm) | finer than GPS itself |
| Ground speed | 0.1 km/h (0.03 m/s) | |
| Direction of travel | 0.01 degree | "course over ground", not where the nose points; meaningless when nearly stopped |
| Height | 1 metre | GPS height above sea level; ArduPilot clips it to 0 to 5000 m |
| Satellites | whole number | |

There is no north/east velocity and no climb rate in the packet. The laptop
program works out north and east speed from speed and direction, and sends
climb rate as zero. There is also no "I have a fix" flag, so the program
treats fewer than 4 satellites, or a position of exactly 0,0, as no fix and
refuses to send it.

How often: ArduPilot 4.7 sends the GPS packet at most every 0.5 s when the
radio link gives it more than 30 telemetry slots a second, and at most every
1 s otherwise. One a second is **not enough** for the Pi's receiver (it would
see 5 or 6 reports and refuse, which is the safe failure). So the radio link's
telemetry ratio has to be raised:

| Packet rate | Telemetry ratio | Telemetry slots per second | Room for the data? |
|---|---|---|---|
| 250 Hz | Std (1:64) | 4 | No. A GPS packet every few seconds at best |
| 250 Hz | 1:8 | 31 | Barely over ArduPilot's threshold; not recommended |
| 250 Hz | **1:4** | 62 | Yes (about 300 bytes a second against roughly 110 to 130 needed) |
| 250 Hz | 1:2 | 125 | Yes, most headroom |
| 150 Hz | 1:4 | 37 | Just |
| 150 Hz | 1:2 | 75 | Yes |
| 50 Hz | any | 25 or fewer | No: ArduPilot drops to one GPS packet a second |

The slots and bytes come from the ExpressLRS bandwidth table; "needed" is my
arithmetic from ArduPilot's packet sizes and rates, not a measurement.

Delay: a GPS packet is 19 bytes and the link carries 5 bytes per telemetry
slot, so it takes 4 slots, about 65 ms at 250 Hz 1:4, plus waiting behind
other packets. At 9 to 12 m/s every 100 ms of delay is about a metre of
position lag. The packet carries no time stamp, so the laptop stamps arrival
time and the delay's jitter goes into the Pi's fit. This has not been
measured.

### Route B: ExpressLRS "MAVLink mode"

How it works: from ExpressLRS 3.5.0 the link can carry MAVLink, the
autopilot's full telemetry language, instead of the small packets. The laptop
then gets the same messages a telemetry radio would give: position with a real
north/east/down velocity and the autopilot's own clock, 5 or more times a
second. `scripts/field/cue_send.py --mavlink` already reads that.

What changes:

- Handset: ExpressLRS Lua script, with the receiver powered off, set
  "Link Mode" to MAVLink. This forces telemetry ratio 1:2 and "Hybrid" switch
  mode ("Wide" is not supported).
- Receiver: on ExpressLRS 3.x set its serial protocol to MAVLink in the Lua
  script's "Other Devices"; on 4.x it follows the transmitter.
- Autopilot: SERIAL6_PROTOCOL from 23 to 2, SERIAL6_BAUD 460 (the ExpressLRS
  page; ArduPilot's own page shows 115 in its example), RSSI_TYPE 5, and
  the message rates for that port set low (the ExpressLRS page says 1 for
  each; raising the position rate to 5 is my suggestion and is untested).
- Where the data comes out: **not the USB port on this handset.** The
  ExpressLRS page says USB is not available for internal modules, and the
  Pocket's module is internal. It comes out over Wi-Fi from the handset's
  "backpack" (a second small chip; the Pocket has one), as UDP on port 14550.
  The laptop either joins the backpack's own Wi-Fi network (range 5 to 10 m)
  or the backpack joins the same Wi-Fi as the laptop and the Pi.
- Capacity: about 590 bytes a second down at 250 Hz, about 1470 at "333 Hz
  Full". A position message is about 40 bytes, so 5 a second is 200 bytes a
  second.

**Stick control and the kill switch do not work the same way.** In MAVLink
mode the receiver stops speaking the radio-control protocol to the autopilot.
It sends the stick and switch positions as MAVLink "override" messages (up to
100 a second) on the same port as the telemetry. ArduPilot's rule for those is
different: when they stop it waits a set time (RC_OVERRIDE_TIME, "a few
seconds" in ArduPilot's own words) before deciding the pilot is gone. I could
not confirm how long the kill switch and the lost-link failsafe take in this
mode on ArduCopter 4.7. That has to be measured with the props off before any
flight: flip the kill switch, then switch the handset off, and time both.

### Route C: other things this exact hardware can do

- **Backpack Wi-Fi, without changing link mode.** The backpack's source code
  forwards each ordinary telemetry packet as a UDP broadcast on port 14550
  when "Backpack > Telemetry" is set to WiFi. Same data and same twice a
  second as Route A, no cable. Against it: it switches on a Wi-Fi transmitter
  inside the handset, next to the 2.4 GHz control radio (the ExpressLRS page
  says to leave backpack telemetry off when not needed for that reason), the
  laptop has to be on the backpack's network, and I could not confirm which
  released backpack version does this. The laptop program has an experimental
  `--udp-listen 14550` option for it. Untested.
- **Backpack ESP-NOW.** Documented, but it needs another small ESP board as
  the listener. Not worth it here.
- **A separate small telemetry radio on a spare autopilot port.** Not over the
  control link, so it is outside the question, but it is the one option that
  gives the full 5 to 10 reports a second and leaves the kill-switch link
  exactly as it is. `cue_send.py --mavlink` already reads it.

## Recommendation

1. Try Route A on the bench. It costs ten minutes and risks nothing on the
   control link apart from the telemetry ratio change, which is reversible
   from the same menu.
2. Record what arrives (`--record`), and read the status line: is the GPS
   rate a steady 2 Hz, are there CRC failures, how many satellites.
3. If the Pi's receiver accepts it and the dash aim is good enough at twice a
   second, stop there.
4. If it is not good enough, choose between Route B (one link, better data,
   but the kill-switch path changes and must be re-proven on the bench) and a
   separate telemetry radio (extra hardware, kill switch untouched). I would
   lean to the separate radio, because the control link is the kill-switch
   link. That is your call.

## Steps for your hands (Route A)

Props off for all of this.

1. **Check telemetry already reaches the handset.** Power the target and the
   handset. On the handset: Model > Telemetry > "Discover new". With a GPS fix
   you should see GPS, GSpd, Hdg, Sats and a height value appear alongside
   the link values (RQly, 1RSS). If only link values appear, the autopilot's
   telemetry is not reaching the receiver: the wire from the autopilot's
   SERIAL6 transmit pad (T6 on the Kakute; check the board's pinout) to the
   receiver's RX pad is missing, and nothing below will work until it is
   there.
2. **Raise the telemetry ratio.** System > Tools > ExpressLRS. Set "Telem
   Ratio" to 1:4 (or 1:2). Leave "Packet Rate" alone. Do this disarmed: the
   ExpressLRS page says not to change settings while armed, and never to
   change packet rate in flight because it forces a disconnect.
3. **Set the USB port to copy telemetry.** System > Hardware > Serial ports >
   USB-VCP: change it to "Telem Mirror". (It is probably on "CLI" now, which
   is what firmware updates of the internal radio module use. Put it back to
   CLI before the next ExpressLRS update.)
4. **Make USB come up as a serial port.** System > Radio Setup > "USB Mode":
   set to Serial. Otherwise the handset asks each time; if it asks, choose
   "USB Serial (VCP)", never "Storage".
5. **Plug the handset's data USB port into the laptop** and find the port
   name in a Windows terminal:

       py -m pip install pyserial
       py -m serial.tools.list_ports -v

6. **Confirm the sticks still work with the cable in.** Move a stick and
   watch the channels respond on your ground station's radio screen (props
   off), and check the kill switch.
7. **Look at what arrives, sending nothing** (use your port name for COM7):

       py scripts\field\crsf_cue_source.py --port COM7 --dry-run --record crsf_first_look.bin

   Once a second it prints packets per second by type, the GPS rate and CRC
   failures. You want "GPS(0x02)=2.0" and "CRC fail 0". Keep the .bin file:
   it is the first real evidence of what this handset sends.

## The laptop command

Under Windows Python (WSL cannot see COM ports):

    py scripts\field\crsf_cue_source.py --port COM7 --to PI_ADDRESS

It sends one small message per GPS packet to the Pi on UDP port 47810, the
same message format `flight/cue_link.py` already receives. It only listens to
the handset; it never writes to it. Close QGroundControl or anything else that
might hold the COM port. Ctrl-C stops it and prints totals. If it sent
nothing, it says why and exits with an error.

It will not send a position with no fix: fewer than 4 satellites
(`--min-sats` changes that, never below 1), a 0,0 position, or an impossible
value. Each refusal is printed with "NOT SENT" and counted on the status line.

## Not checked yet

- That this Pocket, on its installed EdgeTX version, offers "Telem Mirror"
  for USB-VCP and sends the raw packets. The current EdgeTX source code says
  it does, for the internal module too. Nobody has looked at the bytes.
- Whether the handset keeps flying the model normally with USB in Serial
  mode (step 6 is the check).
- Which of the Pocket's USB sockets carries data.
- Whether the serial port needs the DTR line set to start sending
  (`--dtr on` or `--dtr off` if nothing arrives).
- The real GPS rate, the delay and its jitter over the link.
- Whether the ExpressLRS receiver passes on every GPS packet the autopilot
  gives it (I could not read that part of its source).
- Everything about Routes B and C on this hardware, and the failsafe timing
  in Route B in particular.
- The versions on this handset, receiver and backpack. Route B needs
  ExpressLRS 3.5.0 or later on both ends and backpack 1.5.0 or later.

## Limits to remember

- Twice a second at best, against the 5 a second the cue plan assumed. The
  newest report at GO is up to 0.5 s old plus link delay, where the plan
  assumed 0.2 s.
- Speed and direction of travel, not a velocity. No climb rate (sent as 0).
- Height in whole metres.
- No time stamp from the autopilot; arrival time on the laptop is used.
- "No fix" is inferred, because the packet has no fix flag.
- ArduPilot 4.6.0 and older sent ground speed 23% too low in this packet.
  4.6.3 and 4.7.0 are correct. The target is on 4.7; if it is ever
  downgraded, this route's speed is wrong.

## Sources

Read 2026-11-18.

- Packet layout, check code and GPS fields: TBS CRSF specification,
  https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
- What ArduPilot puts in the GPS packet and how often: `calc_gps()` and
  `update_custom_telemetry_rates()` in
  https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.0/libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
- ArduPilot set-up for CRSF and ExpressLRS, and MAVLink over ExpressLRS:
  https://ardupilot.org/copter/docs/common-tbs-rc.html
- ArduPilot on MAVLink stick input and RC_OVERRIDE_TIME:
  https://ardupilot.org/dev/docs/mavlink-rcinput.html
- EdgeTX serial port options (Telem Mirror, USB-VCP):
  https://manual.edgetx.org/bw-radios/radio-settings/hardware
  and USB Mode: https://manual.edgetx.org/bw-radios/radio-settings/radio-setup
- EdgeTX source, the mirror and the packet reader:
  https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/telemetry.cpp ,
  https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/crossfire.cpp ,
  https://github.com/EdgeTX/edgetx/blob/main/radio/src/serial.cpp
- ExpressLRS MAVLink mode (versions, settings, USB and Wi-Fi, capacity):
  https://www.expresslrs.org/software/mavlink/
- ExpressLRS telemetry bandwidth table:
  https://www.expresslrs.org/info/telem-bandwidth/
- ExpressLRS Lua script settings and warnings:
  https://www.expresslrs.org/quick-start/transmitters/lua-howto/
- ExpressLRS backpack telemetry: https://expresslrs.org/software/backpack-telemetry
- ExpressLRS source: default telemetry ratios in
  https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/common.cpp ,
  the receiver's MAVLink output in
  https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/rx-serial/SerialMavlink.cpp
- The Pocket's module and backpack, and the RP1 receiver, in the ExpressLRS
  hardware lists: https://github.com/ExpressLRS/targets/blob/master/targets.json ,
  https://github.com/ExpressLRS/Backpack/blob/master/hardware/targets.json
- Backpack Wi-Fi forwarding:
  https://github.com/ExpressLRS/Backpack/blob/master/src/Tx_main.cpp ,
  https://github.com/ExpressLRS/Backpack/blob/master/lib/WIFI/devWIFI.cpp
