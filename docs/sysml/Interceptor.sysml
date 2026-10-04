// =============================================================================
// Interceptor.sysml -- the lean section-0c ram interceptor (Tier 2): Source One
// V6 airframe, Pixhawk 6C Mini on PX4, Raspberry Pi 5 + OV9281 seeker, ELRS
// kill/arm-only RC. No datalink after launch (constraint no-datalink).
//
// Two independent power rails (the two-rail rule): PM02 -> flight controller,
// Matek BEC -> Raspberry Pi. The Pi is never powered from the FC's rail.
// The brain-bench wires (6C Mini <-> Pi on TELEM2, M10 on GPS1) are
// `verified` -- they were proven on the bench and move onto the airframe as
// built. Everything that needs the airframe is `planned` until afr-01..05.
// =============================================================================
package Interceptor {
    doc /* Physical architecture of the interceptor. */

    private import HardwareLibrary::*;
    private import Components::*;

    part def InterceptorDrone {
        doc /* PX4 5-inch quad carrying a Pi 5 seeker; ram vehicle. */

        // ---------------------------------------------------------- parts
        part battery : LiPo6S1500 {
            doc /* Shared 6S pack pool with the target. */
            @BOM { tab = "target"; bomName = "CNHL 6S 1500"; }
            @Layout { col = 0; row = 0; }
        }
        part pm02 : PM02V3 {
            @BOM { tab = "airframe"; bomName = "Holybro PM02 V3"; }
            @Layout { col = 1; row = 0; }
        }
        part esc : KO60IIESC {
            @BOM { tab = "airframe"; bomName = "KO60II"; }
            @Layout { col = 2; row = 0; }
        }
        part motors : EcoII2207 [4] {
            doc /* 5 ordered: 4 flight + 1 spare shared with the target. */
            @BOM { tab = "airframe"; bomName = "EMAX ECO II 2207"; }
            @Layout { col = 3; row = 0; }
        }
        part props : HQV2S5in [4] {
            @BOM { tab = "airframe"; bomName = "HQProp V2S"; }
            @Layout { col = 4; row = 0; }
        }
        part fcSd : MicroSDCard {
            doc /* ULogs are the kill metrology -- no card, no scored flight. */
            @BOM { tab = "airframe"; bomName = "FC High-Endurance microSD"; }
            @Layout { col = 0; row = 1; }
        }
        part fc : Pixhawk6CMini {
            @BOM { tab = "brain"; bomName = "Pixhawk 6C Mini"; }
            @Layout { col = 1; row = 1; }
        }
        part gps : HolybroM10 {
            @BOM { tab = "brain"; bomName = "Holybro M10 GPS"; }
            @Layout { col = 2; row = 1; }
        }
        part airframe : SourceOneV6 {
            @BOM { tab = "airframe"; bomName = "TBS Source One V6"; }
            @Layout { col = 3; row = 1; }
        }
        part bec : MatekBEC12SPro {
            @BOM { tab = "airframe"; bomName = "Matek 12S Pro BEC"; }
            @Layout { col = 0; row = 2; }
        }
        part pi : RaspberryPi5 {
            doc /* The same board as the Tier-1 seeker rig, moved onto the airframe. */
            @BOM { tab = "seeker"; bomName = "Raspberry Pi 5"; }
            @Layout { col = 1; row = 2; }
        }
        part rx : RP3Diversity {
            @BOM { tab = "airframe"; bomName = "RP3 ELRS"; }
            @Layout { col = 2; row = 2; }
        }
        part finder : FlywooFinderV1 {
            @BOM { tab = "ground"; bomName = "Flywoo Finder"; }
            @Layout { col = 3; row = 2; }
        }
        part railCap : RailCap1000uF {
            @BOM { tab = "airframe"; bomName = "1000 uF"; }
            @Layout { col = 0; row = 3; }
        }
        part camera : OV9281Camera {
            @BOM { tab = "seeker"; bomName = "OV9281"; }
            @Layout { col = 1; row = 3; }
        }
        part piSd : MicroSDCard {
            doc /* Pi OS + timestamped raw frames. */
            @BOM { tab = "seeker"; bomName = "SanDisk 128 GB"; }
            @Layout { col = 2; row = 3; }
        }
        part cooler : PiActiveCooler {
            @BOM { tab = "seeker"; bomName = "active cooler"; }
            @Layout { col = 3; row = 3; }
        }
        part cameraMount : NoseCameraMount {
            @BOM { tab = "airframe"; bomName = "Nose-cantilever"; }
            @Layout { col = 4; row = 3; }
        }

        // ---------------------------------------------------------- power: flight rail
        interface mainPower : DCPowerLink connect battery.main to pm02.battIn {
            attribute :>> connector = "XT60";
            attribute :>> detail = "6S main power into the power module";
            attribute :>> linkState = LinkState::planned;
        }
        interface escPower : DCPowerLink connect pm02.battOut to esc.batt {
            attribute :>> connector = "XT60 pigtail, 14 AWG";
            attribute :>> detail = "PM02 pass-through to the ESC pads";
            attribute :>> linkState = LinkState::wired;
        }
        interface motorLeads : PhaseLink connect esc.motorOut to motors.phase {
            attribute :>> connector = "3-phase leads, soldered";
            attribute :>> detail = "4 channels; verify order + direction in QGC (afr-03)";
            attribute :>> linkState = LinkState::planned;
        }
        interface fcPower : PowerModuleLink connect pm02.fcLead to fc.power1 {
            attribute :>> connector = "6-pin JST-GH";
            attribute :>> detail = "5.2 V + analog V/I sense to POWER1";
            attribute :>> linkState = LinkState::wired;
        }

        // ---------------------------------------------------------- power: Pi rail
        interface becFeed : DCPowerLink connect battery.main to bec.vin {
            attribute :>> connector = "16 AWG tap on the main leads";
            attribute :>> detail = "LiPo main straight to the BEC -- second rail";
            attribute :>> linkState = LinkState::wired;
        }
        interface piRail : DCPowerLink connect bec.out5v to pi.pwrIn {
            attribute :>> connector = "5 V lead to the Pi (entry point TBD)";
            attribute :>> detail = "5.2 V; USB-C pigtail or header 5 V/GND -- decide at afr-05";
            attribute :>> linkState = LinkState::planned;
        }
        interface railSmoothing : DCPowerLink connect bec.out5v to railCap.rail {
            attribute :>> connector = "soldered across the rail at the Pi end";
            attribute :>> detail = "1000 uF 16 V low-ESR";
            attribute :>> linkState = LinkState::planned;
        }
        interface fan : FanLink connect pi.fanHeader to cooler.header {
            attribute :>> connector = "4-pin fan header";
            attribute :>> detail = "5 V + PWM + tach";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-07";
        }

        // ---------------------------------------------------------- data
        interface escSignal : DShotLink connect fc.mainOut to esc.signal {
            attribute :>> connector = "signal lead (pin map TBD)";
            attribute :>> detail = "DShot600 M1-M4 + GND; contract says JST-SH 8-pin, wiring card says MAIN is 3-pin S/+/- -- check at afr-01";
            attribute :>> linkState = LinkState::wired;
        }
        interface gpsCable : GPSCableLink connect fc.gps1 to gps.fcConn {
            attribute :>> connector = "10-pin JST-GH";
            attribute :>> detail = "GPS UART (UBX 10 Hz) + IST8310 compass on GPS1's own I2C + 5 V";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "brn-02";
        }
        interface companion : UARTLink connect fc.telem2 to pi.gpioUart {
            @BOM { tab = "brain"; bomName = "Pi-to-TELEM2"; }
            attribute :>> connector = "JST-GH 6-pin -> DuPont (3 wires)";
            attribute :>> detail = "MAVLink 921600: pin2 TX -> GPIO15 (pin 10), pin3 RX <- GPIO14 (pin 8), pin6 GND -> pin 6; pin1 +5 V NOT connected";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "brn-03, brn-05";
        }
        interface rcSbus : RCInLink connect fc.rcIn to rx.fcConn {
            attribute :>> connector = "RC IN socket";
            attribute :>> detail = "SBUS (ELRS flashed to SBUS out) + 5 V; kill/arm only";
            attribute :>> linkState = LinkState::wired;
        }
        interface buzzer : BuzzerLink connect fc.buzzerTap to finder.fcConn {
            attribute :>> connector = "TBD";
            attribute :>> detail = "drive pin UNRESOLVED -- the 6C Mini has no buzzer pad";
            attribute :>> linkState = LinkState::wired;
        }
        interface ulogCard : SDCardLink connect fc.sdSlot to fcSd.contacts {
            attribute :>> connector = "onboard microSD slot";
            attribute :>> detail = "PX4 ULogs";
            attribute :>> linkState = LinkState::wired;
        }
        interface cameraRibbon : CSI2Link connect camera.csi to pi.cam0 {
            @BOM { tab = "seeker"; bomName = "Wonrabai"; }
            attribute :>> connector = "22-to-15-pin CSI ribbon";
            attribute :>> detail = "MIPI CSI-2 into CAM/DISP0; 1280 x 800 mono";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-02";
        }
        interface frameCard : SDCardLink connect pi.sdSlot to piSd.contacts {
            attribute :>> connector = "Pi microSD slot";
            attribute :>> detail = "Pi OS + raw PNG frames";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-01, skr-05";
        }

        // ---------------------------------------------------------- mechanical
        interface motorMounts : MountLink connect airframe.arms to motors.base {
            attribute :>> connector = "M3x8 screws";
            attribute :>> linkState = LinkState::planned;
        }
        interface propShafts : MountLink connect motors.shaft to props.hub {
            attribute :>> connector = "M5 shaft + nyloc";
            attribute :>> linkState = LinkState::planned;
        }
        interface escMount : MountLink connect airframe.stack to esc.standoffs {
            attribute :>> connector = "30 mm standoffs";
            attribute :>> detail = "insulate the frame under every board (carbon conducts)";
            attribute :>> linkState = LinkState::planned;
        }
        interface fcMount : MountLink connect airframe.stack to fc.softMount {
            attribute :>> connector = "vibration mount";
            attribute :>> linkState = LinkState::planned;
        }
        interface gpsMast : MountLink connect airframe.topPlate to gps.mast {
            attribute :>> connector = "short mast";
            attribute :>> detail = "compass away from power leads";
            attribute :>> linkState = LinkState::planned;
        }
        interface piTray : MountLink connect airframe.topPlate to pi.standoffs {
            @BOM { tab = "airframe"; bomName = "M2.5 nylon standoff"; }
            attribute :>> connector = "printed tray + M2.5 nylon standoffs";
            attribute :>> linkState = LinkState::planned;
        }
        interface noseMount : MountLink connect airframe.nose to cameraMount.frameFoot {
            attribute :>> connector = "nose cantilever";
            attribute :>> detail = "corner-spin prop-clearance gate at loaded throttle (afr-05)";
            attribute :>> linkState = LinkState::planned;
        }
        interface cameraSeat : MountLink connect cameraMount.cameraSeat to camera.mount {
            attribute :>> connector = "adjustable 10-30 deg bracket";
            attribute :>> detail = "angle locked from the first dash ULog";
            attribute :>> linkState = LinkState::planned;
        }
        interface batteryStrap : MountLink connect airframe.batteryPad to battery.strap {
            attribute :>> connector = "strap";
            attribute :>> linkState = LinkState::planned;
        }
    }
}
