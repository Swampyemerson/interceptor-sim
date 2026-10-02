// =============================================================================
// Target.sysml -- the target drone: a cheap ArduPilot 5-inch quad that flies
// repeatable AUTO legs with an AprilTag placard on top and logs its own track.
//
// linkState tells you how far each wire has got on the REAL aircraft. The
// original Kakute's baro/I2C bus died (diagnosis closed 2026-09-23) and a
// replacement is on order, so every flight-controller-side wire is back to
// `planned` even though it was once made on the dead board.
// =============================================================================
package Target {
    doc /* Physical architecture of the target drone. */

    private import HardwareLibrary::*;
    private import Components::*;

    part def TargetDrone {
        doc /* ArduPilot 5-inch quad, 6S, AprilTag placard on the top plate. */

        // ---------------------------------------------------------- parts
        part battery : LiPo6S1500 {
            @BOM { tab = "target"; bomName = "CNHL 6S 1500"; }
            @Layout { col = 0; row = 0; }
        }
        part esc : Tekko32F4Metal65A {
            @BOM { tab = "target"; bomName = "Tekko32"; }
            @Layout { col = 1; row = 0; }
        }
        part motors : EcoII2207 [4] {
            @BOM { tab = "target"; bomName = "EMAX ECO II 2207"; }
            @Layout { col = 2; row = 0; }
        }
        part props : HQV2S5in [4] {
            @BOM { tab = "target"; bomName = "HQ V2S"; }
            @Layout { col = 3; row = 0; }
        }
        part sd : MicroSDCard {
            doc /* DataFlash .BIN -- no card, no target track, no scored pass. */
            @BOM { tab = "target"; bomName = "microSD 32-64 GB"; }
            @Layout { col = 0; row = 1; }
        }
        part fc : KakuteH7V15 {
            @BOM { tab = "target"; bomName = "Kakute H7 V1.5"; }
            @Layout { col = 1; row = 1; }
        }
        part gps : MatekM10Q5883 {
            @BOM { tab = "target"; bomName = "Matek M10Q-5883"; }
            @Layout { col = 2; row = 1; }
        }
        part airframe : SourceOneV6 {
            @BOM { tab = "target"; bomName = "TBS Source One V6"; }
            @Layout { col = 3; row = 1; }
        }
        part finder : FlywooFinderV1 {
            @BOM { tab = "ground"; bomName = "Flywoo Finder"; }
            @Layout { col = 0; row = 2; }
        }
        part rx : RP1V2 {
            @BOM { tab = "target"; bomName = "RP1 V2"; }
            @Layout { col = 1; row = 2; }
        }
        part placard : AprilTagPlacard {
            @BOM { tab = "target"; bomName = "AprilTag tag36h11 placard"; }
            @Layout { col = 3; row = 2; }
        }

        // ---------------------------------------------------------- power
        interface mainPower : DCPowerLink connect battery.main to esc.batt {
            attribute :>> connector = "XT60 pigtail";
            attribute :>> detail = "6S main power, 22.2 V nominal";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "tgt-04";
        }
        interface motorLeads : PhaseLink connect esc.motorOut to motors.phase {
            attribute :>> connector = "3-phase leads, soldered";
            attribute :>> detail = "4 channels; order + direction remapped in software";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "tgt-05";
        }

        // ---------------------------------------------------------- power + data
        interface stackHarness : StackHarnessLink connect esc.fcHarness to fc.escHarness {
            attribute :>> connector = "8-pin stack harness";
            attribute :>> detail = "VBAT + current sense up; DShot600 M1-M4 down";
            attribute :>> linkState = LinkState::planned;
            attribute :>> evidence = "tgt-05";
            doc /* Proven on the original board 2026-09-23; re-made on the
                 * replacement Kakute. */
        }
        interface gpsCable : GPSCableLink connect fc.uart3Gps to gps.fcConn {
            attribute :>> connector = "6-wire harness to pads";
            attribute :>> detail = "UART3 (SERIAL3_PROTOCOL=5) + I2C compass + 5 V";
            attribute :>> linkState = LinkState::planned;
            attribute :>> evidence = "tgt-02";
            doc /* 3D fix proven on the original board (tgt-02), then lost after
                 * the RX solder session -- re-make and wiggle-test on the
                 * replacement. */
        }
        interface rcCrsf : PoweredUARTLink connect fc.uart6Rc to rx.fcConn {
            attribute :>> connector = "soldered pads";
            attribute :>> detail = "CRSF: RX TX -> R6, RX RX -> T6, 5 V, GND (BRD_ALT_CONFIG=1)";
            attribute :>> linkState = LinkState::planned;
            attribute :>> evidence = "tgt-05";
        }
        interface buzzer : BuzzerLink connect fc.buzzerPads to finder.fcConn {
            attribute :>> connector = "buzzer pads";
            attribute :>> detail = "5 V + BZ- drive";
            attribute :>> linkState = LinkState::planned;
        }

        // ---------------------------------------------------------- data
        interface logCard : SDCardLink connect fc.sdSlot to sd.contacts {
            attribute :>> connector = "onboard microSD slot";
            attribute :>> detail = "DataFlash .BIN with POS + GPS (LOG_BITMASK=176126)";
            attribute :>> linkState = LinkState::planned;
        }

        // ---------------------------------------------------------- mechanical
        interface motorMounts : MountLink connect airframe.arms to motors.base {
            attribute :>> connector = "M3x8 screws";
            attribute :>> detail = "6.0 mm arm -> 2 mm thread engagement";
            attribute :>> linkState = LinkState::wired;
            attribute :>> evidence = "tgt-03";
        }
        interface propShafts : MountLink connect motors.shaft to props.hub {
            attribute :>> connector = "M5 shaft + nyloc";
            attribute :>> detail = "props on only after the props-off gates";
            attribute :>> linkState = LinkState::planned;
        }
        interface stackMount : MountLink connect airframe.stack to esc.standoffs {
            attribute :>> connector = "30.5 mm stack standoffs";
            attribute :>> detail = "ESC under, Kakute on top; insulate from carbon";
            attribute :>> linkState = LinkState::wired;
            attribute :>> evidence = "tgt-03";
        }
        interface fcMount : MountLink connect airframe.stack to fc.softMount {
            attribute :>> connector = "30.5 mm soft grommets";
            attribute :>> detail = "replacement board";
            attribute :>> linkState = LinkState::planned;
        }
        interface gpsMast : MountLink connect airframe.topPlate to gps.mast {
            attribute :>> connector = "mast / top plate";
            attribute :>> detail = "away from power leads (compass EMI)";
            attribute :>> linkState = LinkState::wired;
            attribute :>> evidence = "tgt-02";
        }
        interface placardMount : MountLink connect airframe.topPlate to placard.base {
            attribute :>> connector = "printed placard mount";
            attribute :>> detail = "rigid, facing the approach; index-disc angle recorded";
            attribute :>> linkState = LinkState::planned;
        }
        interface batteryStrap : MountLink connect airframe.batteryPad to battery.strap {
            attribute :>> connector = "strap";
            attribute :>> detail = "top- or bottom-mount per CG";
            attribute :>> linkState = LinkState::planned;
        }
    }
}
