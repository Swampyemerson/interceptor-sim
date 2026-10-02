// =============================================================================
// Components.sysml -- one PART DEF per kind of hardware actually in the build.
//
// A part def is the catalogue entry (what the board IS and which plugs it has);
// the systems in Target.sysml / Interceptor.sysml / SeekerRig.sysml /
// GroundSegment.sysml are PART USAGES of these (where each one sits and what it
// is wired to). Port names are what the diagrams print next to each plug, so
// they follow the silkscreen where one exists (TELEM2, GPS1, R6/T6 = uart6).
// =============================================================================
package Components {
    doc /* Catalogue of the hardware used by the target drone, the interceptor,
         * the Tier-1 seeker rig and the ground segment. */

    private import HardwareLibrary::*;
    private import SI::*;

    // ------------------------------------------------------------ power
    part def LiPo6S1500 {
        doc /* CNHL Black 6S 1500 mAh 100C, XT60. One shared pack pool for both
             * aircraft and one charger. */
        port main : DCPowerPort { attribute :>> nominal = 22.2 [V]; }
        port balance : ~ChargePort;
        port strap : ~MountPort;
    }
    part def PM02V3 {
        doc /* Holybro PM02 V3 ANALOG power module (the digital PM02D does not
             * match the 6C Mini). XT60 pass-through plus a 6-pin lead carrying
             * 5.2 V and analog V/I sense to POWER1. */
        port battIn : ~DCPowerPort;
        port battOut : DCPowerPort { attribute :>> nominal = 22.2 [V]; }
        port fcLead : PowerModulePort;
    }
    part def MatekBEC12SPro {
        doc /* Matek 12S Pro BEC -- the dedicated Raspberry Pi rail (two-rail
             * rule: the Pi is never powered from the flight controller). Spec
             * says 5.2 V on the 5 V tap; bench-verify under Pi load. */
        port vin : ~DCPowerPort;
        port out5v : DCPowerPort { attribute :>> nominal = 5.2 [V]; }
    }
    part def RailCap1000uF {
        doc /* 1000 uF 16 V low-ESR electrolytic across the Pi 5.2 V rail at the
             * Pi end -- smooths load steps. Found missing from the kit at the
             * 2026-09-29 power-tree bench. */
        port rail : ~DCPowerPort;
    }
    part def USBCPSU27W {
        doc /* Official Raspberry Pi 27 W USB-C PSU -- bench rail for the Pi. */
        port usbC : DCPowerPort { attribute :>> nominal = 5.1 [V]; }
    }
    part def USBCPowerBank {
        doc /* >= 27 W USB-C PD power bank -- field rail for the tripod Pi. */
        port usbC : DCPowerPort { attribute :>> nominal = 5.0 [V]; }
    }
    part def Cells18650Pair {
        doc /* Two 18650 Li-ion cells -- the RC transmitter's power. */
        port output : DCPowerPort { attribute :>> nominal = 7.4 [V]; }
    }
    part def BalanceCharger1to6S {
        doc /* 1-6S balance charger, AC input. Always inside the fireproof bag. */
        port output : ChargePort;
    }

    // ------------------------------------------------------------ propulsion
    part def Tekko32F4Metal65A {
        doc /* Holybro Tekko32 F4 Metal 65A 4-in-1 ESC, 30.5 x 30.5, BLHeli_32,
             * DShot600. Ships the matched 8-pin harness for the Kakute H7. */
        port batt : ~DCPowerPort;
        port fcHarness : StackHarnessPort;
        port motorOut : PhasePort;
        port standoffs : ~MountPort;
    }
    part def KO60IIESC {
        doc /* Skystars KO60II 60A 3-6S AM32 4-in-1 ESC, 30 x 30, DShot600. */
        port batt : ~DCPowerPort;
        port signal : ~DShotPort;
        port motorOut : PhasePort;
        port standoffs : ~MountPort;
    }
    part def EcoII2207 {
        doc /* EMAX ECO II 2207 1900KV (6S). Mount with M3x8 -- the Source One
             * V6 arm is 6.0 mm, so the M3x6 screws engage zero thread. */
        port phase : ~PhasePort;
        port base : ~MountPort;
        port shaft : MountPort;
    }
    part def HQV2S5in {
        doc /* HQProp V2S 5x4.3x3 tri-blade, CW + CCW. */
        port hub : ~MountPort;
    }
    part def SourceOneV6 {
        doc /* TBS Source One V6 5-inch frame -- same frame on both aircraft, so
             * arms and spares are shared. Carbon conducts: insulate under boards. */
        port arms : MountPort;
        port stack : MountPort;
        port topPlate : MountPort;
        port nose : MountPort;
        port batteryPad : MountPort;
    }

    // ------------------------------------------------------------ flight control
    part def KakuteH7V15 {
        doc /* Holybro Kakute H7 V1.5 running ArduPilot (firmware target
             * KakuteH7). BRD_ALT_CONFIG=1 moves RC onto USART6 (pads R6/T6);
             * GPS on SERIAL3; DataFlash .BIN on its own microSD. */
        port escHarness : ~StackHarnessPort;
        port uart3Gps : GPSConnPort;
        port uart6Rc : PoweredUARTPort;
        port sdSlot : SDHostPort;
        port buzzerPads : BuzzerPort;
        port usbC : ~USBHostPort;
        port softMount : ~MountPort;
    }
    part def Pixhawk6CMini {
        doc /* Holybro Pixhawk 6C Mini (FMUv6C, STM32H743) running PX4 v1.16 --
             * the stack the simulator validated. TELEM2 is the companion link
             * (MAV_1_CONFIG=TELEM2, 921600 baud, flow control off). */
        port power1 : ~PowerModulePort;
        port mainOut : DShotPort;
        port gps1 : GPSConnPort;
        port telem2 : UARTPort { attribute :>> baud = 921600; }
        port rcIn : RCInPort;
        port sdSlot : SDHostPort;
        port buzzerTap : BuzzerPort {
            doc /* UNRESOLVED: the 6C Mini has no dedicated buzzer pad
                 * (hardware_order_list section 0b). Which pin drives the
                 * Finder is decided at afr-01. */
        }
        port usbC : ~USBHostPort;
        port softMount : ~MountPort;
    }

    // ------------------------------------------------------------ navigation
    part def MatekM10Q5883 {
        doc /* Matek M10Q-5883: u-blox M10 GNSS + QMC5883 compass. */
        port fcConn : ~GPSConnPort;
        port antenna : ~GNSSBroadcastPort;
        port mast : ~MountPort;
    }
    part def HolybroM10 {
        doc /* Holybro M10 GPS + IST8310 compass, 10-pin JST-GH to GPS1. */
        port fcConn : ~GPSConnPort;
        port antenna : ~GNSSBroadcastPort;
        port mast : ~MountPort;
    }

    // ------------------------------------------------------------ RC
    part def RP1V2 {
        doc /* RadioMaster RP1 V2 ELRS 2.4 GHz nano receiver, wired CRSF. */
        port fcConn : ~PoweredUARTPort;
        port radio : ~RadioTxPort;
    }
    part def RP3Diversity {
        doc /* RadioMaster RP3 ELRS diversity receiver (UFL V2), flashed to SBUS
             * output -- stock PX4 has no CRSF driver and the Mini has no spare
             * UART. Kill/arm-only link (constraint no-datalink). */
        port fcConn : ~RCInPort;
        port radio : ~RadioTxPort;
    }
    part def PocketTX {
        doc /* RadioMaster Pocket ELRS 2.4 GHz transmitter (EdgeTX). One per
             * aircraft, so each keeps an independent kill (ADR-0089). Same ELRS
             * major version + bind phrase as its receiver; Model Match on. */
        port radio : RadioTxPort;
        port cells : ~DCPowerPort;
    }

    // ------------------------------------------------------------ compute + vision
    part def RaspberryPi5 {
        doc /* Raspberry Pi 5, 8 GB. Runs capture, AprilTag decode (96.6 fps
             * measured), the relative-state filter and guidance; talks MAVLink
             * to the FC over GPIO14/15. */
        port pwrIn : ~DCPowerPort;
        port cam0 : ~CSI2Port {
            doc /* The ribbon is on CSI port 0 (CAM/DISP0), not the default
                 * port 1 -- found at skr-02. */
        }
        port gpioUart : ~UARTPort;
        port sdSlot : SDHostPort;
        port fanHeader : FanHeaderPort;
        port wifi : ~WiFiPort;
        port standoffs : ~MountPort;
    }
    part def PiActiveCooler {
        doc /* Raspberry Pi 5 active cooler -- mandatory under sustained CV load. */
        port header : ~FanHeaderPort;
    }
    part def OV9281Camera {
        doc /* innomaker OV9281 mono global-shutter MIPI camera, 1280 x 800,
             * 118 deg horizontal FoV (vendor-stated). */
        port csi : CSI2Port;
        port lens : ~TagFacePort;
        port mount : ~MountPort;
    }
    part def MicroSDCard {
        doc /* A microSD card. Which card is which is on the part usage. */
        port contacts : ~SDHostPort;
    }

    // ------------------------------------------------------------ mechanical / targets
    part def NoseCameraMount {
        doc /* Printed nose-cantilever mount with an adjustable 10-30 deg up-tilt
             * bracket -- puts the props out of the camera's view (no software
             * fallback). Angle locked from the first dash ULog. */
        port cameraSeat : MountPort;
        port frameFoot : ~MountPort;
    }
    part def AprilTagPlacard {
        doc /* tag36h11 id 0, 0.35 m edge, rigid; printed mount with an index
             * disc whose angle is recorded for scoring. */
        port face : TagFacePort;
        port base : ~MountPort;
    }
    part def Checkerboard9x6 {
        doc /* 9 x 6 inner-corner calibration board, rigid backing. */
        port face : TagFacePort;
    }
    part def TripodHead {
        doc /* Tripod + ball head + inclinometer -- the Tier-1 seeker rig's mount. */
        port head : MountPort;
    }
    part def FlywooFinderV1 {
        doc /* Flywoo Finder V1.0 lost-model buzzer with LED and its own cell. */
        port fcConn : ~BuzzerPort;
    }

    // ------------------------------------------------------------ owned / external
    part def Laptop {
        doc /* The builder's laptop: QGroundControl / Mission Planner, SSH to the
             * Pi, flashing. Owned, not on the build list. */
        port usb : USBHostPort;
        port wifi : ~WiFiPort;
    }
    part def PhoneHotspot {
        doc /* Phone Wi-Fi hotspot -- SSH + NTP time sync in the field. */
        port ap : WiFiPort;
    }
    part def GNSSConstellation {
        doc /* GPS / Galileo / BeiDou / GLONASS satellites. */
        port sky : GNSSBroadcastPort;
    }
}
