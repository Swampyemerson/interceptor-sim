// =============================================================================
// HardwareLibrary.sysml -- the shared vocabulary for every physical model here.
//
// SysML v2 textual notation. Validated by the OMG reference implementation
// (scripts/sysml/check_sysml_official.sh) and rendered to the diagrams in
// docs/sysml/views/ by scripts/render_sysml.py. How to edit: docs/sysml/README.md.
//
// Three ideas carry the whole file:
//   1. A PORT DEF is a type of plug/pad. The side that drives (supplies power,
//      sends DShot, hosts the bus) owns the plain type; the side that receives
//      owns the CONJUGATE, written with a tilde (~). Conjugating a UART port is
//      exactly "TX/RX crossed".
//   2. An INTERFACE DEF is a type of wire or cable between two ports. Every
//      concrete one specialises one or more of the categories below
//      (PowerLink, DataLink, RFLink, MechanicalLink, OpticalLink), and the
//      category decides which diagram it is drawn in.
//   3. STATUS IS NOT STORED HERE. A part carries @BOM pointing at its row in
//      docs/project_state.json (build_tab); the renderer reads the live status
//      (in hand / ordered / built / print / must-add) from the contract, so the
//      model cannot disagree with the build sheet.
// =============================================================================
package HardwareLibrary {
    doc /* Port, interface and annotation types shared by the target, interceptor,
         * seeker-rig and ground-segment models. */

    private import ScalarValues::*;
    private import ISQ::*;
    private import SI::*;

    // ------------------------------------------------------------------ annotations
    metadata def BOM {
        doc /* Ties a part (or a cable) to its row in project_state.json build_tab.
             * tab = the build_tab subsystem id; bomName = a substring that matches
             * exactly one row name in that subsystem. The renderer fails if it
             * matches zero rows or more than one. */
        attribute tab : String;
        attribute bomName : String;
    }

    metadata def NotModelled {
        doc /* A build_tab row deliberately left out of the diagrams (tools, safety
             * gear, consumables, paperwork). Every build_tab row must be either
             * referenced by a @BOM or listed by one of these -- the renderer checks. */
        attribute tab : String;
        attribute bomName : String;
        attribute reason : String;
    }

    metadata def Layout {
        doc /* Grid cell for the diagram renderer (column, row; 0-based). SysML v2
             * carries no layout of its own -- this is the only presentation data
             * kept in the model. */
        attribute col : Integer;
        attribute row : Integer;
    }

    metadata def External {
        doc /* Owned or environmental equipment that is not on the build list
             * (laptop, phone, GNSS constellation). */
    }

    enum def LinkState {
        doc /* How far a connection has got on the real hardware. */
        enum planned;   // designed, not yet made on this hardware
        enum wired;     // physically made, not yet proven by a logged test
        enum verified;  // made AND proven by a logged build step (evidence names it)
    }

    // ------------------------------------------------------------- what flows
    item def ElectricalPower;
    item def SerialData;
    item def MotorCommand;      // DShot frames, FC -> ESC
    item def MotorTelemetry;    // ESC -> FC
    item def PhaseCurrent;      // 3-phase drive, ESC -> motor
    item def SenseSignal;       // analog voltage/current sense
    item def ImageFrames;       // MIPI CSI-2 video
    item def RCCommands;        // stick / switch channels
    item def TagAppearance;     // an AprilTag as seen through a lens
    item def RadioSignal;       // GNSS or other broadcast RF
    item def LogData;           // DataFlash .BIN, ULog, frames on a card
    item def Torque;            // shaft / mount loads

    // ------------------------------------------------------------- port defs
    port def DCPowerPort {
        doc /* A DC rail. Supplier owns DCPowerPort, consumer owns ~DCPowerPort. */
        attribute nominal :> ISQ::voltage;
        out item power : ElectricalPower;
    }
    port def PhasePort {
        doc /* The three motor leads of one ESC channel (ESC side). */
        out item drive : PhaseCurrent;
    }
    port def DShotPort {
        doc /* DShot motor protocol, FC side. */
        out item cmd : MotorCommand;
        in item telem : MotorTelemetry;
    }
    port def AnalogSensePort {
        doc /* Battery voltage / current sense, sensor side. */
        out item sense : SenseSignal;
    }
    port def UARTPort {
        doc /* Full-duplex serial, host side. The peripheral owns ~UARTPort, which
             * is TX/RX crossed by construction. */
        attribute baud : Integer;
        out item tx : SerialData;
        in item rx : SerialData;
    }
    port def I2CPort {
        doc /* I2C bus, controller side. */
        inout item bus : SerialData;
    }
    port def RCOutPort {
        doc /* One-way RC channel stream (SBUS), receiver side. */
        out item channels : RCCommands;
    }
    port def CSI2Port {
        doc /* MIPI CSI-2 camera ribbon, camera side. */
        out item frames : ImageFrames;
    }
    port def SDHostPort {
        doc /* microSD slot, host side. */
        out item log : LogData;
    }
    port def USBHostPort {
        doc /* USB, host side (PC / laptop). The device owns ~USBHostPort. */
        inout item data : SerialData;
        out item vbus : ElectricalPower;
    }
    port def RadioTxPort {
        doc /* 2.4 GHz ELRS RC link, transmitter side. */
        out item rc : RCCommands;
        in item telemetry : SerialData;
    }
    port def WiFiPort {
        doc /* 2.4/5 GHz Wi-Fi, access-point side. */
        inout item packets : SerialData;
    }
    port def GNSSBroadcastPort {
        doc /* Satellite navigation signals, sky side. */
        out item signal : RadioSignal;
    }
    port def TagFacePort {
        doc /* The printed face of an AprilTag / checkerboard, target side. A
             * camera lens owns ~TagFacePort. */
        out item appearance : TagAppearance;
    }
    port def MountPort {
        doc /* A mechanical mounting interface (screws, standoffs, shaft). */
        inout item load : Torque;
    }
    port def ChargePort {
        doc /* Charger output: XT60 + JST-XH balance lead. */
        out item charge : ElectricalPower;
    }

    // composite connectors: one physical plug carrying several signals
    port def StackHarnessPort {
        doc /* 4-in-1 ESC <-> FC 8-pin stack harness, ESC side: battery voltage
             * and current sense up to the FC, DShot down to the ESC. */
        port vbat : DCPowerPort;
        port isense : AnalogSensePort;
        port dshot : ~DShotPort;
    }
    port def GPSConnPort {
        doc /* GPS + compass connector, FC side: UART for fixes, I2C for the
             * compass, 5 V out to the module. */
        port uart : UARTPort;
        port i2c : I2CPort;
        port pwr : DCPowerPort;
    }
    port def PoweredUARTPort {
        doc /* A UART that also feeds 5 V to its peripheral (a CRSF receiver). */
        port uart : UARTPort;
        port pwr : DCPowerPort;
    }
    port def RCInPort {
        doc /* Dedicated RC-input socket, FC side: SBUS in, 5 V out. */
        port sbus : ~RCOutPort;
        port pwr : DCPowerPort;
    }
    port def PowerModulePort {
        doc /* Analog power-module 6-pin lead, module side: regulated 5.2 V and
             * battery V/I sense to the FC. */
        port pwr : DCPowerPort;
        port sense : AnalogSensePort;
    }
    port def BuzzerPort {
        doc /* Lost-model buzzer pads, FC side: 5 V + beeper drive. */
        port pwr : DCPowerPort;
        port beep : AnalogSensePort;
    }
    port def FanHeaderPort {
        doc /* Raspberry Pi 5 4-pin fan header, Pi side. */
        port pwr : DCPowerPort;
        in item tach : SenseSignal;
    }

    // ------------------------------------------------------- interface categories
    abstract interface def Link {
        doc /* Base of every wire, cable, radio path and mount in the models. */
        attribute connector : String;    // physical connector / pads / medium
        attribute detail : String;       // pins, protocol, rate -- one line
        attribute linkState : LinkState;
        attribute evidence : String;     // build-step id(s) behind linkState verified/wired
    }
    abstract interface def PowerLink :> Link;
    abstract interface def DataLink :> Link;
    abstract interface def RFLink :> Link;
    abstract interface def MechanicalLink :> Link;
    abstract interface def OpticalLink :> Link;

    // ------------------------------------------------------- concrete interfaces
    interface def DCPowerLink :> PowerLink {
        end supplier : DCPowerPort;
        end consumer : ~DCPowerPort;
    }
    interface def PhaseLink :> PowerLink {
        end esc : PhasePort;
        end motor : ~PhasePort;
    }
    interface def SenseLink :> DataLink {
        end sensor : AnalogSensePort;
        end reader : ~AnalogSensePort;
    }
    interface def StackHarnessLink :> PowerLink, DataLink {
        end esc : StackHarnessPort;
        end fc : ~StackHarnessPort;
    }
    interface def DShotLink :> DataLink {
        end fc : DShotPort;
        end esc : ~DShotPort;
    }
    interface def UARTLink :> DataLink {
        end host : UARTPort;
        end peripheral : ~UARTPort;
    }
    interface def GPSCableLink :> DataLink, PowerLink {
        end fc : GPSConnPort;
        end module : ~GPSConnPort;
    }
    interface def PoweredUARTLink :> DataLink, PowerLink {
        end fc : PoweredUARTPort;
        end peripheral : ~PoweredUARTPort;
    }
    interface def RCInLink :> DataLink, PowerLink {
        end fc : RCInPort;
        end receiver : ~RCInPort;
    }
    interface def PowerModuleLink :> PowerLink, DataLink {
        end module : PowerModulePort;
        end fc : ~PowerModulePort;
    }
    interface def BuzzerLink :> PowerLink, DataLink {
        end fc : BuzzerPort;
        end buzzer : ~BuzzerPort;
    }
    interface def FanLink :> PowerLink {
        end pi : FanHeaderPort;
        end fan : ~FanHeaderPort;
    }
    interface def CSI2Link :> DataLink {
        end camera : CSI2Port;
        end host : ~CSI2Port;
    }
    interface def SDCardLink :> DataLink {
        end slot : SDHostPort;
        end card : ~SDHostPort;
    }
    interface def USBLink :> DataLink, PowerLink {
        end host : USBHostPort;
        end device : ~USBHostPort;
    }
    interface def ChargeLink :> PowerLink {
        end charger : ChargePort;
        end pack : ~ChargePort;
    }
    interface def ELRSLink :> RFLink {
        end transmitter : RadioTxPort;
        end receiver : ~RadioTxPort;
    }
    interface def WiFiLink :> RFLink {
        end accessPoint : WiFiPort;
        end station : ~WiFiPort;
    }
    interface def GNSSLink :> RFLink {
        end sky : GNSSBroadcastPort;
        end antenna : ~GNSSBroadcastPort;
    }
    interface def SightLink :> OpticalLink {
        end face : TagFacePort;
        end lens : ~TagFacePort;
    }
    interface def MountLink :> MechanicalLink {
        end carrier : MountPort;
        end mounted : ~MountPort;
    }
}
