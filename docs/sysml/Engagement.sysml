// =============================================================================
// Engagement.sysml -- the top-level context: the two aircraft, the ground
// segment and the sky, and every link that crosses between them. Chains like
// `target.rx.radio` reach inside a system to the exact port.
//
// The one link that matters most is `tagSight`: the target's placard seen by
// the interceptor's camera. After launch it is the ONLY input from the target
// the interceptor gets (no datalink) -- the honesty boundary drawn as a wire.
// =============================================================================
package Engagement {
    doc /* System context for a tripod day / kill attempt. */

    private import HardwareLibrary::*;
    private import Components::*;
    private import Target::*;
    private import Interceptor::*;
    private import GroundSegment::*;

    part def EngagementContext {
        part sky : GNSSConstellation {
            @External;
            @Layout { col = 1; row = 0; }
        }
        part target : TargetDrone {
            @Layout { col = 0; row = 1; }
        }
        part interceptor : InterceptorDrone {
            @Layout { col = 2; row = 1; }
        }
        part ground : Ground {
            @Layout { col = 1; row = 2; }
        }

        interface rcTarget : ELRSLink connect ground.pocketTarget.radio to target.rx.radio {
            attribute :>> connector = "ELRS 2.4 GHz";
            attribute :>> detail = "full RC on the target: Loiter / AUTO legs / kill";
            attribute :>> linkState = LinkState::planned;
        }
        interface rcInterceptor : ELRSLink connect ground.pocketInterceptor.radio to interceptor.rx.radio {
            attribute :>> connector = "ELRS 2.4 GHz";
            attribute :>> detail = "arm / GO / kill ONLY -- no guidance data (constraint no-datalink)";
            attribute :>> linkState = LinkState::planned;
        }
        interface tagSight : SightLink connect target.placard.face to interceptor.camera.lens {
            attribute :>> connector = "line of sight";
            attribute :>> detail = "the seeker's only measurement of the target after launch";
            attribute :>> linkState = LinkState::planned;
        }
        interface piWifi : WiFiLink connect ground.hotspot.ap to interceptor.pi.wifi {
            attribute :>> connector = "Wi-Fi";
            attribute :>> detail = "pre-flight SSH + NTP only; not used in flight";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-01";
        }
        interface gnssTarget : GNSSLink connect sky.sky to target.gps.antenna {
            attribute :>> connector = "L1 GNSS";
            attribute :>> detail = "3D fix, 11 satellites, indoors on the bench";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "tgt-02";
        }
        interface gnssInterceptor : GNSSLink connect sky.sky to interceptor.gps.antenna {
            attribute :>> connector = "L1 GNSS";
            attribute :>> detail = "position hold + failsafe only; guidance never reads GPS of the target";
            attribute :>> linkState = LinkState::planned;
        }
        interface chargeTarget : ChargeLink connect ground.charger.output to target.battery.balance {
            attribute :>> connector = "XT60 + JST-XH balance";
            attribute :>> detail = "in the fireproof bag, attended";
            attribute :>> linkState = LinkState::planned;
        }
        interface chargeInterceptor : ChargeLink connect ground.charger.output to interceptor.battery.balance {
            attribute :>> connector = "XT60 + JST-XH balance";
            attribute :>> detail = "same pack pool";
            attribute :>> linkState = LinkState::planned;
        }
        interface usbInterceptor : USBLink connect ground.laptop.usb to interceptor.fc.usbC {
            @BOM { tab = "brain"; bomName = "USB-C DATA cable"; }
            attribute :>> connector = "USB-C data cable";
            attribute :>> detail = "flash PX4 + params, QGC (bench)";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "brn-01";
        }
        interface usbTarget : USBLink connect ground.laptop.usb to target.fc.usbC {
            @BOM { tab = "brain"; bomName = "USB-C DATA cable"; }
            attribute :>> connector = "USB-C data cable";
            attribute :>> detail = "DFU flash KakuteH7, param upload, .BIN pull";
            attribute :>> linkState = LinkState::planned;
        }
    }
}
