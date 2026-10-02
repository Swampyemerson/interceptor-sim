// =============================================================================
// SeekerRig.sysml -- the Tier-1 tripod seeker rig: the interceptor's eye on a
// tripod, producing both money-gate curves on the tripod day. The camera, Pi,
// cards and cooler are the SAME physical boards that later move onto the
// interceptor (Interceptor.sysml) -- two configurations of one set of parts.
// =============================================================================
package SeekerRig {
    doc /* Physical architecture of the Tier-1 tripod seeker rig. */

    private import HardwareLibrary::*;
    private import Components::*;

    part def TripodSeekerRig {
        doc /* OV9281 + Pi 5 on a tripod; bench-powered indoors, bank-powered outdoors. */

        part checkerboard : Checkerboard9x6 {
            @BOM { tab = "seeker"; bomName = "Checkerboard 9x6"; }
            @Layout { col = 0; row = 0; }
        }
        part camera : OV9281Camera {
            @BOM { tab = "seeker"; bomName = "OV9281"; }
            @Layout { col = 1; row = 0; }
        }
        part tripod : TripodHead {
            @BOM { tab = "ground"; bomName = "Field layer"; }
            @Layout { col = 2; row = 0; }
        }
        part psu : USBCPSU27W {
            @BOM { tab = "seeker"; bomName = "27 W USB-C PSU"; }
            @Layout { col = 0; row = 1; }
        }
        part pi : RaspberryPi5 {
            @BOM { tab = "seeker"; bomName = "Raspberry Pi 5"; }
            @Layout { col = 1; row = 1; }
        }
        part piSd : MicroSDCard {
            @BOM { tab = "seeker"; bomName = "SanDisk 128 GB"; }
            @Layout { col = 2; row = 1; }
        }
        part powerBank : USBCPowerBank {
            doc /* Same build_tab row as the tripod: the 'field layer' bundle. */
            @BOM { tab = "ground"; bomName = "Field layer"; }
            @Layout { col = 0; row = 2; }
        }
        part cooler : PiActiveCooler {
            @BOM { tab = "seeker"; bomName = "active cooler"; }
            @Layout { col = 1; row = 2; }
        }
        part hotspot : PhoneHotspot {
            @External;
            @Layout { col = 2; row = 2; }
        }
        part laptop : Laptop {
            @External;
            @Layout { col = 3; row = 2; }
        }

        interface benchPower : DCPowerLink connect psu.usbC to pi.pwrIn {
            attribute :>> connector = "USB-C PD";
            attribute :>> detail = "bench rail -- OR the power bank, never both";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-07";
        }
        interface fieldPower : DCPowerLink connect powerBank.usbC to pi.pwrIn {
            attribute :>> connector = "USB-C PD >= 27 W";
            attribute :>> detail = "field rail -- the wall PSU is useless outdoors";
            attribute :>> linkState = LinkState::planned;
        }
        interface fan : FanLink connect pi.fanHeader to cooler.header {
            attribute :>> connector = "4-pin fan header";
            attribute :>> detail = "750 s soaks, no throttling";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-07";
        }
        interface cameraRibbon : CSI2Link connect camera.csi to pi.cam0 {
            @BOM { tab = "seeker"; bomName = "Wonrabai"; }
            attribute :>> connector = "22-to-15-pin CSI ribbon";
            attribute :>> detail = "CAM/DISP0 (not the default port 1)";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-02";
        }
        interface frameCard : SDCardLink connect pi.sdSlot to piSd.contacts {
            attribute :>> connector = "Pi microSD slot";
            attribute :>> detail = "Pi OS + timestamped raw PNG frames";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-01, skr-05";
        }
        interface calibrationView : SightLink connect checkerboard.face to camera.lens {
            attribute :>> connector = "line of sight";
            attribute :>> detail = ">= 15 views, RMS reprojection <= 1.0 px";
            attribute :>> linkState = LinkState::planned;
        }
        interface tripodMount : MountLink connect tripod.head to camera.mount {
            attribute :>> connector = "ball head";
            attribute :>> detail = "inclinometer records the exact tilt";
            attribute :>> linkState = LinkState::planned;
        }
        interface piWifi : WiFiLink connect hotspot.ap to pi.wifi {
            attribute :>> connector = "Wi-Fi";
            attribute :>> detail = "SSH spot-checks between passes + NTP time sync";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-01";
        }
        interface laptopWifi : WiFiLink connect hotspot.ap to laptop.wifi {
            attribute :>> connector = "Wi-Fi";
            attribute :>> detail = "same hotspot as the Pi";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-01";
        }
    }
}
