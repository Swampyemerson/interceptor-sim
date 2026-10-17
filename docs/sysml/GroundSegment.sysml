// =============================================================================
// GroundSegment.sysml -- what stays on the ground: one RC transmitter PER
// AIRCRAFT (each aircraft keeps an independent kill, ADR-0089), the charger,
// the laptop and the phone hotspot. Engagement.sysml wires these to the two
// aircraft. Safety gear and tools are on the build sheet but are not wired to
// anything, so they are listed below as @NotModelled rather than drawn.
// =============================================================================
package GroundSegment {
    doc /* Physical architecture of the ground segment, plus the register of
         * build-sheet rows that are deliberately not drawn. */

    private import HardwareLibrary::*;
    private import Components::*;

    part def Ground {
        doc /* Two operators, two transmitters, one laptop. */

        part cells : Cells18650Pair {
            @BOM { tab = "ground"; bomName = "18650"; }
            @Layout { col = 0; row = 0; }
        }
        part pocketTarget : PocketTX {
            doc /* Flies the target; its kill. */
            @BOM { tab = "ground"; bomName = "Pocket M2 ELRS TX"; }
            @Layout { col = 1; row = 0; }
        }
        part pocketInterceptor : PocketTX {
            doc /* The interceptor's independent arm/kill (ordered 2026-09-22).
                 * Its power source is NOT on the build sheet -- check the cell
                 * type its battery bay takes before the first bind. */
            @BOM { tab = "ground"; bomName = "2nd RadioMaster Pocket"; }
            @Layout { col = 1; row = 1; }
        }
        part charger : BalanceCharger1to6S {
            @BOM { tab = "ground"; bomName = "balance charger"; }
            @Layout { col = 0; row = 2; }
        }
        part laptop : Laptop {
            @External;
            @Layout { col = 1; row = 2; }
        }
        part hotspot : PhoneHotspot {
            @External;
            @Layout { col = 2; row = 2; }
        }

        interface txPower : DCPowerLink connect cells.output to pocketTarget.cells {
            attribute :>> connector = "battery bay";
            attribute :>> detail = "2 x 18650";
            attribute :>> linkState = LinkState::planned;
        }
        interface laptopWifi : WiFiLink connect hotspot.ap to laptop.wifi {
            attribute :>> connector = "Wi-Fi";
            attribute :>> detail = "SSH to the Pi; QGC over Pi Wi-Fi";
            attribute :>> linkState = LinkState::verified;
            attribute :>> evidence = "skr-01";
        }
    }

    // ---------------------------------------------------------------------------
    // Build-sheet rows that are real and in use but have no wire to draw.
    // ---------------------------------------------------------------------------
    @NotModelled { tab = "ground"; bomName = "LiPo safety set"; reason = "safety gear"; }
    @NotModelled { tab = "ground"; bomName = "Smoke stopper"; reason = "bench tool: inline only at first power-up after soldering"; }
    @NotModelled { tab = "ground"; bomName = "Solder/build layer"; reason = "build consumables"; }
    @NotModelled { tab = "ground"; bomName = "Tool layer"; reason = "tools"; }
    @NotModelled { tab = "ground"; bomName = "Regulatory + site"; reason = "paperwork, closed"; }
    @NotModelled { tab = "brain"; bomName = "Holybro PM02 V3 power module"; reason = "first unit, vendor-refunded; the flying PM02 is the airframe-tab replacement"; }
    @NotModelled { tab = "airframe"; bomName = "Pixhawk 6C Mini + M10 GPS"; reason = "duplicate row: the boards are modelled from the brain tab"; }
    @NotModelled { tab = "airframe"; bomName = "DIY thrust rig"; reason = "bench test rig, not flight hardware"; }
}
