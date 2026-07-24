# Hardware (WIP)

<p>
  <img src="enclosure-render.png" alt="CAD render: E-stop button over the 16-LED ring, ports in the side pod" width="49%" />
  <img src="real.jpg" alt="Printed unit: yellow lid, red E-stop, RJ45 in the side pod" width="49%" />
</p>

The physical design of the Protective Stop remote lives here: enclosure CAD
(FreeCAD source `casing.FCStd` plus printable exports), the Waveshare
ESP32-S3-ETH carrier board (`ESP32-S3-ETH-Schematic.pdf`, photos, STEP), and
the LED and connector models. CAD render on the left, a printed unit on the
right: the E-stop button sits in the center, the LED ring lights the
diffuser around its base, and the Ethernet and USB-C ports live in the
side pod.

## Pinout and wiring

Two external parts connect to the board: the 16-LED WS2812 ring and a DPST
normally-closed E-stop switch. Each pole of the switch sits in its own
loopback: the firmware drives one GPIO and reads the echo on another, once
per pole, so a broken wire reads the same as a pressed button (STOP). The
GPIO assignments are fixed in the firmware; the wire colors are only an
assembly convention to keep units consistent and easy to check.

| Board pin | Goes to | Wire color |
|---|---|---|
| 5V | Ring 5V | red |
| GND | Ring GND | black |
| GPIO17 | Ring DIN (data) | green |
| GPIO39 | Switch pole 1, terminal A | white |
| GPIO40 | Switch pole 1, terminal B | white |
| GPIO41 | Switch pole 2, terminal A | yellow |
| GPIO42 | Switch pole 2, terminal B | yellow |

```mermaid
flowchart LR
    subgraph BOARD["ESP32-S3-ETH"]
        direction TB
        P5V["5V"]
        GND["GND"]
        G17["GPIO17"]
        G39["GPIO39 drive"]
        G40["GPIO40 sense"]
        G41["GPIO41 drive"]
        G42["GPIO42 sense"]
    end

    subgraph RING["WS2812 ring, 16 LEDs"]
        direction TB
        RV["5V"]
        RG["GND"]
        RD["DIN"]
    end

    subgraph SW["DPST E-stop switch, NC"]
        direction TB
        P1A["pole 1, term A"]
        P1B["pole 1, term B"]
        P2A["pole 2, term A"]
        P2B["pole 2, term B"]
    end

    P5V --- |red| RV
    GND --- |black| RG
    G17 --- |green| RD
    G39 --- |white| P1A
    G40 --- |white| P1B
    G41 --- |yellow| P2A
    G42 --- |yellow| P2B

    linkStyle 0 stroke:#d33,stroke-width:2px
    linkStyle 1 stroke:#333,stroke-width:2px
    linkStyle 2 stroke:#2a2,stroke-width:2px
    linkStyle 3,4 stroke:#aaa,stroke-width:2px
    linkStyle 5,6 stroke:#cc2,stroke-width:2px
```

Notes:

- The switch poles have no polarity; either terminal of a pole can go to
  the drive pin. Keep each color pair on one pole.
- The sense pins are pulled down internally, so an open loop reads 0 and
  the firmware treats it as STOP. Pressing the button opens both poles at
  once; a single open loop (one broken wire) makes the two cores disagree,
  which also stops the machine.
- The ring can be installed in any of its 16 rotations. Which pixel counts
  as LED 1 is a per-device setting, calibrated over the network after
  assembly (`POST /api/ring_offset`, see `../docs/API.md`).
- The board's onboard status LED (GPIO21) needs no wiring.

## Bill of materials

Everything except the printed parts comes off Amazon. The links below are
search links, since specific listings come and go; the specs in the notes
are what matters. When you find a listing that fits, pin it here.

| Qty | Part | Notes | Order |
|---|---|---|---|
| 1 | Waveshare ESP32-S3-ETH | The W5500 Ethernet carrier board this design is built around. The PoE variant powers the unit over the Ethernet cable. | [search](https://www.amazon.com/s?k=waveshare+esp32-s3-eth) |
| 1 | WS2812B ring, 16 pixels | 5V addressable ring. Verify the outer diameter against the lid pocket in `casing.FCStd` before ordering. | [search](https://www.amazon.com/s?k=ws2812b+ring+16+led) |
| 1 | E-stop mushroom button, twist release | Must have two normally-closed contacts (DPST NC / 2NC). A 1NO+1NC unit will not work: both loops need an NC pole. | [search](https://www.amazon.com/s?k=emergency+stop+button+2nc+twist+release) |
| 1 set | Silicone hookup wire | Red, black, green, white, yellow, 24 to 26 AWG, per the wiring table above. | [search](https://www.amazon.com/s?k=silicone+hookup+wire+kit+26awg) |
| ~6 | M3 socket-head screws + brass heat-set inserts | Lid fasteners. Confirm length and count against the current print before buying in bulk. | [search](https://www.amazon.com/s?k=m3+heat+set+insert+socket+head+kit) |
| 1 | Printed enclosure | `print.3mf` in this folder (base, lid, diffuser). Not purchased; any common PLA/PETG works. | n/a |

## Status: work in progress

These files are the current working set, not a finished, reproducible
design package. Some parts exist only as STL/STEP exports rather than
editable source, and there is no assembly guide yet. Still to do:

- editable source for every custom part,
- pinned product listings and verified quantities in the BOM,
- assembly instructions,
- per-file licensing.

OSHWA certification progress is tracked in
[`../docs/OSHWA_COMPLIANCE.md`](../docs/OSHWA_COMPLIANCE.md).
