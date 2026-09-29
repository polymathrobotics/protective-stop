# Hardware

<a href="https://certification.oshwa.org/us002846.html">
  <img src="oshwa-certification-mark.svg" alt="OSHWA open source hardware certification mark, UID US002846" width="320" />
</a>

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

The robot-side machine box (relay box) lives here too: enclosure source
`machine-casing.FCStd` plus printable exports `machine-*.stl`. Its parts
list and build steps are in the machine section of the assembly guide.

Build steps with photos: [ASSEMBLY.md](ASSEMBLY.md).

## Pinout and wiring

Two external parts connect to the board: the 16-LED WS2812 ring and a DPST
normally-closed E-stop button. Each pole of the button sits in its own
loopback: the firmware drives one GPIO and reads the echo on another, once
per pole, so a broken wire reads the same as a pressed button (STOP). The
pin assignments are fixed in the firmware; the wire colors are only an
assembly convention to keep units consistent and easy to check. Pin names
match the board silkscreen (IO17, IO39, and so on).

| Board pin | Goes to | Wire color |
|---|---|---|
| VBUS | Ring 5V | red |
| GND | Ring GND | black |
| IO17 | Ring DIN (data) | green |
| IO39 | Button terminal 11 (pole A) | white |
| IO40 | Button terminal 12 (pole A) | white |
| IO41 | Button terminal 21 (pole B) | yellow |
| IO42 | Button terminal 22 (pole B) | yellow |

```mermaid
flowchart LR
    subgraph BOARD["ESP32-S3-ETH"]
        direction TB
        P5V["VBUS"]
        GND["GND"]
        G17["IO17"]
        G39["IO39 drive"]
        G40["IO40 sense"]
        G41["IO41 drive"]
        G42["IO42 sense"]
    end

    subgraph RING["WS2812 ring, 16 LEDs"]
        direction TB
        RV["5V"]
        RG["GND"]
        RD["DIN"]
    end

    subgraph SW["DPST E-stop button, NC"]
        direction TB
        P1A["terminal 11, pole A"]
        P1B["terminal 12, pole A"]
        P2A["terminal 21, pole B"]
        P2B["terminal 22, pole B"]
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

- The button contacts have no polarity; within a pole, either terminal can
  go to the drive pin. Keep each color pair on one pole: 11/12 together
  (white), 21/22 together (yellow).
- The sense pins are pulled down internally, so an open loop reads 0 and
  the firmware treats it as STOP. Pressing the button opens both poles at
  once; a single open loop (one broken wire) makes the two cores disagree,
  which also stops the machine.
- The ring can be installed in any of its 16 rotations. Which pixel counts
  as LED 1 is a per-device setting, calibrated over the network after
  assembly (`POST /api/ring_offset`, see `../docs/API.md`).
- The board's onboard status LED (IO21) needs no wiring.

## Machine box pinout and wiring

The machine box uses the same board and ring. Instead of a button it drives a
2-channel 5 V relay module, one relay per core, and the relays' normally open
(NO) contacts sit in series in the robot's stop circuit: the robot can run only
while both cores hold their relay closed. The firmware pulls both drive pins
low from the first instructions of boot, so power loss, a reset, a broken
signal wire or a STOP all open the circuit. Build steps are in the
[machine section of the assembly guide](ASSEMBLY.md#machine-side-relay-box).

| Board pin | Goes to | Wire color |
|---|---|---|
| VBUS | Relay module VCC and ring 5V | red |
| GND | Relay module GND and ring GND | black |
| IO17 | Ring DIN (data) | green |
| IO39 | Relay module IN1 (core 0, relay 1) | white |
| IO41 | Relay module IN2 (core 1, relay 2) | yellow |

```mermaid
flowchart LR
    subgraph BOARD["ESP32-S3-ETH"]
        direction TB
        P5V["VBUS"]
        GND["GND"]
        G17["IO17"]
        G39["IO39 core 0"]
        G41["IO41 core 1"]
    end

    subgraph RELAY["2-channel relay module, active-high"]
        direction TB
        RVCC["VCC"]
        RGND["GND"]
        IN1["IN1, relay 1"]
        IN2["IN2, relay 2"]
    end

    subgraph RING["WS2812 ring, 16 LEDs"]
        direction TB
        RV["5V"]
        RG["GND"]
        RD["DIN"]
    end

    P5V --- |red| RVCC
    GND --- |black| RGND
    G39 --- |white| IN1
    G41 --- |yellow| IN2
    P5V --- |red| RV
    GND --- |black| RG
    G17 --- |green| RD

    linkStyle 0,4 stroke:#d33,stroke-width:2px
    linkStyle 1,5 stroke:#333,stroke-width:2px
    linkStyle 2 stroke:#aaa,stroke-width:2px
    linkStyle 3 stroke:#cc2,stroke-width:2px
    linkStyle 6 stroke:#2a2,stroke-width:2px
```

The stop circuit runs through the relay contacts and the Phoenix header in the
port wall, in wire sized for the robot's loop:

| From | To |
|---|---|
| Header pin 1 | Relay 1 COM |
| Relay 1 NO | Relay 2 COM |
| Relay 2 NO | Header pin 2 |

```mermaid
flowchart LR
    A["robot stop circuit"] --- H1["header pin 1"] --- K1["relay 1<br/>COM to NO"] --- K2["relay 2<br/>COM to NO"] --- H2["header pin 2"] --- B["robot stop circuit"]

    linkStyle 0,1,2,3,4 stroke:#36c,stroke-width:3px
```

Notes:

- Use an **active-high** module (IN high = coil energized) and the **NO**
  contacts only. An active-low module, or the NC contacts, would close the stop
  circuit whenever the box loses power or a signal wire breaks. The fail-safe
  check in the assembly guide (step 5) catches both; run it before the box is
  connected to a robot.
- Leave the module's JD-VCC jumper fitted, so the coils run from the board's
  5 V.
- IO40 and IO42 stay unconnected: relay feedback is compiled out
  (`CONFIG_MACHN_RELAY_FEEDBACK`, see `../docs/RELAY_FEEDBACK_DESCOPE.md`).
- The header is rated 12 A / 320 V; the relays and the robot's stop circuit
  set the real limit.
- The ring's LED 1 is calibrated over the network, as on the remote.

## Bill of materials

Everything except the printed parts is ordered online. Listings are
pinned where the exact part matters; the wire and lid hardware are
generic, so those stay as search links. All links open in a new window.

| Qty | Part | Notes | Order |
|---|---|---|---|
| 1 | Waveshare ESP32-S3-ETH | The W5500 Ethernet carrier board this design is built around. For PoE-powered units order the ESP32-S3-POE-ETH variant on the same page; it includes the PoE Module (B) that mounts on the board. | <a href="https://www.waveshare.com/esp32-s3-eth.htm" target="_blank" rel="noopener">Waveshare</a> |
| 1 | DIYmall WS2812B ring, 16 pixels | 5V addressable ring. | <a href="https://www.amazon.com/DIYmall-WS2812B-Integrated-Individually-Addressable/dp/B0B2D5QXG5" target="_blank" rel="noopener">Amazon</a> |
| 1 | NKK Switches FF0126BBCAEA01 E-stop button | Twist-release mushroom, two normally-closed contacts (DPST NC). Rated 100,000 operations (mechanical and electrical life, FF01 datasheet); the firmware counts presses against this in `GET /api/health` (`CONFIG_DCS_BUTTON_RATED_OPS`). The STEP model in this folder is this exact part. Substitutes must have 2NC: both loops need an NC pole, so a 1NO+1NC unit will not work; set the Kconfig rating to the substitute's datasheet value. | <a href="https://octopart.com/part/nkk-switches/FF0126BBCAEA01" target="_blank" rel="noopener">Octopart</a> |
| 1 set | Silicone hookup wire | Red, black, green, white, yellow, 24 to 26 AWG, per the wiring table above. | <a href="https://www.amazon.com/s?k=silicone+hookup+wire+kit+26awg" target="_blank" rel="noopener">search</a> |
| ~6 | M3 socket-head screws + brass heat-set inserts | Lid fasteners. Confirm length and count against the current print before buying in bulk. | <a href="https://www.amazon.com/s?k=m3+heat+set+insert+socket+head+kit" target="_blank" rel="noopener">search</a> |
| 4 | M1.6 heat-set threaded inserts, M1.6 x 5 x 2.5 mm | Anchor the ESP32-S3-ETH board to the base; 2.5 mm is the insert height/depth. | <a href="https://www.amazon.com/s?k=m1.6+heat+set+threaded+insert" target="_blank" rel="noopener">search</a> |
| 3 | 1 oz (28 g) adhesive steel weights | Optional. Ballast in the base floor pockets so the unit stays put on a table. Wheel-weight style, adhesive backed. | <a href="https://www.amazon.com/s?k=1oz+adhesive+wheel+weights+steel" target="_blank" rel="noopener">search</a> |
| 1 | Printed enclosure | `print.3mf` in this folder (base and lid; the diffuser is a white section of the lid print), or the pre-sliced `print.gcode.3mf` for an X1C. Not purchased; any common PLA/PETG works. | n/a |

## Component datasheets

The design contains no custom electronics; the active components are all
off the shelf, with freely accessible datasheets:

- <a href="https://files.waveshare.com/wiki/common/Esp32-s3_datasheet_en.pdf" target="_blank" rel="noopener">ESP32-S3 datasheet</a> (Espressif, via the Waveshare wiki)
- <a href="https://files.waveshare.com/wiki/common/W5500_ds_v110e.pdf" target="_blank" rel="noopener">W5500 Ethernet controller datasheet</a> (WIZnet, via the Waveshare wiki)
- <a href="https://files.waveshare.com/wiki/ESP32-S3-ETH/ESP32-S3-ETH-Schematic.pdf" target="_blank" rel="noopener">ESP32-S3-ETH carrier board schematic</a> (Waveshare)
- <a href="https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf" target="_blank" rel="noopener">WS2812B datasheet</a> (WorldSemi, the ring's LEDs)
- <a href="https://www.nkkswitches.eu/pdf/estop_FF01.pdf" target="_blank" rel="noopener">FF01 series E-stop datasheet</a> (NKK Switches)

## Licensing

The design is CERN-OHL-P-2.0 and the documentation is CC-BY-4.0; the full
texts live in [`../LICENSES/`](../LICENSES/). The vendor reference files
stay the property of their manufacturers and are included only as
fit-check references, with their origins listed below.

| File(s) | License / origin |
|---|---|
| `casing.FCStd`, `base*.stl`, `lid*.stl`, `led.stl`, `print.3mf`, `print.gcode.3mf` | CERN-OHL-P-2.0 (original design) |
| `esp32-S3.step` | CERN-OHL-P-2.0 (original board fit model) |
| `LED.step` | CERN-OHL-P-2.0 (ring model commissioned by Polymath Robotics) |
| `machine-casing.FCStd` (enclosure geometry), `machine-*.stl` | CERN-OHL-P-2.0 (original design) |
| Phoenix Contact header and plug models inside `machine-casing.FCStd` | Phoenix Contact DFK-MSTB 2,5/2-GF-5,08 (0710170) and MSTB 2,5/2-STF-5,08 (1777989) CAD models, from the <a href="https://www.phoenixcontact.com" target="_blank" rel="noopener">Phoenix Contact</a> product pages; included only as fit references |
| `README.md`, `ASSEMBLY.md`, `enclosure-render.png`, `real.jpg`, `assembly/*.jpg`, `assembly/machine-render-*.png` | CC-BY-4.0 (original documentation) |
| `oshwa-certification-mark.svg` | Official OSHWA wide certification mark generated for UID `US002846`; use is governed by the [OSHWA certification agreement](https://certification.oshwa.org/license-agreement.html) |
| `oshw-license-facts.svg` | Generated with the [Open Source Licenses Facts generator](https://oshwa.github.io/certification-mark-generator/facts) for this project's certified license combination |
| `ESP32-S3-ETH-Schematic.pdf` | Waveshare, from the <a href="https://www.waveshare.com/wiki/ESP32-S3-ETH" target="_blank" rel="noopener">product wiki</a> (<a href="https://files.waveshare.com/wiki/ESP32-S3-ETH/ESP32-S3-ETH-Schematic.pdf" target="_blank" rel="noopener">direct PDF</a>) |
| `ESP32-S3-ETH-details-15.jpg` | Waveshare product photo, from the <a href="https://www.waveshare.com/esp32-s3-eth.htm" target="_blank" rel="noopener">product page</a> |
| `FF0126BBCAEA01.stp` | NKK Switches E-stop model, from the <a href="https://www.nkkswitches.com/distributor-landing-page/?part_no=FF0126BBCAEA01&vendor=digikey" target="_blank" rel="noopener">NKK part page</a>; series datasheet: <a href="https://www.nkkswitches.eu/pdf/estop_FF01.pdf" target="_blank" rel="noopener">FF01 (NKK)</a> |

## Design files

The editable source for the custom parts is `casing.FCStd` (remote) and
`machine-casing.FCStd` (machine box); the STL/3MF files are their printable
exports, and the STEP models are references used for fit.
Build steps live in [ASSEMBLY.md](ASSEMBLY.md).

The machine box will move from an off-the-shelf relay module to a custom board
with force-guided relays whose contact state the microcontroller reads back.
Work in progress (schematic done, layout not started): read the
[schematic PDF](relay-board/relay-board-schematic.pdf), then
[relay-board/DESIGN.md](relay-board/DESIGN.md), and
[relay-board/TOOLING.md](relay-board/TOOLING.md) for the tools used and how to
reproduce the environment.

OSHWA certification details and maintenance notes are in
[`../docs/OSHWA_COMPLIANCE.md`](../docs/OSHWA_COMPLIANCE.md).
