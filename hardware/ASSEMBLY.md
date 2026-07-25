# Assembly guide

Building one Protective Stop remote takes about an hour. Order the parts
from the [BOM](README.md#bill-of-materials) and print the enclosure
(`print.3mf`: base, lid, diffuser) first. Wire colors and pin names follow
the [wiring table](README.md#pinout-and-wiring).

Tools: soldering iron (wiring and heat-set inserts), solder, flush
cutters, 2.5 mm hex driver, wire stripper.

## 1. Prepare the base

![Bare base next to a base with inserts and ballast installed](assembly/step-01-inserts-and-ballast.jpg)

Press the brass heat-set inserts into the base with the soldering iron:
M3 inserts into the lid-screw bosses, M1.6 inserts into the board anchor
holes in the side pod. Stick the three 1 oz steel weights into the floor
pockets. The weights are ballast so the unit stays put on a table instead
of following the Ethernet cable around.

## 2. Mount the button and ring in the lid

![Lid underside with the E-stop button and LED ring installed](assembly/step-02-button-and-ring-in-lid.jpg)

Set the diffuser into the lid opening. Push the E-stop button through the
center hole from the top and lock it with its collar from below. Seat the
LED ring around the button body, LEDs facing the diffuser. Rotation does
not matter: which pixel counts as LED 1 is calibrated over the network
after assembly.

## 3. Seat the board and solder the harness

![ESP32 board seated in the side pod with wires soldered](assembly/step-03-board-in-pod.jpg)

If this unit is PoE powered, fit the Waveshare PoE Module (B) onto the
board first. Drop the board into the pod channel, connectors facing out,
and anchor it with the M1.6 screws. Solder the harness to the labeled
pads, leaving enough slack to open the lid comfortably:

| Wire | Board pad |
|---|---|
| red | VBUS |
| black | GND |
| green | IO17 |
| white x2 | IO39, IO40 |
| yellow x2 | IO41, IO42 |

## 4. Wire the lid

![Lid and base joined by the harness](assembly/step-04-lid-wiring.jpg)

Solder the other ends. The white pair goes to button terminals 11 and 12,
the yellow pair to 21 and 22 (within a pair, either wire on either
terminal). On the ring: red to PWR5V, black to GND, green to DI. Mind the
DI/DO marking; data only enters at DI.

## 5. Close the lid

![Lid screwed down with the 2.5 mm hex driver](assembly/step-05-close-the-lid.jpg)

Fold the harness into the base so nothing sits on top of the board, seat
the lid, and drive the M3 screws into the inserts with the 2.5 mm hex
driver. Snug is enough; the inserts strip before the screws do.

## 6. Commission

First flash goes over the wire (hold BOOT, tap RST); everything after is
over the network. See the [quickstart](../README.md#quickstart) for the
flash and machine-pairing steps, then calibrate the ring rotation:
`POST /api/ring_led1?on=1` lights the pixel the firmware currently calls
LED 1, and `POST /api/ring_offset?n=0..15` moves it to where the bezel
says LED 1 should be ([API reference](../docs/API.md)).
