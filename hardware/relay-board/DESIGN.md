# Machine relay board: design record

Status: **schematic complete and verified; PCB layout not started.** 2026-09-29.
Nothing here is a functional-safety certification. SIL 3 is the design target;
it is a property of the whole safety function and is **not claimed** until the
FMEDA is recomputed with manufacturer data (section 8, item 10).

**Read the schematic first: [relay-board-schematic.pdf](relay-board-schematic.pdf)**
(one A3 page; regenerate with `tools/export_pdf.sh` whenever the `.kicad_sch` changes).

This board replaces the off-the-shelf 2-channel relay module in the machine box
(`../machine-casing.FCStd`, `../ASSEMBLY.md` "Machine side"). It keeps the same
footprint and the same firmware pins, and adds what the module could not:
force-guided relays whose state the microcontroller can read back.

## 1. Requirements

| Requirement | Value |
|---|---|
| Stop loop | up to 24 V DC, up to 500 mA, **inductive** (contactor or relay coil) |
| Topology | two independent relays, contacts in series in the loop; either alone stops the robot |
| Relay type | force-guided, EN 61810-3 type A, 5 V coil |
| Fail state | de-energized = loop open. Power loss, reset, floating GPIO, broken wire: all open |
| Monitoring | each relay's real contact state readable by the ESP32 (per channel) |
| Fit | machine box relay space: 51 x 39 x 19 mm, M3 pattern 44.86 x 33.57 mm |
| Manufacture | JLCPCB PCB and SMT assembly; relays consigned; Phoenix header off-board, wired |

## 2. Architecture (per channel; channel B is identical)

```
+5V --- K.A1  [coil]  K.A2 --- drain Q (AO3400A) --- source GND
                              |
                              +--- D (SMAJ12A, clamp to GND)   fast release
IO39 --- R1 100R --- gate Q ---+--- R3 10k --- GND               fail-off

loop:  LOOP_IN -- K1 NO 34-33 -- NO 43-44 -- LOOP_MID -- K2 NO 34-33 -- NO 43-44 -- LOOP_OUT

mirror: 3V3 -- R5 330R --+-- K NC 21-22 -- NC 12-11 -- GND
                         +-- R7 100R -- IO40 (sense)
```

- **Drive.** Low-side N-MOSFET, active-high, gate pulled down. Firmware behaviour is
  unchanged: energized = run.
- **Loop path.** Two NO contacts in series inside each relay, then the two
  relays in series: four gaps. This splits the DC arc for the inductive load
  (NO contact rated DC13 3 A against a 0.5 A loop) and means one welded contact
  never defeats a relay.
- **Mirror.** The two NC contacts of the same relay in series, tied to GND, with a
  pull-up. Force-guided contacts keep every NC contact at least 0.5 mm open while
  any NO contact is welded, so the sense line reads "open" only if **both** NO
  contacts released.
- **Sense levels** match what the firmware already assumes: `1` = commanded
  energized and NO closed, `0` = de-energized and NC closed. About 10 mA flows
  through the NC contacts (datasheet minimum load 5 V / 10 mA), which keeps them
  wetted.

| Commanded | Loop contacts | NC mirror | SNS pin | Meaning |
|---|---|---|---|---|
| 0 | open | closed | 0 | healthy stopped |
| 1 | closed | open | 1 | healthy running |
| 0 | welded or coil still driven | open | 1 | **dangerous**: partner relay carries the stop, raise fault |
| 1 | not closed | closed | 0 | safe: relay did not pull in, availability fault |

The check runs every time a relay changes state, so a stuck or broken sense line
is found on the next transition. **Every re-arm should first require both SNS = 0**
(both relays confirmed released) before energizing.

## 3. Interfaces

| J1 pad | Net | ESP32-S3-ETH pin |
|---|---|---|
| 1 | +5V | VBUS (left header, pin 40) |
| 2 | +3V3 | 3V3 (left header, pin 36) |
| 3 | DRV_A | IO39 (right header pin 12) |
| 4 | SNS_A | IO40 (pin 11) |
| 5 | DRV_B | IO41 (pin 10) |
| 6 | SNS_B | IO42 (pin 9) |
| 7 | GND | GND (right header pin 8 or 13) |

The pad order on J1 is chosen for the schematic (power pins at the ends), not to
match the ESP32 header; the wires cross over freely.

J2 (LOOP IN) and J3 (LOOP OUT) are single solder pads for wires to Phoenix header
pins 1 and 2, sized for the loop. Pads only, no headers fitted. Channel A is
core 0, channel B is core 1.

## 3a. Reading the schematic

One A3 sheet ([PDF copy](relay-board-schematic.pdf) for reviewers), laid out to the usual conventions: signal flow left to right, positive
rails at the top pointing up, GND at the bottom pointing down, one dashed frame per
function, wires for local connections, and labels only where a signal crosses a
frame (DRV_x, SNS_x, and the three loop-net names).

- **J1** (left): the wire pads to the ESP32-S3-ETH, rail test points, bulk capacitor.
- **CHANNEL A / B** (middle): coil drive on the left of each frame, contact mirror on
  the right. The two channels are drawn identically.
- **STOP LOOP** (bottom): the four loop contacts drawn as a left-to-right series
  chain, in the released state.
- **HOW IT WORKS / MECHANICAL** (right): fail-safe summary, the SNS meaning table,
  and the mounting-hole pattern.

The relay is a five-unit symbol in the IEC split-symbol style, each unit drawn
where it acts: **A** coil, **B** NC 11-12 and **C** NC 21-22 (mirror), **D** NO
33-34 and **E** NO 43-44 (loop). So `K1A`..`K1E` are one physical part, and the
reference on each unit says which contact it is.

## 4. Safety design rules (enforced by `tools/check_netlist.py`)

1. **Nothing bridges the loop contacts.** No TVS, capacitor, snubber, resistor or
   LED from LOOP_IN, LOOP_MID or LOOP_OUT to anything else. Such parts fail
   short in their dominant failure mode and would bypass the relays. Only the
   relay contacts, the J2 and J3 wire pads and one open test pad touch these nets.
   Inductive kick belongs at the **load**: the integrator fits a flyback diode
   or TVS across the contactor or relay coil being switched.
2. Only NO contacts (33-34, 43-44) carry the loop.
3. Each gate has a pull-down, so a floating or resetting GPIO cannot energize a relay.
4. The coil is low-side driven from +5V, and the mirror uses NC contacts 11/12/21/22 only.
5. The two channels share only +5V, GND and the loop chain itself.

## 5. Mechanical (from `machine-casing.FCStd`, case coordinates in mm)

- Envelope X 1.5..52.5, Y -41.2..-2.2, Z 4.5..23.5. Lid ceiling is at least Z 24.2 over the whole area.
- M3 standoffs at (4.57, -4.91), (49.43, -4.91), (4.57, -38.48), (49.43, -38.48).
- SR4D4005 is 40 x 13 x 16.5 mm. On a 1.6 mm board its top is Z 22.6, leaving 1.6 mm.
- Phoenix header pins reach about X 6.5 at Z 10, near Y -22.8: keep tall parts out of that zone.
- Two relays (13 mm wide) side by side need about 27.6 mm of the 38 mm depth once the
  M3 screw-head clearance at the four corners is respected. Expect only about
  2 mm between relays. Heat is the reason to revisit this (section 8).
- Suggested board: 50 x 38 mm, mounting holes at the four standoff positions,
  origin = (case X - 2.0, -case Y - 2.7).

## 6. Bill of materials

Verified against jlcpcb.com / lcsc.com pages on 2026-09-29. LCSC stock changes daily.

| Ref | Part | LCSC | Note |
|---|---|---|---|
| K1, K2 | TE SR4D4005 | C1525104 | **Consigned.** Out of stock at LCSC on 2026-09-29 (about $19 each at 1 pc). Buy from DigiKey, Mouser or TE |
| Q1, Q2 | AO3400A SOT-23 | C20917 | basic |
| D1, D2 | SMAJ12A (Littelfuse) | C148213 | 12 V standoff, 19.9 V clamp, well under the 30 V MOSFET |
| R1, R2, R7, R8 | 100 R 0603 | C22775 | basic |
| R3, R4 | 10 k 0603 | C25804 | basic |
| R5, R6 | 330 R 0603 | C23138 | basic |
| C1 | 22 uF 0805 | C45783 | basic |

Do not trust part matching by value in the MCP's `lib_search_components`: it matched
"10k 0603" to a 510 k part. Every number above was checked on the vendor page.

## 7. Verification done

- ERC: clean, 0 findings (KiCad 10.0.0 via kicad-mcp-pro).
- Readability: kicad-mcp-pro `sch_cosmetic_score` 100 / 100, no findings (the first,
  label-only version of this sheet scored 32 and was unreadable).
- `tools/check_netlist.py`: KiCad's own netlist export matches the intended 20 nets
  by pin membership, labelled nets keep their names, and rules 1-5 hold.
  `--selftest` injects 9 faults (bridged loop, NC on the loop, removed pull-down,
  shorted coil, renamed label and others): all caught.
- SR4D4005 symbol and footprint parse in KiCad 10. Footprint geometry re-derived
  from TE drawing S0413-BC (bottom view, mirrored to top view). **Not yet checked
  against a physical relay.**

## 8. Risks and open items

| # | Item | Why it matters |
|---|---|---|
| 1 | **B10d and failure rates from TE** for the SR4D4005 at 24 V DC inductive, 0.5 A | TE states 99% diagnostic coverage for the monitored contact and gives B10d only on request. The FMEDA needs it |
| 2 | **Heat.** Each 5 V coil is 806 mW (161 mA), so 1.6 W in a small closed box | The SR4 is rated -25..+70 C; PLA softens near 55 C. Measure, consider PETG or ASA, or an economizer |
| 3 | **USB power.** Both coils draw about 322 mA on top of the board and ring | Fine on PoE; likely over a 500 mA USB budget |
| 4 | **Footprint** vs a physical sample before ordering boards | Pin mapping was read from a drawing |
| 5 | **Firmware.** Re-enable `CONFIG_MACHN_RELAY_FEEDBACK` and change `expected[ch]` to the channel's own command; the old code expected `A AND B` on channel B | Otherwise a false fault whenever A is open and B closed |
| 6 | **3V3 dependence.** The mirror is powered from the ESP32 board's 3V3 (about 20 mA) | 3V3 loss reads as SNS = 0; while commanded energized that is a fault (safe) |
| 7 | **Creepage.** TE: contact to coil 10 mm, adjacent contacts 3 / 3.5 mm | Adequate for a 24 V loop; keep coil and loop copper apart on the PCB |
| 8 | **JLC assembly of consigned through-hole relays** | Confirm with JLC; fallback is hand-soldering |
| 9 | Optional: sense loop voltage at J2 | Not done: needs the loop's supply and polarity, and any input must never bridge the contacts |
| 10 | **FMEDA recompute** and safety-owner sign-off | The existing safety documents mark this as stale for the machine channel |

## 9. Next steps

Roughly in order; items in the same phase can run in parallel. "Blocked by" is what
has to happen first.

**Phase 1: data and parts (start now, long lead times)**

1. Ask TE for the SR4D4005 **B10d and failure-rate data** (24 V DC inductive, 0.5 A)
   and the DC13 rating curve. Blocks the FMEDA (phase 4).
2. Buy 4 to 6 SR4D4005 from DigiKey, Mouser or TE (LCSC is out of stock). Confirm with
   JLCPCB that they will assemble **consigned through-hole** relays; if not, plan to
   hand-solder K1 and K2.
3. Check the **footprint against a physical relay** before any board is ordered.
4. Confirm the loop spec with the integrator: 24 V DC max, 500 mA max, the load really
   is inductive, and a flyback diode or TVS is fitted **at the load**.

**Phase 2: PCB layout** (needs the KiCad GUI with the IPC API enabled: Preferences,
Plugins; blocked by nothing)

1. Import the schematic, set the outline to 50 x 38 mm with the four M3 holes from
   section 5, and place K1 and K2 first (40 x 13 mm each, 5 mm clear between them if the
   screw-head keep-outs allow).
2. Keep-outs: 5.5 mm screw-head circles at the corners, the Phoenix pin zone
   (X <= 7, Y about -23, tall parts only above 8 mm), 16.5 mm height limit.
3. Net classes: LOOP nets 0.5 mm minimum clearance from logic and coil copper (the
   relay itself gives 3 to 10 mm), wider tracks for LOOP and +5V, GND pour on the logic side.
4. Run DRC, then export STEP and check it against `machine-casing.FCStd` in FreeCAD.
5. JLCPCB DFM check and a BOM/CPL export with the LCSC numbers from section 6.

**Phase 3: firmware** (blocked by nothing; can start before the board exists)

1. Set `CONFIG_MACHN_RELAY_FEEDBACK=y` and change `expected[ch]` to the channel's own
   command (section 2 and risk 5). The existing sense pin setup in `relay_gpio_init()`
   (input, internal pull-down, IO40 and IO42) works unchanged: the pull-down is a
   negligible divider against the 330 ohm pull-up, and it makes a broken sense wire
   read 0, which is a fault whenever the relay is commanded on.
2. Require both SNS = 0 before a re-arm; treat commanded-0 with SNS = 1 as a weld fault.
3. Add HIL tests for each row of the state table in section 2, and update
    `docs/RELAY_FEEDBACK_DESCOPE.md` to say feedback is restored and how.

**Phase 4: safety case** (blocked by 1 and 11)

1. Recompute the FMEDA for the machine channel with real B10d, restore the feedback
   diagnostic credit (MC-2, MC-3), update `SR-SYS-05` and `docs/safety/OPEN_ITEMS.md`.
2. Define the proof-test interval and demand rate; safety-owner sign-off.

**Phase 5: bring-up** (blocked by 2, 3 and boards in hand)

1. Fail-safe continuity check from `../ASSEMBLY.md` step 5, then a weld simulation
   (bridge one NO contact on a bench sample) to confirm the mirror reads it.
2. Measure release time with the clamp fitted, coil temperature and box temperature in a
    soak, and USB versus PoE supply margin. Decide PLA versus PETG or ASA, or an economizer.

**Phase 6: documentation**

1. Update `../README.md` (BOM, wiring) and `../ASSEMBLY.md` for the new board: J1, J2 and J3
   wiring, relay mounting, consigned part sourcing.
2. Cosmetic: the layout of this schematic is fine but has not been reviewed by a second person.

## 10. Tooling notes

- KiCad 10.0.0 AppImage; `kicad-mcp-pro` 3.35.2 (MIT) drives schematic capture and checks.
- Project libraries are self-contained (`relay-board.kicad_sym`, `power.kicad_sym`,
  `relay-board.pretty`), copied from KiCad's stock libraries plus the SR4D4005.
- The sheet was laid out by a script and then hand-checked; from here the `.kicad_sch`
  is the source of truth, so edit it in KiCad. Re-run `tools/check_netlist.py` after any change.
- PCB placement and routing need the KiCad GUI with its IPC API enabled; the MCP
  cannot do them headless.
- `relay-board-schematic.pdf` is a committed export of the schematic for human review; it
  goes stale silently, so run `tools/export_pdf.sh` and commit it with any schematic change.
- Regenerate checks with: `python3 tools/check_netlist.py --kicad-cli <path>`.
- Full tool list, MCP install and vetting notes, and how to set this up on another
  computer: [TOOLING.md](TOOLING.md).

## 11. Datasheets

- TE SCHRACK SR4: https://www.te.com/en/product-CAT-SCH691-SR1A.html (drawing S0413-BC / S0413-BB)
- Panasonic safety relay B10d note (for comparison): https://industry.panasonic.eu/storage/imported/industrial.panasonic.com/ac/cdn/e/control/catalog/unconfirm/mech_eng_machinesafety.pdf
- ESP32-S3-ETH pinout: `../ESP32-S3-ETH-details-15.jpg`
