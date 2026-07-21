# Hardware (WIP)

The physical design of the Protective Stop remote lives here:

- **Enclosure CAD** — FreeCAD source (`casing.FCStd`) plus printable
  exports (`base*.stl`/`.3mf`, `lid*.stl`, `led*.stl`/`.3mf`).
- **Carrier board** — Waveshare-style **ESP32-S3-ETH** (W5500 Ethernet):
  `ESP32-S3-ETH-Schematic.pdf`, `ESP32-S3-ETH-details-15.jpg`,
  `esp32.step`.
- **LED / lens / connector** — `LED.step`, `led.stl`,
  `Part Studio 2 - LED Lens.step`, `JACK-USBC-6SMT-2SL.stp`, and other
  STEP models.

> **Status: messy / work in progress.** These files are the current
> working set, not a finished, reproducible design package. Some parts are
> present only as STL/STEP exports rather than editable source, and there
> is no BOM or assembly guide yet.

This will be **cleaned up and completed ASAP** with:

- editable source for every custom part (not just exports),
- a full **bill of materials** (parts, quantities, references, sources),
- **assembly / build instructions**,
- clear per-file licensing.

OSHWA-certification-grade documentation progress is tracked in
[`../docs/OSHWA_COMPLIANCE.md`](../docs/OSHWA_COMPLIANCE.md).
