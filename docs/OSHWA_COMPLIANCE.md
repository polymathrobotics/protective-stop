# OSHWA Certification Compliance Checklist

Tracking progress toward **OSHWA Open Source Hardware certification** for
the Polymath Protective Stop.

- Certification process: <https://certification.oshwa.org/process.html>
- Open Source Hardware definition: <https://www.oshwa.org/definition/>

OSHWA certification is a **self-certification**: the creator attests, via
an online license agreement, that the product meets the community
[OSHW definition](https://www.oshwa.org/definition/), registers each
unique product to receive a **UID** (e.g. `US000123`), and renews yearly.
The bar is that everything **within our control** — hardware design,
firmware, and documentation — is openly licensed and shared in editable
form; third-party components are allowed but must have freely accessible,
shareable datasheets.

## Status legend

- **Met** — satisfies the requirement today.
- **Partial** — some artifacts exist but are incomplete or not in the
  required editable form.
- **TODO** — not yet done.

## Requirements

| # | Requirement (OSHWA) | This repo | Status |
|---|---------------------|-----------|--------|
| 1 | Product meets the OSHW definition (freely usable, modifiable, redistributable, no field-of-use restrictions) | Software is Apache-2.0; hardware/docs licensing not yet formally declared | Partial |
| 2 | **Design files released in the original, editable format** (native CAD, not just exports) | Enclosure has FreeCAD source (`casing.FCStd`) ✅; other parts present mainly as STL/STEP/3MF exports; ensure every custom part has editable source committed | Partial |
| 3 | **Schematics** available and, ideally, in editable EDA source | Only `ESP32-S3-ETH-Schematic.pdf` + a details JPG for the off-the-shelf carrier board; no custom PCB of our own | Partial |
| 4 | **Bill of Materials (BOM)** — parts, quantities, references, sources | Not present | TODO |
| 5 | **Assembly / build instructions** | Not present | TODO |
| 6 | Clearly specify which portion of the design is released under which license | Not yet stated per-directory | TODO |
| 7 | Firmware/software openly licensed (OSI-approved) or interfaces fully documented | Firmware + host are Apache-2.0 (OSI-approved); protocol documented in `docs/` | Met |
| 8 | Third-party components documented with accessible, shareable datasheets | Carrier board is a commercial ESP32-S3-ETH (W5500); datasheet/schematic public but not yet linked/collected | Partial |
| 9 | Documentation licensed (recommended CC-BY 4.0) | No docs license declared yet | TODO |
| 10 | Hardware licensed (recommended CERN-OHL family) | No hardware license declared yet | TODO |
| 11 | Design files hosted in a public, accessible location | Repo is the intended host; must be public and complete | Partial |
| 12 | Register each unique product with OSHWA and obtain a UID | Not started | TODO |
| 13 | Complete the OSHWA license agreement / self-certification | Not started | TODO |
| 14 | Correct use of the OSHWA certification mark + UID marking (on product/docs) once certified | Not started | TODO |

## Recommended license split

OSHWA certification requires a license per element. The conventional,
certification-friendly split is:

- **Hardware** (enclosure CAD, mechanical design, any custom board):
  **CERN-OHL-P-2.0** (permissive) or **CERN-OHL-S-2.0** (strongly
  reciprocal). CERN-OHL-P pairs naturally with the permissive Apache-2.0
  firmware; choose CERN-OHL-S if reciprocal share-alike on hardware is
  wanted.
- **Software / firmware:** **Apache-2.0** — already in place (`LICENSE`).
- **Documentation:** **CC-BY-4.0** (attribution).

Recommendation: **CERN-OHL-P-2.0 (hardware) + Apache-2.0 (software) +
CC-BY-4.0 (docs)** for a coherent permissive stack. Add a top-level note
(e.g. in `README.md` or a `LICENSES/` set) stating which license governs
which directory (requirement #6).

## What's met vs. what remains

**Met / close:**

- Software licensing (Apache-2.0, OSI-approved) — requirement #7.
- An editable enclosure source exists (`casing.FCStd`) — good progress on
  #2.
- Protocol/interface documentation exists in `docs/`.

**Remaining before applying:**

1. Finish and clean the hardware design (`hardware/` is WIP — see
   `hardware/README.md`); ensure **every custom part ships editable
   source**, not only STL/STEP exports (#2, #3).
2. Author a complete **BOM** and **assembly instructions** (#4, #5).
3. Declare the **hardware** and **documentation** licenses and add a
   per-directory license statement (#1, #6, #9, #10).
4. Collect/link **datasheets** for the commercial carrier board and any
   third-party parts (#8).
5. Make the repository fully **public and complete** (#11).
6. Complete OSHWA **self-certification**, register the product for a
   **UID**, and apply the **certification mark** to product + docs
   (#12–#14).

## Next steps (ordered)

1. Land the hardware cleanup: editable CAD/EDA sources, BOM, assembly
   docs.
2. Adopt the license split above; add `LICENSES/` and per-directory
   notices.
3. Verify the whole design meets the OSHW definition end-to-end.
4. Self-certify at <https://certification.oshwa.org/process.html>,
   register for a UID, and mark the product.

_This checklist is a living document; update the status column as
hardware and licensing work lands._
