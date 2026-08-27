# OSHWA certification

Protective Stop is certified open source hardware through the Open Source
Hardware Association certification program.

| Field | Value |
|---|---|
| Project | Protective Stop |
| OSHWA UID | [`US002846`](https://certification.oshwa.org/us002846.html) |
| Certified version | v1.2 |
| Certification date | August 27, 2026 |
| Responsible party | Open Source Safety Consortium |
| Hardware license | CERN-OHL-P-2.0 |
| Software license | Apache-2.0 |
| Documentation license | CC-BY-4.0 |

OSHWA certification confirms that the design meets the Open Source Hardware
Definition and that the source needed to study, modify, make, and redistribute
the hardware is available under open licenses. It is not a functional-safety
certification and does not by itself establish SIL or PL performance.

## Published source

The certified source package is maintained in this public repository:

- Editable enclosure source and exports: [`hardware/`](../hardware/)
- Bill of materials and wiring: [`hardware/README.md`](../hardware/README.md)
- Photographed build guide: [`hardware/ASSEMBLY.md`](../hardware/ASSEMBLY.md)
- Firmware and host software: [`firmware/`](../firmware/) and [`host/`](../host/)
- Design, API, testing, and safety documentation: [`docs/`](./)
- License texts: [`LICENSE`](../LICENSE) and [`LICENSES/`](../LICENSES/)

The design uses an off-the-shelf ESP32-S3 Ethernet carrier, LED ring, and DPST
normally closed stop switch. Their source links and freely accessible
datasheets are listed in [`hardware/README.md`](../hardware/README.md).

## Certification mark

The OSHW certification mark may be used with UID `US002846` on the enclosure,
documentation, packaging, and project material. Follow the official
[mark usage guidelines](https://certification.oshwa.org/mark-usage.html) and use
the artwork generated from the
[registry entry](https://certification.oshwa.org/us002846.html).

OSHWA's approved plaintext form is:

```text
[OSHW] US002846 | Certified open source hardware | oshwa.org/cert
```

## Maintenance

Keep the registry entry and this repository aligned when the certified hardware
version, responsible party, project URL, or license mapping changes. Network
heartbeat timing should be described as machine-controlled rather than as a
fixed update frequency.
