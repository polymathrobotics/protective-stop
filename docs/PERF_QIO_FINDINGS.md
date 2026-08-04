# Performance findings — opt level & flash mode (perf branch)

Measured on-device, ESP32-S3-ETH (Waveshare) @ 240 MHz, via /api/perf.

## Adopted: app compiled at -O2 (was -Og debug)  — ~2x on hot paths

|             | X25519    | AEAD 1420B      | flash code |
|-------------|-----------|-----------------|-----------|
| -Og (base)  | 28.3 ms   | 774 us / 14 Mbps| 910 KB    |
| -Os         | 24.9 ms   | 678 us / 16 Mbps| 813 KB    | (1.1x — size-opt helps cold cache, not our hot loops)
| **-O2**     | **15.5 ms** | **381 us / 29 Mbps** | 883 KB | **(1.8-2.0x, chosen)** |

-O2 accelerates the perf-critical hot paths (WG/disco crypto, the 100 ms
lockstep comparator, packet processing) and came out SMALLER than -Og
while fitting IRAM with headroom.

## Rejected: QIO flash mode — blocked by a CLONE flash chip

Investigated per the schematic (correct order): U4 is a W25Q128, and the
board IS fully wired for quad — IO0-IO3 all on the ESP32-S3 dedicated
flash bus (SPID/SPIQ/SPIWP/SPIHD + SPICLK/SPICS0). eFuse reports quad;
the QE (Quad Enable) status bit reads SET (SR 0x0200).

BUT the actual part is **manufacturer 0x20, device 0x4018 — a clone**,
not genuine Winbond (0xEF). ESP-IDF has no vendor driver for 0x20, so it
falls to the generic quad path, which produces NON-FUNCTIONAL QIO reads
on this part: a full QIO flash **crash-loops at both 40 MHz and 80 MHz**,
while DIO@80 is rock-solid. (d7eed0 recovered each time via LC-relay
power-cycle + DIO reflash over USB.)

QIO would give ~30-50% on XIP (cold-path) execution IF it worked, but:
- it does not work on this board's clone flash without custom
  flash-driver work (high risk on a safety device), and
- boot time is network-bound (eth link + DHCP + tailnet registration),
  identical across -Og/-O2/QIO — so QIO gives ZERO boot benefit; its
  value is only cold-path XIP, which -O2's smaller code already helps.

Recommendation: keep DIO. Revisit QIO only on units confirmed to carry
genuine Winbond (0xEF) flash, or after adding a vendor driver + a
reliability soak for the 0x20 clone.

## Boot time is network-bound, not compute/flash

reset -> httpd 6.15 s, -> tailnet 8.24 s, -> pstop bond ~18 s. Identical
across -Og / -O2 / QIO. The 0-6 s and 6-18 s windows are Ethernet PHY +
DHCP + Tailscale control-plane registration + WG handshake — I/O waits,
not CPU. Boot-latency wins would come from static IP (skip DHCP),
faster link detection, or a cached netmap — NOT from code speed.
