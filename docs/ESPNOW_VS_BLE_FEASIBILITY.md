<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ESP-NOW vs BLE as Infrastructure-Independent RF Sidechannels — Feasibility Study

**Scope:** ESP32-S3 / ESP-IDF 5.5 pstop platform. Two use cases:

- **Use case A — heartbeat sidechannel:** remote↔machine direct RF heartbeat as a *second*
  path alongside the WiFi/WireGuard/DERP network. 5 Hz, 40–80 B frames, <200 ms latency
  budget (2 s loss-of-comms failsafe), 10–400 m outdoor industrial, up to ~10
  remote–machine pairs per site, must coexist with active WiFi STA **on the same radio**.
- **Use case B — safety tags:** battery-powered wearable tags for person/asset proximity
  near autonomous vehicles. Months+ battery, RSSI proximity, dozens of tags/site, <1 s
  detection latency, machine-side ESP32-S3 as detector while running WiFi.

**Safety framing (applies to everything below):** Neither ESP-NOW nor BLE is a certified
functional-safety channel (no ISO 13849 / IEC 61508 claim). Both are defense-in-depth
layers on top of the existing black-channel heartbeat. Any STOP-capable path must fail
safe: *loss* of the sidechannel must never mask loss of the primary channel, and a spoofed
sidechannel message must only be able to cause a (nuisance) STOP, never an ARM/run.

---

## 1. Technology overviews

### 1.1 ESP-NOW

Connectionless, vendor-specific 802.11 action frames on the WiFi PHY. Runs inside the WiFi
driver — no association, no IP stack. Key facts from the S3 API reference
(https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/network/esp_now.html):

- Payload: 250 B (v1), 1470 B (v2, IDF ≥5.4). Our 40–80 B heartbeat fits v1 trivially.
- Peers: max 20 total; max 17 encrypted (`CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM`, default 7).
- Crypto: PMK encrypts LMK (AES-128); per-peer LMK encrypts frames via CCMP
  (IEEE 802.11-2012). A random-value field "prevents relay attacks" (Espressif wording);
  see §7 caveats. Multicast cannot be encrypted.
- Default bitrate 1 Mbps; per-peer rate config allows the proprietary LR PHY (512/256 kbps).
- **Channel is not free:** in STA mode, ESP-NOW frames go out on the channel of the
  associated AP (https://circuitlabs.net/esp-now-with-wifi-coexistence/,
  https://docs.espressif.com/projects/esp-faq/en/latest/application-solution/esp-now.html).

### 1.2 BLE on ESP32-S3

The S3 has a Bluetooth 5 (LE) controller: 1M/2M PHYs, **Coded PHY (125/500 kbps) for long
range, and extended advertising** (https://www.espressif.com/en/products/socs/esp32-s3,
datasheet: https://cdn-shop.adafruit.com/product-files/5477/esp32-s3_datasheet_en.pdf).
Two operating patterns matter here:

- **Connected GATT:** paired link, connection events at 7.5 ms–4 s intervals, supervision
  timeout, LE Secure Connections pairing.
- **Connectionless advertising/scanning:** beacon-style. Legacy adverts (31 B payload, 3
  primary channels) or extended adverts (larger payload, optional Coded PHY on primary +
  secondary channels). Min advertising interval 20 ms.

### 1.3 BT Classic — dismissed

The ESP32-S3 controller is **BLE-only; it has no BT Classic support** (see datasheet
above). Even on chips that have it, Classic offers no connectionless mode, high pairing
latency, ~7-slave piconet limits, and the worst WiFi-coexistence profile of the three
options. Not considered further.

---

## 2. Per-criterion analysis

### 2.1 Coexistence with active WiFi STA on the same radio

**ESP-NOW.** Same MAC/PHY as WiFi, so there is no time-slicing arbiter — ESP-NOW frames
queue into the same transmit path and contend via normal CSMA. Two regimes:

- **Same channel as the AP:** impact is small. Espressif and third-party guidance describe
  delay as "usually minimal for typical low-to-moderate traffic loads"
  (https://circuitlabs.net/esp-now-with-wifi-coexistence/). Bench reports show <1 ms RTT
  for ~95% of packets with occasional 2–6 ms outliers when idle
  (https://www.esp32.com/viewtopic.php?t=12772), and <0.1% loss between unassociated nodes.
- **Different channel:** effectively broken. A STA-associated S3 cannot camp on another
  channel; community tests report >80% packet loss when one node is associated to an AP on
  a different channel than its peer (https://www.esp32.com/viewtopic.php?t=12772,
  https://circuitlabs.net/esp-now-with-wifi-coexistence/). **This is the single biggest
  ESP-NOW risk for us:** remote and machine may associate to different APs on different
  channels at a multi-AP site, or the machine may be on Ethernet/USB-NCM with WiFi on an
  arbitrary channel.

Mitigations: (a) pin site WiFi to one 2.4 GHz channel (often acceptable at industrial
sites we control); (b) exchange the current channel over the existing network path and
have the unassociated side follow; (c) run the sidechannel on a fixed channel and accept
that a node associated elsewhere transmits/receives only on its AP channel — i.e. design
for "sidechannel works whenever WiFi channel plans align, plus always when WiFi is down"
(when the STA is disassociated the radio can sit on the fixed sidechannel channel — which
is exactly the infrastructure-failure scenario that motivates the sidechannel).

**BLE.** Separate controller sharing the one 2.4 GHz RF front-end via time-division
software coexistence (`CONFIG_ESP_COEX_SW_COEXIST_ENABLE`;
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/coexist.html).
Documented behavior: BLE scan windows are interrupted/truncated when WiFi claims the RF;
WiFi throughput drops; Espressif recommends pinning WiFi and BT stacks to different cores
and keeping default connectionless-PS parameters. Practical consequences:

- BLE **scanning** effective duty cycle falls well below the configured window/interval
  when WiFi is active — expect missed adverts and detection-latency tails
  (https://circuitlabs.net/bluetooth-and-wifi-coexistence-strategies/).
- BLE **connections** can drop under heavy WiFi load (supervision timeout) — known issue
  class on Espressif chips (https://github.com/espressif/esp-idf/issues/11280).
- **Triple-stack warning:** WiFi + ESP-NOW + BLE simultaneously has open bug reports on
  IDF 5.x (https://github.com/espressif/esp-idf/issues/17874). Plan for WiFi+one
  sidechannel per device, not both.

### 2.2 Range and link budget (outdoor industrial, vehicle-mounted antennas)

Espressif's own outdoor benchmark (ESP32-C6 devkits, PCB antennas, no external antenna;
https://developer.espressif.com/blog/esp-now-for-outdoor-applications/):

| Mode | Open field | Forest/obstructed |
|---|---|---|
| ESP-NOW 1 Mbps | ~100% PDR to 150 m; 60% at 300 m; <20 ms latency | <50% PDR at 125 m; ~30 ms at 150 m |
| ESP-NOW **LR** (512/256 kbps) | ~100% PDR to **450 m**; 40% at 900 m; <25 ms | ~100% to 200 m; <50% at 400 m |

Independent tests report similar magnitudes (up to ~500 m LR outdoors,
https://www.atomic14.com/videos/posts/oz0a7Ur7nko;
https://www.hackster.io/news/long-range-wifi-for-the-esp32-9429ab89f450 for the 1 km
line-of-sight LR claim, which is soft-AP-to-STA best case, not moving-vehicle reality).
LR's ~6–9 dB sensitivity gain is real, but it is a proprietary PHY: **both ends must be
ESP32-family**, and rate must be set per peer (`esp_now_set_peer_rate_config`). With a
vehicle-mounted external 3–5 dBi antenna and a handheld remote at chest height, the
10–400 m requirement is credible on LR in line-of-sight; expect dropouts behind metal
structures — acceptable for a *secondary* path with a 2 s failsafe window.

**BLE:** 1M PHY with PCB antennas is realistically 30–100 m outdoors. Coded PHY S=8 adds
~12 dB of coding gain and the S3 supports it, so 200–400 m LOS is achievable with good
antennas — but only if *both* ends do Coded PHY (fine for our own remotes; rules out
commodity phone interop) and only via extended advertising or LE connections on that PHY.
For use case B (tags at proximity ranges, tens of meters), plain 1M-PHY legacy advertising
is more than sufficient, and *shorter* range is actually desirable for proximity semantics.

### 2.3 Latency and jitter

**5 Hz heartbeat (use case A):**
- ESP-NOW: sub-millisecond airtime for an 80 B frame at 1 Mbps; measured application-level
  latencies of ~1–20 ms including retries, among the best of any MCU wireless link
  (https://electricui.com/blog/latency-comparison,
  https://developer.espressif.com/blog/esp-now-for-outdoor-applications/). Jitter under
  concurrent WiFi is the open question — bench experiment §9.1.
- BLE GATT: bounded by connection interval (min 7.5 ms; realistic 30–50 ms under
  coexistence). Meets <200 ms when the link is up; risk is connection *drops* under WiFi
  load, and reconnection takes hundreds of ms to seconds — a bad failure mode for a
  heartbeat.
- BLE advertising as heartbeat: 5 Hz advertising is legal (min interval 20 ms), but the
  receiver's truncated scan duty under coexistence turns deterministic 200 ms budgets into
  probabilistic ones.

**Advertising→scan detection (use case B):** detection latency ≈ advertising interval ×
(1/effective scan duty) plus per-channel alignment. With tags at 100–300 ms intervals and
a machine scanner at ~50% effective duty (coexistence-degraded), P95 detection well under
1 s is realistic; at 1 s tag intervals it is marginal. ESP-NOW "tags" would beacon at ≥1 Hz
and be received with near-zero latency — but see power, §2.4.

### 2.4 Power (tag-side focus)

- **ESP32-as-tag is unsuitable for coin cells.** BLE advertising bursts on ESP32-class
  parts run ~130 mA per 2–4 ms event, and modem-sleep floors ~25 mA
  (https://hubble.com/community/guides/esp32-power-consumption-in-ble-mode-what-to-expect-from-advertising-scanning-and-connected-states/).
  Duty-cycled deep-sleep helps, but each deep-sleep wake costs 100–300 ms of boot at tens
  of mA before the first frame — at 1 Hz beaconing that is a multi-mA average: a CR2032
  dies in ~a day, a 500 mAh LiPo in under a week. Comparative bench measurements conclude
  "ESP32s are not suitable for BLE projects with low current consumption"
  (https://forum.seeedstudio.com/t/ble-advertising-current-comparison-esp32c3-c6-s3-mg24-nrf52840/287847).
  The same math kills **ESP-NOW tags**: ESP-NOW requires ESP32 silicon, so the tag
  inherits the boot/TX energy profile. ESP-NOW is fine for *mains/vehicle-powered* nodes,
  wrong for wearables.
- **Commodity BLE beacon tags (nRF52-class)** average ~5–15 µA: a CR2032 yields **16–24
  months at a 1 s interval / 0 dBm**, dropping to ~8–12 months at 500 ms or +4 dBm
  (https://hubble.com/community/guides/ble-advertising-interval-vs-battery-life-how-to-model-the-tradeoff-before-you-build/,
  https://devzone.nordicsemi.com/f/nordic-q-a/36982/cr2032-coin-cell-battery-life-estimation-with-nrf52-as-beacon,
  https://www.beaconzone.co.uk/blog/beacon-battery-size-type-capacity-and-life/). Note
  CR2032 capacity derates ~30% at −10 °C — relevant for outdoor sites; an AAA-cell or
  small-LiPo badge at a 200–300 ms interval still gives a year+.
- Use case A nodes are vehicle/handheld powered (existing pstop hardware) — power is not a
  discriminator there. ESP-NOW receivers do support duty-cycled listening via
  `esp_now_set_wake_window()` if the remote's battery budget ever matters.

### 2.5 Capacity and scaling

- **ESP-NOW:** 20 peers/device, 17 encrypted max (config; default 7). Ten pairs per site is
  comfortable since each device only peers with its counterpart(s); our existing
  many-to-one machine logic maps directly. Airtime for 20 nodes × 5 Hz × 80 B at 1 Mbps is
  ~0.1% of the channel — congestion comes from site WiFi, not from us. LR rate frames take
  ~4× airtime; still negligible. No protocol-level ACK collision management beyond 802.11
  CSMA — fine at this scale.
- **BLE tags:** advertising is stateless; dozens of tags at 100–300 ms intervals is
  routine (advertising collisions are randomized by the spec's advDelay). The scanner is
  the bottleneck, not the air. BLE *connections* on ESP32-S3 are practically limited to
  ~4–9 concurrent and degrade sharply under WiFi coexistence — another reason to avoid
  GATT for either use case.
- 2.4 GHz interference: both ride the same ISM band as site WiFi. ESP-NOW is *on* a WiFi
  channel (co-channel CSMA — polite, but shares that channel's fate); BLE hops/uses 3
  advertising channels chosen to sit between WiFi channels 1/6/11 — more resilient to a
  single busy WiFi channel.

### 2.6 Security

- **ESP-NOW:** PMK+per-peer LMK, CCMP frame protection, AES-128. Unicast-only encryption;
  key distribution is entirely ours (bake per-pair LMKs derived from the existing
  black-channel identity scheme). Espressif claims the random-value field prevents replay,
  but the mechanism is not publicly specified/audited — add an application-layer monotonic
  counter + truncated MAC inside the payload regardless (we already have sequence
  discipline in the pstop protocol; reuse it). MAC-address spoofing of *unencrypted*
  ESP-NOW is trivial, so encrypt or app-MAC everything.
- **BLE:** connected GATT gets LE Secure Connections (ECDH pairing, AES-CCM link crypto)
  and RPA privacy — strong, but irrelevant if we avoid connections. **Connectionless
  adverts are cleartext and replayable by design.** For tags: put a rolling code in
  manufacturer-specific data — AES-CMAC(counter ‖ coarse time) truncated to 4–6 B,
  Eddystone-EID-style — so a captured advert expires in seconds. Replayed/spoofed tag ⇒
  nuisance slow/stop (fail-safe). The un-mitigatable threat is **jamming/absence**: a
  jammed tag simply isn't detected. Therefore tags are an *assistive* proximity layer,
  never the primary safeguarding function — same posture as the heartbeat sidechannel.
- **Both:** no certification story. The sidechannel may assert STOP and may *corroborate*
  liveness, but the 2 s network-heartbeat failsafe remains the authoritative safety
  mechanism; sidechannel presence must never extend or reset it upward.

### 2.7 Implementation effort and footprint (IDF 5.5, alongside WiFi+lwIP+WireGuard+TLS)

- **ESP-NOW:** the code is already linked — it lives in the WiFi driver we ship. API is
  ~10 calls (`esp_now_init/add_peer/send/register_recv_cb`), mature since IDF 3.x. Adds
  ~2–4 KB flash, negligible RAM, no new task. Gotchas: send callback runs in the WiFi task
  (do not block); channel discipline (§2.1); LR rate per-peer config; v2 payload needs
  both ends ≥ IDF 5.4. Estimated effort: **days**, mostly in the channel-follow logic and
  telemetry.
- **BLE (NimBLE host):** +~70–120 KB flash, and — critically — the BLE controller
  allocates **tens of KB of *internal* SRAM** (DMA-capable). Our fleet telemetry already
  shows an internal-RAM low-watermark outlier around 8 KB on a busy DUT; enabling BLE on
  the machine node without a memory diet is a **crash risk**, and this is measurable
  before any radio work. Scanner-only (observer role) is the lightest configuration.
  Estimated effort: **1–2 weeks** for a hardened scanner + coexistence tuning + memory
  audit; more for GATT.
- Machine-side alternative if S3 memory or scan duty disqualifies on-chip BLE: a $2–3
  dedicated scanner (ESP32-C3 or nRF52 module) on UART feeding the S3 — decouples
  coexistence entirely at the cost of a BOM line.

### 2.8 Regulatory (brief)

All options are 2.4 GHz ISM. FCC 15.247 imposes no duty-cycle limit; ETSI EN 300 328
requires adaptivity (LBT) above 10 dBm — WiFi/BLE modes are covered by the module's
existing certification. ESP-NOW is standard 802.11 framing (no new issue). ESP-NOW **LR**
is a proprietary PHY; Espressif ships it within the module certifications, but if we sell
into ETSI markets, confirm the specific module's test report covers the LR rates.
BLE advertising at high rates is duty-cycle-fine everywhere.

---

## 3. Comparison matrix

| Criterion | A: ESP-NOW | A: BLE (GATT) | A: BLE (adv) | B: ESP-NOW tags | B: BLE adv tags |
|---|---|---|---|---|---|
| Same-radio WiFi coexistence | **Good** (same-channel); broken cross-channel | Poor (conn drops under load) | Fair (scan duty loss) | Good (detector side) | Fair — scanner duty is the bottleneck |
| Range 10–400 m outdoor | **Yes w/ LR + ext. antenna** (450 m @100% PDR open field) | No (~30–100 m @1M); Coded PHY maybe | Same as GATT | n/a (proximity) | Yes for proximity (10–50 m, tunable via TX power) |
| Latency (5 Hz / <200 ms; detect <1 s) | **~1–25 ms** | 7.5–50 ms up; sec-level reconnects | 20 ms floor; jittery under coex | <50 ms detect | P95 <1 s @ ≤300 ms adv interval |
| Tag power (months on coin cell) | n/a | n/a | n/a | **No** (~mA avg; days) | **Yes**: 16–24 mo @1 s CR2032 (nRF52-class) |
| Capacity | 20 peers/17 enc — fits 10 pairs | ~4–9 conns, fragile | fine | 17-enc-peer cap bites at dozens | **Dozens+ trivially** |
| Security | AES-128 CCMP/pair + app MAC | LE SC — strongest | cleartext + rolling code | CCMP but key mgmt/scale awkward | rolling-code MAC; jam = miss |
| Effort / footprint on S3 | **Days; ~0 RAM** | Weeks; big internal-RAM hit | Weeks; big internal-RAM hit | Tag HW is custom ESP32 | Scanner on S3 (RAM risk) or $3 co-processor; **commodity tags $5–15** |
| Fit | **Recommended** | Rejected | Fallback | Rejected | **Recommended** |

---

## 4. Recommendations

### Use case A — heartbeat sidechannel: **ESP-NOW (LR rate per-peer), same-channel discipline**

Rationale: only option meeting 400 m; near-zero marginal flash/RAM on an internal-RAM-
constrained firmware; lowest latency/jitter; both endpoints are our own ESP32-S3s so the
proprietary PHY costs nothing; encryption + our existing identity/sequence scheme covers
security. BLE GATT fails on range and on reconnect-storm behavior under coexistence — the
sidechannel would be down exactly when the network is stressed, defeating its purpose.

Design constraints to adopt up front: (1) channel-follow protocol — nodes advertise their
current WiFi channel over the network path while it is up, and fall back to a site-fixed
channel when disassociated; (2) sidechannel can assert STOP and corroborate liveness, but
never suppresses the 2 s network failsafe on its own; (3) per-pair LMKs derived from the
0x01<mac24> identity scheme + app-layer counter/MAC.

### Use case B — safety tags: **BLE connectionless advertising, commodity nRF52-class tags, S3 (or co-processor) scanner**

Rationale: tags are the whole game, and only BLE gives months-on-coin-cell wearables at
$5–15 in existing IP67 badge form factors. ESP-NOW tags would need custom ESP32 hardware
with ~days of battery life at 1 Hz — disqualifying. Machine-side S3 runs observer-only
NimBLE with software coexistence; tags advertise at 200–300 ms with a rolling-code MAC;
RSSI smoothed over ≥5 adverts with hysteresis zones (warn/slow/stop radii), calibrated per
tag TX power. Decision gate: if bench shows effective scan duty under pstop load cannot
deliver P95 <1 s detection, or internal RAM cannot absorb the controller, move scanning to
a UART-attached ESP32-C3/nRF52 co-processor (protocol unchanged).

---

## 5. Risks

1. **ESP-NOW cross-channel blindness (A, top risk):** remote and machine on different-
   channel APs cannot exchange ESP-NOW frames (>80% loss reported). Mitigated by
   channel-follow + single-channel site policy; residual risk during roaming transients.
2. **BLE internal-SRAM pressure (B):** controller needs tens of KB of internal RAM; fleet
   already shows an 8 KB low-watermark outlier. Measure before committing; co-processor
   fallback exists.
3. **Coexistence scan-duty collapse under load (B):** WiFi bursts (OTA, map-stream
   reconnects, DERP TLS bursts) may crater scan duty exactly when detection matters.
   Bench §9.2 quantifies; tag interval and P95 budget sized against the worst case.
4. **Triple-stack instability:** WiFi+ESP-NOW+BLE concurrently on one S3 has open IDF bug
   reports (https://github.com/espressif/esp-idf/issues/17874). Keep A and B features on
   separate device roles, or gate combining them on a dedicated soak.
5. **Jamming / absence is undetectable-by-design for tags:** a non-detected tag looks like
   an absent person. Tags remain assistive; primary safeguarding is unchanged.
6. **LR regulatory coverage in ETSI markets** for the proprietary PHY rates — verify the
   module test report before EU deployment.
7. **RSSI ≠ distance:** ±6 dB multipath swings around vehicles mean ~2× distance error;
   zones + hysteresis + multi-advert smoothing are mandatory, and stop radii must be set
   from measured site data, not free-space math.
8. **Firmware-first debugging discipline:** per our own operating rule, any sidechannel
   instability should be presumed firmware (coexistence config, callback blocking, memory)
   before blaming RF; instrument from day one (host-side sniffer as independent witness).

## 6. Minimal bench experiments (<1 day each)

**A — ESP-NOW under real pstop load.** Two S3 devkits running the *actual* pstop firmware
(WiFi STA associated, WireGuard up, 5 Hz network heartbeats flowing) + a ~50-line ESP-NOW
module sending 5 Hz 80 B sequenced frames. Matrix: {same-channel, cross-channel} ×
{network idle, network stressed via forced DERP reconnect loop + OTA download} × {1 Mbps,
LR 256 kbps} at 10/100/300 m outdoors. Record per-frame RTT, PDR, and gap histogram;
pass = P99 gap < 400 ms (one missed 5 Hz slot) in same-channel stressed case at 300 m LR.
This directly answers the critical unknown: coexistence degradation while the stack is busy.

**B — S3 scan duty + detection latency under load.** One S3 running full pstop firmware +
NimBLE observer (log internal-RAM watermark before/after BLE init — this alone is a
go/no-go gate). Five commodity nRF52 beacons (off-the-shelf, ~$10 ea) at 100/300/1000 ms
intervals, walked through 5–50 m around a parked vehicle. Under the same WiFi stress loop
as A, record: per-tag inter-detection gap CDF (pass = P95 < 1 s at 300 ms interval), RSSI
variance vs distance for zone calibration, and heap/internal-RAM watermarks over 1 h.
If P95 fails, rerun with an ESP32-C3 devkit as UART scanner to validate the co-processor
fallback the same day.
