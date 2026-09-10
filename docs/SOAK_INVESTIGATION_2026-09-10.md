# PSTOP06 USB / Ethernet stability investigation

## Acceptance and ownership

The target is **four consecutive armed hours on each transport**, on one final
firmware and machine configuration. A failure resets the affected clean window;
elapsed test time is not acceptance. A firmware/configuration fix requires new
qualification. The machine silence budget remains **400 ms × 4 = 1600 ms**, with
an externally observed STOP deadline of at most two seconds.

- Bench `100.124.215.14`: PSTOP06, firmware, relay loops, transport selection,
  DUT telemetry, packet capture and qualification controller.
- Peer `100.110.35.58`: isolated ROS 2 Humble software machine, UDP **8893**, ID
  **0x01020393**, domain 42, operator allowlist **[0x01D7F344]**.
- DUT slot **3** is dedicated to this run. The existing slot-0 machine at peer
  UDP 8891 remains a separate process.
- USB management: `10.43.0.122` via `esp-pstop0`; DUT Ethernet: `10.74.30.17`;
  DUT VPN identity: `100.75.70.74`.

USB qualification disables the DUT Ethernet driver and verifies active interface
2, inactive WiFi, and encrypted USB underlay traffic. Ethernet qualification
verifies interface 1 and inactive WiFi, and uses a dedicated host nftables table
to restrict USB to local HTTP/DHCP/ICMP management. Merely plugging in USB does
not select USB transport. Inner VPN packet source addresses do not identify the
physical uplink. Relays 1/2 operate the loops; relay 3 is left as found (OFF).

## Build provenance and tested diagnostic correction

Upstream `main` is `e67b5c5`; its protocol is already v2 / 48 bytes and includes
remote-announced roles. ROS 2, `pstop_c` and `common` match that revision in the
newer firmware branches. Bench integration `d4b86ad` combines:

- `feat/lifetime-health` at `9ad2fce`, including current documentation/test client;
- `fix/relay-refetch-backoff` at `6f51532`, retaining the deployed recovery change.

Commit `901cafd` corrects a demonstrated forensic defect: `/api/last_log` used a
4096-byte buffer for a 7168-byte retained ring and exported its **oldest** prefix,
discarding the newest approximately 3 KiB, potentially including the panic.
The endpoint now exports the complete ring using HTTP-only PSRAM scratch.
This is an evidence-collection correction, **not a claimed connection fix**.

The local DUT was updated to `v1.2-29-g901cafd`, ELF prefix `89a1018c1`.
Hardware readback verified 7168 returned log bytes, valid OTA partition, retained
operator role, real slot-3 bonding, and relay-driven ROS arming.

Build/verification performed with pinned ESP-IDF 5.5.4 image
`sha256:b9f2d6ea1c19e0c9f7959bdb74a9e3c775642f9d0f3b841937c5fa3363db892b`:

- firmware build, compiled dual-core verdict diversity, sdkconfig parity: pass;
- host clock guard: 5117 checks; crypto KAT: 21; demote veto: 24; aux channel: pass;
- collector offline regression tests and live short checks are recorded with
  their respective artifacts, not substituted for four-hour qualification.

Provisioning was transferred privately from the exact prior DUT build. Neither
credentials nor core dumps belong in Git.

## Validated physical STOP timing

USB cut #2 was independently reviewed against peer pcap:

- 05:16:02.906Z: fresh ROS ACTIVE following the physical gesture.
- At least 11.3 seconds of reciprocal OK traffic before the cut.
- 05:16:14.182818Z: last **accepted** OK, answered by the machine.
- 05:16:15.802200Z: ROS ACTIVE → DEACTIVATED.
- Observed silence STOP latency: **1619.4 ms**, below 2000 ms.
- Traffic resumed at 05:16:18.455Z; BOND was accepted; a new gesture restored
  ACTIVE at 05:16:21.102Z.

The first apparent 1.032-second USB measurement is **invalid**: an earlier
transport switch had already stopped the machine, and the calculation used a
rejected BOND rather than the last accepted frame. Its artifact is retained with
an invalidation note. Transport-transition intervals are labelled, not credited.

## Actual uncommanded USB losses

With interface 2 selected, Ethernet/WiFi off and no relay/routing actions, the
peer observed two genuine losses around **05:23:54Z** and **05:25:02Z**. Both DUT
machine slots lost replies and rebonded together. The first caused an unintended
armed → stopped transition; the second occurred while already stopped.

Failure-triggered DUT snapshots show both DERP connections (home region 9 and
auxiliary region 2) down. WG key-invalid/expired counters stayed zero; peer-remove
and re-key-retire counters did not change. Heap remained available, with no new
recorded >1-second WG/DERP loop event. Control-plane reconnect counters increased
after safety traffic recovered; forced refetch was unchanged during the first
loss. Thus these observations do not justify blaming peer teardown or extending
refetch/heartbeat deadlines.

The USB pcap contains actual inbound resets:

| UTC | Source → DUT TCP port |
| --- | --- |
| 05:23:53.789825 | `192.73.252.134:443` → `63517`, RST |
| 05:23:53.851793 | `192.73.248.83:443` → `63519`, RST |
| 05:23:56.936259 | `192.200.0.106:80` → `63524`, RST |
| 05:25:02.076753 | `192.73.252.134:443` → `63525`, RST |
| 05:25:02.119596 | `192.73.248.83:443` → `63527`, RST |
| 05:25:04.248624 | `192.200.0.113:80` → `63528`, RST |

The two DERP flows had no preceding FIN in the retained capture. This supports
**received TCP resets → lost relay routes → ENOTCONN → heartbeat loss/rebond**.
The origin of the resets (servers versus an intermediate network device) remains
unresolved. Separate host-uplink capture was added to compare NAT mappings and
packet provenance on recurrence. The source address of an RST alone does not
establish which device generated it.

## Separate old-image OTA watchdog event

The first USB HTTP OTA attempt reset the old `6f51532` image before switching OTA
partitions; a subsequent Ethernet upload succeeded. Exact matching ELF
`6d130a0dfae081dcadd18f4374cfb844a9acae48f453bdf378174b121924fb88`
decoded the saved core: **task watchdog, IDLE1 starved**, interrupted WG manager
inside synchronous UART output during `add_peer` / peer refresh.

UART execution at capture is not its duration. The existing 40-line console
budget, incomplete all-action peer-ingest work bound, coordination/TinyUSB CPU1
load, and TinyUSB synchronous-send waits are competing/contributing hypotheses.
Missing PSRAM in the dump is not UAF evidence. No watchdog enlargement, priority
change, or speculative logging/transport fix was applied for this event.

## Instrumentation validity

Live preflight exposed and corrected instrumentation defects:

- DUT endpoints are `/api/role` (authenticated GET), `/api/health`, and
  `/api/last_log`, not corresponding `/admin/api/` aliases.
- An early forced health read must not consume its future periodic tick; that
  formerly produced a false approximately 58-second observation gap.
- Peer AF_PACKET capture must see outgoing TUN traffic; the initial observer
  did not. No-reply/stall events through peer sequence 346 are invalidated.
- Replies must match **any outstanding request**, not only the most recently
  arrived request. Stale-echo events through sequence 490 are invalidated; the
  sequence-490 pcap proves correct replies to two closely spaced requests.
- Delayed reports of relay-induced mismatch increments are labelled as
  intentional preflight evidence. Uncommanded mismatches remain failures.

The peer generation after these observer corrections is
`5c5a7064726185f11b5a53cb9bc7ff2fca8e06e400f7d3f18e4dbf10e1af8946`.
The node binary/parameters remain unchanged. Each qualification freezes the
actual live generation, hashes, identities and firmware before counting time.

## Evidence and continuation

Bench evidence is under `/tmp/opencode/pstop-soak-20260910/`: action journal,
raw state/monitor/health and peer samples, per-event prehistory and diagnostics,
USB/uplink pcap rings, host kernel/link logs, matched old ELF and protected core.
Peer status/config/events/evidence are served at `http://100.110.35.58:8895`.

`tools/soak_stability.py` is the read-only one-phase acceptance collector. It
requires fresh armed/packet evidence, reconciled event watermarks and closing
health/config snapshots. Only an actual clean target returns zero. The bench
controller handles labelled transitions, evidence-before-rearm, supervision and
notifications to both agents. A local `PAUSE` marker is required before manual
hardware or firmware intervention while that controller owns the rig.

**Setup results above are not four-hour passes.** The campaign manifest and
per-phase `status.json` files are the authority for subsequent qualification.
