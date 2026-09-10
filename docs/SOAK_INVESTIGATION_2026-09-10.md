# PSTOP06 USB / Ethernet stability investigation

## Verified milestone: Ethernet four-hour pass

The `campaign-20260910T100027Z/ethernet` window **PASSED 14405.500 seconds**
of consecutive clean armed operation, completing closing checks at approximately
14:00:58 UTC. The independent machine-side review concurs: no machine state
changes, remote drops or rebonds; capture coverage spans the complete window.
The transport was Ethernet with a direct WireGuard path throughout.

DUT slot-3 deltas: sent +71119, replies +71116, send failures +0, rebonds +0.
No new boots, comparator/lifetime mismatch events, button presses, Ethernet
recoveries or health-flush failures were recorded. There were 42 warning records,
including seven isolated reordered-frame incidents and their duplicate reporting
channels, recovered driver retries, and control-plane reconnects; these were not
42 connection outages. Closing health/configuration/event-watermark checks passed.

Firmware remained `v1.2-29-g901cafd` / ELF `89a1018c1`, operator role; peer
generation remained `c51a8398c7f761436af2a397febc40a4df415abdb8ea06bea39931184ac8f074`.
See the phase's `result-summary.json`, `status.json` and campaign manifest.

The controller then transitioned to USB, settled and re-armed, and started the
USB phase at **14:01:25 UTC**. USB four-hour acceptance is still pending at this
milestone. A future clean window documents observed behavior on this build; it
does not resolve the open USB deferred-send defect hypothesis. Any firmware fix
requires both modes to be qualified again on that new build.

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

### Subsequent correlation: the bench host also loses its connections

The bench's own `tailscaled` journal independently records resets to its DERP,
control-plane and log-upload TCP connections at **05:23:57–59Z** and
**05:25:01–02Z**, in the same windows. Its advertised STUN endpoint changes from
`192.184.222.186` to `104.59.120.100` and then back; cached endpoints remain in the
reported list. Its local interface/default route did not change.

No DUT SYN precedes the first reset in either window: the new DERP connections
start after those resets. Bench conntrack occupancy was 403/262144, established
timeout 432000 seconds, and the live nftables rules contained no TCP-reject rule.
RST TTLs were 41–50; IP-ID 0/DF/window 0 are not unique middlebox fingerprints.

These observations strongly favor a **shared upstream WAN/NAT disruption** over
a DUT-only peer-table or USB-masquerade fault. They do not yet identify the router
policy or prove a particular failover mechanism. No safety timeout or firmware
hot path was changed to compensate. Uplink capture and continuous bench
`tailscaled` logs were added for the next occurrence.

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
The peer subsequently removed historical active probing loops and the active
ping from its diagnostic collector, announcing generation
`5e6bd5394329cb92886e9d1dc7ffce507a2a5b357fb5918fca2321c2d4780b50`.

## Evidence and continuation

### Qualification-collector replay correction

The 05:37Z campaign was paused at 06:39Z after repeated collector alerts while
the peer independently observed continuous ACTIVE/OK traffic. Arrival-order
replay of 12,779 retained records over 3685 seconds established:

- 31 false frontier regressions: converting an unchanged native watermark
  through successive HTTP epoch/monotonic receipt pairs shifted the bound by
  only 0.067–0.499 microseconds in the inspected cases. Certificates now retain
  their original bound for the same cumulative watermark; native source-clock,
  sequence, counter and coverage regressions are still rejected.
- Three negative-age alerts: the peer timestamped its HTTP envelope before
  reading an atomically replaced snapshot. Its writer now reads first, then
  timestamps. Receiver validation also handles the precisely bounded legacy
  race using absolute observation time at receipt, never a blind zero clamp.
- One remaining 1.672-second HTTP-receipt coverage gap is retained as an
  **observation gap**, not a DUT fault or an accepted clean interval.

The corrected collector's full replay has zero frontier/negative-age false
failures. Compact real-timebase regression fixtures plus mutations verify that
genuine clock rollbacks, counter loss and coverage loss still fail. Acceptance
events now retain previous/candidate/certified bounds and the limiting component.
Original event files remain immutable; separate assessments classify the alerts
as `COLLECTOR_FAULT`, `PEER_STATUS_ENVELOPE_RACE` or
`OBSERVATION_COVERAGE_GAP`. No firmware or machine timeout changed for these fixes.
Peer generation after the timestamp-order correction is
`4f188f43319437a37c351041f6f009555c92b8122f2834e7f705f7b47a9b5df0`.

Live validation then exposed insufficient freshness margin from the original
1.00–1.25-second snapshot cadence plus topic age, network delay and the retained
100 ms clock-uncertainty allowance. The peer now publishes snapshots every
250 ms (generation
`09d12a820f2a06806ce5d32ec9ad754cb619073301dbc8cba35b5c24df932b21`),
and the bench polls peer status/events every 500 ms while DUT state remains at
1 Hz. No machine deadline or evidence-age bound was increased. The corrected
collector passed 95 offline tests and a **303.316-second live clean window with
zero failures**; one control-plane reconnect was retained as a warning without
any safety interruption. Fresh four-hour qualifications follow this validation.

### Later observation and transport events

- **07:37:18Z:** a 1.457-second DUT HTTP response delay expired the previous
  observation, while peer pcap showed uninterrupted OK/OK traffic. Frame stamps
  localized the main jitter after generation; USB/uplink captures showed a
  shared outgoing payload pause already at the USB boundary and only
  microseconds of host forwarding delay. Exact scheduling/TCP/TinyUSB cause is
  unresolved. Header-only HTTP capture, usbmon metadata and host counters were
  added for recurrence; no firmware behavior was changed.
- **08:07:16Z:** a recorder-freshness alert was traced to libpcap delivering
  packets in approximately one-second batches despite `tcpdump -U`. An earlier
  inode-timestamp explanation was superseded by syscall timing and size-growth
  measurements. The peer now uses `--immediate-mode`, an independent 4 Hz
  file-size progression sampler, and timestamped background status refresh.
  Recorder configuration is included in provenance without volatile runtime
  fields. Generation
  `c51a8398c7f761436af2a397febc40a4df415abdb8ea06bea39931184ac8f074`
  passed a 303.75-second live gate check with zero failures.
- **08:29:13–08:30:22Z:** a second shared upstream disruption pair occurred,
  approximately 68 seconds apart. Bench STUN endpoints again changed public IP;
  its own DERP/control/log connections also reset. Matching reset packets
  appeared on the uplink before USB by 7–20 microseconds. No interface change
  occurred at qualification start. The first outage added seven local send
  failures per DUT slot, caused a rebond and machine STOP, and required the
  controller's documented gesture. The second included a genuine approximately
  two-second return gap. USBmon showed successful transfers in both directions,
  at least three tracked bulk-IN requests pending, and no completion errors.
  This is not evidence of a global USB freeze. Gateway logs are still needed to
  establish the exact WAN/NAT policy causing the repeated disruption.
- **08:32:41Z:** a smaller return-delay bubble combined with observation aging:
  671 ms raw DUT reply age plus 976 ms local elapsed time exceeded 1.6 seconds.
  The following sample showed 149 ms age and catch-up replies. It is retained
  as a return-delay/observation gap, not a demonstrated machine silence STOP.

### Review annotations are not new measurements

Peer journal sequences 806/807/817 incorrectly marked reviews of prior events
as ERROR. Their publication, and the watcher bundles generated in response
(808/818), caused duplicate qualification resets. Original records remain;
corrections classify them as `ANNOTATION_SEVERITY_FAULT`.

The receiver retains the four explicit annotation types from `machine_agent`
(`INDEPENDENT_RESULT`, `SOAK_PHASE`, `INSTRUMENTATION_FIX`, `PREFLIGHT_ACTION`)
without treating them as fresh failures. The same applies narrowly to
`EVIDENCE_COLLECTED` records explicitly attributed to those annotations. Genuine
packet, ROS, DUT and recorder-exit alarms, and unknown failure types, remain
failure-bearing. Regression tests replay the observed shapes and verify that
actual silence/recorder failures still reset qualification.

After this correction, the next attempt order is **Ethernet, then USB**, so
recurring USB-path disruptions do not prevent long-duration coverage of the
other transport. Both still require four consecutive clean hours on the same
build/configuration; WAN events are not excluded.

### USB management-response loss during Ethernet qualification

At **09:38:57Z**, Ethernet pstop remained healthy, but a USB `/state.json`
response took 1.513 seconds and expired the preceding observation. Header-only
capture shows a missing 1440-byte TCP body segment, duplicate header/later-body
segments, repeated ACKs for the missing byte sequence, and gap recovery about
1.3 seconds later. USBmon places the two later-body copies in one 3048-byte IN
completion, preceded by a 180 ms IN-completion gap with requests still pending.
USB completion statuses and host receive-error counters did not change.

Independent source review identifies plausible stale deferred-work and completion
races in `tinyusb_net_send_sync`. Abstract schedules reproduce omission and
duplication, but callback-generation/NTB-content evidence is still needed to
establish the exact hardware interleaving. A proposed pointer-clear-only patch
was rejected because it can lose a semaphore token and deadlock the TCPIP caller;
clearing an event bit at entry is also insufficient. No component patch was
applied.

Qualifying telemetry now follows the selected underlay: Ethernet uses
`http://10.74.30.17`, USB uses `http://10.43.0.122`. This separates the Ethernet
test from the demonstrated USB management fault; the USB defect remains open
and USB still requires its own full qualification. The prior reset stands.
The aligned Ethernet observation path passed a 123.746-second live check with
zero failures; five recoverable/diagnostic warnings were retained. Firmware,
machine settings and all timing limits remain unchanged.

### Current artifact locations

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

**Only explicitly verified windows are passes.** The campaign manifest and
per-phase `status.json` files are the authority for subsequent qualification.
The supervised campaign started at **05:37Z**, directory
`campaign-20260910T053721Z`, with USB first and a 14400-second clean target for
each phase. This start record is not a completion result.
