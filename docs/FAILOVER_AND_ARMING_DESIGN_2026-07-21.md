# Design record — DERP failover speedup + arming hardening (2026-07-21)

Two designs, researched before implementation. Constraint honoured
throughout: **pstop_c (certification track, `components/pstop/upstream`)
is not modified.**

## 1. DERP failover speedup (target <10 s; measured 10–60 s)

### Root cause (from code trace)
The WG layer has no direct/DERP flag — DERP is "endpoint is 0.0.0.0"
(`wireguardif_peer_output`, wireguardif.c:157). Demotion direct→DERP has
exactly ONE trigger: a fixed 60 s trust lease (`ML_DISCO_TRUST_DURATION_MS`)
checked at 1 Hz in `disco_periodic_probes`
(`components/microlink/src/ml_wg_mgr.c`). Pong recency
(`last_pong_recv_ms`) is tracked but was consulted nowhere. Heartbeat
pings on an active direct path deliberately skip DERP, so during a
direct-path blackhole nothing renews trust and nothing reacts sooner
than the lease wall → failover = 60 s − (time since last pong) ± 1 s.

Two latent bugs found by the trace, fixed in the same patch:
- B1: trust-expiry demotion calls `wireguardif_connect_derp` only if the
  session is up; a session that died mid-blackhole leaves a stale direct
  endpoint that handshake retries hammer forever.
- B2: handshake retransmits go to `peer->ip` (dead direct endpoint) and
  even the 540 s `should_reset_peer` path restores `connect_ip`, which
  DISCO overwrote with the same dead direct endpoint — no DERP escape.

### Options considered
- **A. Pong-recency watchdog** (chosen): demote when a direct path stops
  producing pongs: `now - last_pong_recv_ms > path-dead threshold`
  (`ML_DISCO_PRIORITY_PATH_DEAD_MS` = 8 s). Deterministic, lives in the
  layer that owns the state, ~50 LOC.
- **C. Priority-peer scoping** (chosen, merged into A): tightened
  heartbeat (`ML_DISCO_PRIORITY_HB_MS` = 3 s) and threshold apply ONLY to
  `priority_peer_ip`; the 16-peer tailnet load profile (historic wedge
  cause) is unchanged.
- **B. App hint API** (`microlink_hint_peer_unreachable` from the pstop
  comparator's 1.5 s reply-loss watchdog): ~2 s failover. Deferred —
  layering smell + conflates machine-dead with path-dead; revisit only
  if A+C's ~7–9 s is insufficient.

Expected failover: heartbeat 3 s (priority peer), path-dead 8 s, reprobe
5 s → demote ≈ 7–9 s after path death (was 10–61 s). Flapping bounded:
re-promotion requires a fresh direct pong or authenticated direct RX —
both prove the path actually works.

## 2. Loop-read debounce + minimum-STOP-duration arming

### Failure being closed
EMC-induced both-channel loop blips (100–200 ms) in WiFi mode produced
agreed STOP→OK message pairs = the machine's arming gesture, re-arming
the robot with no operator. The lockstep cross-check is blind to
symmetric faults by design.

### Chip side (defence-in-depth): asymmetric release debounce
In `estop_channel_closed` (`firmware/main/main.c`): the open→STOP edge
stays SINGLE-TICK (no added stop latency — the stop path must never be
filtered). The closed→OK edge now requires
`LOOP_RECLOSE_DEBOUNCE_TICKS = 3` consecutive healthy reads, so any blip
yields a ≥300 ms STOP episode and brief chatter can't race the wrapper's
measurement. Fail-safe direction only: the debounce can only extend STOP,
never suppress it.

### Machine side (the enforcement point): wrapper-owned arming policy
Per feasibility study (agent-verified against pstop_c @ d9f4970, cited
in the review transcript): entirely in `host/machine_app_runner.c`,
library untouched.
- Episode tracking: timestamp the accepted STOP that moves
  `restart_state` NEED_STOP→STOP_RECEIVED (machine's own monotonic clock).
- Arming detection: `robot_state` STOPPED→OK across a
  `machine_process_message` call — machine.c:103 is the only line in the
  library that arms, and the runner is single-threaded.
- Veto: if the episode lasted `< min_stop_ms` (`host/machine.toml`,
  default 500 ms, 0 = off): call the PUBLIC `machine_stop_robot()` (lands
  in the library's own tested heartbeat-timeout state: STOPPED/NEED_STOP/
  stop_id=0), rewrite the pending reply's `message` to STOP before
  encoding (CRC is computed at encode time — wire-valid), log ANOMALY.
- status_cb latch: OK pulses from the library are NOT propagated until
  the policy approves the arming (the library emits status_cb(OK) inside
  machine_process_message, before the wrapper can veto). Any real
  actuation MUST hang off the latched path, never the raw callback.
- Episode tracker resets on veto, approved arm, BOND/UNBOND, and
  heartbeat-validation failure.

Properties: every added path moves toward STOPPED — a wrapper bug can
cause a spurious veto (availability), never a spurious arm. Behaviour
change to sign off: a transient STOP while running no longer
self-recovers in <min_stop_ms; resuming requires a deliberate
press-and-hold (strictly more conservative).

### If the policy must itself be certified (option, not taken)
~15-line pstop_c patch: `min_stop_ms` in `pstop_application_config_t`,
episode timestamp in `handle_stop_msg`, guard before the arming branch
in `handle_ok_msg`. Touches the certification artifact; requires new
tests + re-validation. **Full design, decision criteria, and risk
register: [`PSTOP_C_MIN_STOP_OPTION.md`](PSTOP_C_MIN_STOP_OPTION.md).**

### Test plan
- Host-side: `tools/pstop_test_remote.py` — a scripted pstop remote
  (bond / timed STOP / OK) against a dedicated runner instance: short
  blip must be vetoed (reply STOP, machine stays NEED_STOP), held press
  must arm. Exercises the real wire protocol incl. the new CRC.
- Chip-side: rebuild with debounce, OTA; verify normal bond/verdicts.
- Failover: re-run the netem blackhole ladder (`test/netem_ladder.sh`),
  measure dark window (expect ≤10 s vs 10–60 s baseline).

## Measured results (bench, 2026-07-21)

- DERP failover (blackhole on the direct WG path, 2 s sampling): replies
  dark from +5.2 s to **+9.3 s** after the cut (was 10-66 s); one clean
  re-bond; back to the direct path **6.6 s** after restore. Target <10 s met.
- Arming policy (tools/pstop_test_remote.py, wire-accurate): 5/5 — blip
  vetoed pre-arm and while armed, 800 ms press arms, re-arm after veto.
- Live veto observed against the real chip: a 200 ms episode was vetoed
  by the policy runner ("ANOMALY: arming VETOED ... 200 ms < 500 ms").
- Boot artifact: the first loop samples glitch open on every boot of this
  board; the initial debounce implementation surfaced this as a ~200-300 ms
  STOP->OK episode per boot (self-arming the pre-policy machine!). Fixed by
  the boot warm-up hold (settled = full closed-debounce cycle OR 5
  consecutive open ticks); verified flat veto count across an OTA boot and
  a clean soft restart.

## Failover improvements — implemented + measured (2026-07-21, round 2)

Landed (pstop_c untouched): P7 (preserve existing peer's direct path on
MapResponse re-ingest), P3 (priority pong-watchdog 8s→4s), P4 (fast DERP
reconnect — 5/5/10s then 60s — when a priority peer is set), P1 (dual-path
send for the priority peer: mirror its encrypted data over direct AND DERP),
runtime `config.max_peers`, and the NaCl shared-key cache.

**No-regression, real 128-peer Polymath tailnet:** 20 min, 8002 sent /
7984 repl, 0 send_fail, 1 mismatch (0.01%), stable, bc=0. All changes
ride together cleanly.

**Dual-path (P1) direct-kill test — the honest result.** With the priority
peer on a direct path + dual_path active, killing the direct WG path (100%
loss on UDP 51820, DERP/443 left up):

- The chip's heartbeats KEPT reaching the machine over DERP — the machine
  log shows counters continuing to arrive during the block. **The
  safety-critical outbound direction (operator verdict → machine) stayed
  up.**
- BUT the pstop protocol is BIDIRECTIONAL: the machine's replies stopped
  (its return path to the chip's dead direct endpoint hadn't failed over),
  so the chip's acked-counter fell behind and the machine rejected the
  heartbeats as `MSG_LOST` once `ack-lag > max_lost_messages (10)`. The
  chip's 1.5s reply-loss watchdog then rebonded; the link re-synced onto
  DERP and recovered within a few seconds. One rebond, clean recovery.

**Conclusion:** dual-path is not gapless *by itself* for a bidirectional
protocol — it protects the outbound (safety) direction, but end-to-end
seamlessness also needs (a) the return path to fail over and (b) enough
loss tolerance to ride the brief ack-lag. It is kept because it is
low-cost (2× sends for one peer), non-regressing, and in a real two-site
deployment the machine's own full tailscaled fails the return path over
independently — so outbound-stays-up + independent return failover
approaches the near-zero goal there. On this single-host bench the return
path is serialized through the one laptop, which exposes the gap.

**Recommended companion tune (not applied — machine-side/safety param):**
raise `machine.toml` `max_lost_messages` 10→~20 so the machine tolerates
the brief ack-lag during a return-path failover without rejecting valid
outbound heartbeats — with dual-path keeping outbound alive, this would
let the machine stay armed through a direct-path death instead of
rebonding. It slightly relaxes loss detection (heartbeat-timeout still
bounds liveness), so it wants deliberate safety sign-off.

**Bench limitation (task: realistic TS tests).** True gapless bidirectional
failover and real direct-path latency need a TWO-SITE setup — remote and
machine on independent internet connections, each with its own tailscaled
failing its own send direction over. This bench has one laptop serving as
both the chip's uplink and the machine, so the return path can't fail over
independently. The 128-peer netmap load, steady-state stability, and the
outbound-direction dual-path behavior ARE validated here; the end-to-end
gapless claim needs the two-site rig.
