# DERP region auto-negotiation — region-follow convergence gap (KNOWN issue)

**Status:** Known, **green-safe** (never drops the safety loop), documented here; robust fix **deferred** (design below). Not a blocker for the "12 h without losing green" goal.
**First observed:** run-19 soak, 2026-08-11, build `5d25354` — but the underlying mechanism predates it and is independent of the relay-recovery work on this branch.

---

## 1. Symptom

In DERP region **auto-negotiation** (`derp_region_override = 0` → `derp_region_source = auto-primary`), a remote follows the **primary machine's** region. When the machine's region changes, a remote **occasionally fails to follow** and stays homed on the *old* region.

Observed on the bench (host-side, `/admin/api/monitor`):

- machine → region 9; DUT followed (`derp_home_region=9`, `mbb_commits`++), **PS did not** (`derp_home_region=2`, `mbb_state=0`, `mbb_commits=0`, `mbb_proofs_*=0`).
- PS never *attempted* the switch — no MBB candidate, no proof activity — i.e. its computed **target region never became 9**.
- **Green never dropped.** machn stayed `armed` (l0=1 l1=1); both remotes stayed `wg_direct=1`. PS@region2 ↔ machn@region9 held a direct bond throughout.
- Frequency: rare — ~1 dropped follow in ~12 forced switches (run-18 had 11/11 clean; run-19 dropped 1 on its first switch).

Net effect: a **region split** (machn + one remote on the new region, the other on the old), which is *cosmetic/optimization-level* while the direct path is up, not a safety event.

## 2. Root cause

A peer's DERP region reaches a remote **only** via a coordination-server `PeersChangedPatch` carrying `DERPRegion`. Two facts combine into a permanent-until-next-change stale state:

1. **The patch can be dropped.** It is delivered through the peer-update queue. `net(derp): never drop the region-bearing peer patch` (commit `1b068a3`) front-queues region-bearing patches (2000 ms) like pinned full-adds, which *reduces* the drop rate but does **not** eliminate it under a heavy peer-churn burst on a busy tailnet.

2. **There is no recovery for a dropped region patch.** `neg_target_region()` (`ml_wg_mgr.c`) reads the **cached** `peers[idx].derp_region`. Steady-state coordination updates are `OmitPeers=true` (`ml_coord.c` — `do_send_endpoint_update`), so a remote **never re-pulls** a peer's current region; a full re-sync only happens on a coord reconnect / reboot. And crucially, the coordination server only pushes a `PeersChanged` on an **actual change** — verified on the bench: re-issuing the machine's *unchanged* region (`POST /admin/api/settings {"derp_region":9}` while already 9) generated **no** new patch to PS. So once the patch drops, the remote's cached machine-region stays stale **until the machine's region genuinely changes again** (i.e. the next switch), at which point a fresh patch is generated — which may itself drop.

This is the same steady-state "`OmitPeers=true`, never re-pull peer state" gap that also underlies the endpoint-staleness / relay-stuck issue (see `reference_pstop_autoneg_split_green_drop` and the relay-recovery work on branch `derp/relay-recovery`); here it manifests on the **region** field instead of **endpoints**.

## 3. Why it is green-safe

The 5 Hz protective-stop heartbeat rides the **WireGuard** tunnel, not the DERP home region:

- When bonded **direct** (`wg_direct=1`, the normal state), packets go peer-to-peer and never touch a DERP relay — the home region is irrelevant to a running bond. A region mismatch between two directly-bonded peers therefore does **not** gap the heartbeat.
- The DERP relay leg is a fallback used only if the direct path dies. A region split raises the *cost* of that fallback (a cross-region relay hop), but for a LAN remote (PS) the direct path is stable, so green holds indefinitely.

Hence: region-follow lag = a convergence/optimization miss, **not** a green-status loss. It fails only the strict "every switch converges both remotes" bar, not the "12 h without losing green" bar.

## 4. Mitigation design (deferred — not yet implemented)

Ranked; **option A is recommended**.

### A. Carry the machine's authoritative region in-band over the pstop bond (recommended)
The machine is region-authoritative (§4) and knows its own region; the **pstop safety bond is the one channel guaranteed up whenever auto-primary is active** (green ⇒ bond up). Have the machine include its current effective DERP region in its periodic pstop reply, surface it through the dcs primary-machine slot table (`dcs_primary_machine_info`), and have `neg_target_region()` **prefer that reported region** over the possibly-stale `peers[idx].derp_region`.
- **Pros:** eliminates the coord-patch dependence entirely; convergence becomes as reliable as the bond itself; recovers automatically because it is re-sent every heartbeat.
- **Cons/constraints:** a pstop message-format addition — **safety-path-adjacent**. Must be strictly additive/optional and gated so a missing/garbled field can **never** affect the STOP/OK decision (per the "no test/aux logic in the safety path" and "protective-stop, not E-stop" rules). Bump the pstop protocol version; both roles must tolerate the field's absence for mixed-firmware fleets.

### B. Periodic reconciliation re-fetch (auto-primary only)
When auto-primary is active and the primary machine is bonded, issue a bounded-cadence (e.g. every 2–5 min) `Stream=false, OmitPeers=false` MapRequest to re-pull peer `HomeDERP`s, so a missed region patch self-corrects.
- **Pros:** self-contained in microlink; also refreshes endpoints (helps the relay-stuck case).
- **Cons:** control-plane traffic; **depends on the coord full peer entry actually including `HomeDERP`** — but full re-syncs frequently deliver `derp_region=0` (region is carried separately via patch; see the region-preserve guard in `ml_wg_apply_peer_update`), so this may **not** reliably deliver the region. Needs server-side verification before relying on it.

### C. Machine periodic re-advertise
Have the machine periodically re-advertise its region even absent a change, so a dropped patch is covered by the next advertise.
- **Cons:** the coord only emits a `PeersChanged` on an actual **change** (bench-verified), so a plain re-advertise of the unchanged region produces no patch. Would require a server-side change to re-push, or a machine-side region "toggle," which is hacky. **Not recommended.**

**Recommendation:** Option **A** (in-band region over the pstop bond) is the robust, coord-independent fix. It should be scoped carefully as a safety-path-adjacent change (additive field, version-bumped, absence-tolerant) when this is picked up.

## 5. Interim posture

- Accept region-follow lag as a **known, green-safe** convergence miss; it does not gate the green soak.
- The relay-recovery work on `derp/relay-recovery` (disco-session reset for a relay-stuck safety peer) addresses the *endpoint* half of the same steady-state gap; the *region* half is documented here for a later, separate change.
