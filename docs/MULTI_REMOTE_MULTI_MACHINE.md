# Bonding and using multiple remotes and machines

This is the authoritative guide to how pstop remotes bond to pstop machines over
Tailscale, how to configure a many-remotes ↔ many-machines fleet, and — most
importantly — the **peer-cap rule you must follow so a remote can always reach a
machine on a large tailnet.** Read the "Peer cap" section before adding units.

## 1. The model in one paragraph

A **remote** (the E-stop button unit) bonds to one or more **machines** (the units
that drive the robot's stop relays) and exchanges heartbeats with each over a
WireGuard/Tailscale black channel. Safety is **fail-safe OR**: a machine runs only
while *every* bonded remote says OK; any remote that sends STOP, goes silent past
its heartbeat timeout, unbonds, or sends a bad message → the machine STOPs. A
remote is either an **operator** (may STOP *and* re-arm) or **stop-only** (may STOP
and is heartbeat-monitored, but may never re-arm). See
`docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md` and the authorization model in
`docs/API.md`.

## 2. Topology

- **Many remotes → one machine:** every bonded remote can STOP the machine (OR).
  Only an operator remote can re-arm. First operator to STOP owns the re-arm cycle.
- **One remote → many machines (≤4 slots):** the remote holds a peer slot per
  machine (ports 8891–8894), bonds each independently, and a STOP/silence on ANY
  slot stops that machine. See `docs/API.md` "Multi-machine".
- Machines and remotes are ordinary Tailscale nodes; they reach each other by
  their tailnet (VPN) IP over WireGuard — **direct** when a path can be punched,
  else **DERP-relayed**. Cross-site (different LANs) is the normal production case.

## 3. How to bond a remote to a machine (step by step)

1. **Point the remote at the machine.** On the remote, set a machine peer slot to
   the machine's tailnet IP + pstop port:
   `POST /api/pstop_peers?slot=N&ip=<machine_vpn_ip>&port=8890` (slot 0..3; admin
   auth). `POST .../api/pstop_peers?slot=N&clear=1` clears a slot. The remote
   **pins** each configured machine so the machine can never be trimmed from the
   remote's netmap (see §5).
2. **Decide operator vs stop-only.** By default the machine accepts the remote as
   **stop-only** (it can stop but not re-arm). To let this remote re-arm, add it to
   the machine's operator allowlist: `POST /api/operators?add=<remote_id>` on the
   machine (id is the 32-bit `0x01<mac24>`, e.g. `0x01d7f344`; hex or decimal).
   `?del=<id>` removes; `GET /api/operators` lists. **This step also PINS the
   remote on the machine (see §5) — it is what makes a cross-site remote reliably
   bond.** The operator allowlist defaults to empty (maximally safe).
3. **Verify** (see §4).

Provisioning at scale is the fleet's job: it stores each remote's machine slots and
each machine's operator list and re-pushes them after any reflash (NVS-preserving,
but re-push to be safe). See the fleet-side handoff.

## 4. Verifying a bond

- **On the machine** — `GET /state.json` → `bonded_remotes[]`: each entry has
  `id`, `state` (2 = bonded), `age_ms` (freshness — small = live), `rtt_ms`,
  `wg_direct` (1 = direct path, 0 = DERP), and `stop_only` (0 = operator, 1 =
  stop-only). `operators[]` lists the configured operator ids.
- **On the remote** — `GET /state.json` → `pstop_machines[]` per slot: `state`,
  `sent`/`replies` (both advancing = healthy), `send_fail`, `rebonds`, `hb_ms`,
  `last_reply_ms`, `wg_rtt_ms`.
- A healthy bond: machine `state=2, age_ms` small; remote `sent≈replies` and both
  climbing.

## 5. The peer cap — the rule that prevents "a remote can't reach the machine"

**Read this.** Each chip holds at most `CONFIG_ML_MAX_PEERS` (= **128**, a hard
ceiling) WireGuard peers. On a tailnet with **more than 128 nodes**, every chip
**trims** its peer table, keeping **pinned** peers and **recently-active** peers and
dropping the rest. WireGuard is symmetric: a session forms only if **both** ends
hold each other's key. Therefore:

- A **remote pins each configured machine** → the remote never trims its machine. ✅
- A **machine pins each operator-allowlist remote** → the machine never trims its
  operators, so a brand-new operator that has never sent a packet is still kept and
  can bond. ✅ (Implemented via a microlink peer-wanted hook; the machine matches
  the incoming netmap FQDN `pstop-01<mac24>.<tailnet>` against its operator list.)

**Consequence you must design around:**

- An **operator** remote will always bond to its machine, anywhere on the tailnet,
  because both sides pin each other. **Make any remote that must reliably re-arm a
  machine an operator** (§3 step 2).
- A **stop-only** remote is **best-effort** on a >128-node tailnet: if it has no
  recent activity it may be trimmed from the machine's netmap and fail to bond
  (`sent=0`, `send_fail` climbing, `errno 128`, and it won't appear in the machine's
  `/admin/api/peers`). This is not a safety hole (the machine isn't relying on a
  remote it never bonded), but it *is* why a bench/test remote "won't connect."
- **Keep the tailnet lean.** Retire stale/test nodes. Below 128 total nodes, nothing
  is trimmed and every remote bonds regardless of operator status. `s_extra_pins`
  holds up to `ML_EXTRA_PINS` (= 16) app-pins per chip (machine slots on a remote,
  operators on a machine).

### 5.1 Scaling to hundreds of remotes and machines

The **128 WG-session ceiling (`ML_MAX_PEERS`) is per chip, not per site.** Hundreds
of units are fine as long as no single node needs sessions to more than ~120 others:

- **Remotes never scale-limit** — a remote sessions only with its ≤4 configured
  machines + the fleet (~5 peers), at any site size.
- **A machine sessions with each bonded remote**, so a *single machine* can serve at
  most ~120 remotes. Beyond that, partition remotes across more machines (a topology
  choice) or raise `ML_MAX_PEERS`/`WIREGUARD_MAX_PEERS` in firmware (bounded by RAM
  and needs validation).

`max_peers` (runtime `POST /api/settings`) is the per-site **active-set** tuning:

- **Dedicated pstop tailnet (recommended for production):** with only real pstop
  nodes in the netmap there's no noise to trim. A value like **`max_peers = 32`** is
  a good production default — it comfortably covers a machine's fan-out + fleet +
  operators while keeping the DISCO/WG probing set small, and it's well under the 128
  ceiling. Raise it toward 128 only for a machine that must serve >~28 remotes.
- **Shared/noisy tailnet:** a low value (e.g. 16) suppresses churn from unrelated
  nodes — a *test accommodation*, not a production setting.
- **`ML_EXTRA_PINS` (= 16) caps pinned peers per chip.** For a machine that must
  reliably hold >16 operators, raise it (small static-RAM cost) so every operator
  survives the trim; stop-only remotes remain best-effort.

**Do not ship `max_peers = 16`** — it would cap a machine at ~16 remotes. Set it to
the production value (≈32, or higher per fan-out) once the deployment tailnet is
sized.

## 6. DERP / direct-path notes

- A chip holds ONE DERP connection and homes it on its **priority peer's (the
  fleet's) DERP region**; a DERP server only relays between peers on the **same**
  region. Machines and remotes both home on the fleet region, so cross-site relay
  works. If a link only comes up after a `tailscale ping` from a full client, the
  region homing is off — see `[[reference_pstop_derp_region]]` (fixed in firmware).
- Same-LAN remote+machine upgrade to a **direct** path automatically; judge
  direct-vs-DERP by `wg_direct`/`tailscale status`, **not** by `rtt_ms` (the pstop
  loop RTT is hold-time inflated).

## 7. Troubleshooting: "remote won't bond to the machine"

Symptom on the remote: `pstop_machines[slot]` `state=1`, `sent=0`, `send_fail`
climbing, `errno 128`; the remote's `/admin/api/peers` *has* the machine but the
machine's `/admin/api/peers` does **not** have the remote.

1. **Is the tailnet > 128 nodes?** (`/admin/api/peers` length = 128 on both = at the
   cap.) If so → peer-cap trim. Fix: **make the remote an operator on the machine**
   (`POST /api/operators?add=<id>`) so the machine pins it, or prune the tailnet
   below 128, or co-locate them on one LAN (direct path, no relay).
2. **Different failure? `sf_route`≠0 / errno 118** = no route → machine peer not in
   the remote's netmap or WG down → check the remote's `/admin/api/monitor`
   (`derp_home_region`, `wg_paused`, `ts_boot_en`). `wg_paused=1`/`ts_boot_en=0` =
   boot-safety pause from crash-boots → `POST /api/ts_boot` then a clean
   `POST /admin/api/restart` (not a power-cycle). See `[[reference_pstop_derp_region]]`.
3. **After adding an operator,** the machine keeps the remote on its next netmap
   sync (seconds). A machine reboot re-reads operators from NVS and re-pins them.

## 8. Reference

- Authorization / arming: `docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`, `docs/API.md`.
- Machine node: `docs/MACHINE_ESP32_DESIGN.md`. Multi-machine API: `docs/API.md`.
- Peer-cap fix: `components/microlink/src/ml_wg_mgr.c` (`s_extra_pins`, peer-wanted
  hook), `machn/main/main.c` (operator match). Safety case: HARA H-13 / SR-SYS-09.
