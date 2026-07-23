# Same-LAN Direct-Path Fix — Plan (Option B)

**Status:** scoping complete, no code written yet (beyond `local_ip` in
`state.json`). Written 2026-07-22.

**Goal:** when a pstop remote and its machine are on the same LAN, the
WireGuard link should upgrade from a DERP relay to a **direct, still-encrypted**
UDP path automatically — with the operator still configuring only the Tailscale
IP. This is "Option B" from the same-site design discussion: fix Tailscale's own
direct-path formation, *not* a plaintext raw-LAN bypass.

---

## 1. Symptom

Two devices on the same LAN (shared public IP) never form a direct WG path; they
stay relay-only through DERP, or — when the relay path is also degraded — never
connect at all. The remote's peer table shows the machine as
`direct: False, allowed: True`, `pstop_sent` never advances, the ring pulses red.
Observed repeatedly on the bench; most recently remote `pstop-01d7f498`
(100.122.25.95, Ethernet) unable to bond to a framework laptop on the same LAN.

This has been a **recurring** problem. The purpose of this plan is to fix the
actual root cause once, with empirical before/after proof, rather than patch a
symptom.

---

## 2. Root cause (evidence-grounded)

### 2.1 Primary bug — the chip advertises no LAN endpoint off WiFi

Tailscale upgrades a relayed path to direct by racing disco **ping/pong** across
a peer's candidate endpoints. Candidates come from three sources: STUN public,
**local interface (LAN)**, and port-mapped. Same-LAN pairs go direct *only*
because each side probes the other's **local** endpoint (e.g. `192.168.x.x`).
(Refs: Tailscale disco docs; DeepWiki "Endpoint Discovery and NAT Traversal".)

microlink advertises its own LAN endpoint in two places, and **both read the LAN
IP only from `WIFI_STA_DEF`**:

- `ml_coord.c:1283` — `add_endpoints_to_json()` (MapRequest / endpoint-update to
  the control plane).
- `ml_wg_mgr.c:1632` — `disco_send_call_me_maybe()` (CallMeMaybe to peers).

On the **W5500 Ethernet** and **USB-NCM** transports — the primary production
uplinks — `esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")` returns NULL, so the
chip advertises **only its STUN public endpoint**. A same-LAN peer therefore
never learns the chip's real LAN address, cannot probe it, and cannot open a
direct path back to it. The pair is stuck on the shared public IP (needs NAT
hairpin, usually unsupported) → DERP relay only.

This is finding #1 of the code survey and is a plain bug, not a tuning issue.

### 2.2 The rest of the direct-path machinery is healthy

- **disco is implemented**: PING `0x01` / PONG `0x02` / CALL_ME_MAYBE `0x03`,
  NaCl-boxed with disco keys (`ml_wg_mgr.c:110-116`). The chip pings *all* of a
  peer's known IPv4 endpoints (`disco_send_ping_to_peer:1049-1071`), replies
  pongs preferring LAN endpoints (`process_disco_ping:1171-1195`), and flips
  `has_direct_path=true` the moment a pong arrives over UDP with `via_derp==false`
  (`process_disco_pong:1234-1303`), rewriting the WG endpoint to the direct
  address. The data path uses direct iff `peer->ip`/`peer->port` are set
  (`wireguardif.c:148-223`).
- **disco is enabled by default**: `.enable_disco = true` (`ml_app.h:72`,
  `ml_net_switch.c:287`); no NVS override forces it off.
- **The direct-UDP wedge is fixed**. Historically `enable_disco` was reverted to
  false because sustained direct-UDP traffic panicked the chip ~100–180 s in.
  Root cause (v15.20, `docs/archive/TAILSCALE_FINAL_2026-05-26.md:274-334`):
  lwIP core APIs were called off the TCPIP thread, corrupting pbuf/netif state.
  Fixed with `CONFIG_LWIP_TCPIP_CORE_LOCKING=y` +
  `LOCK_TCPIP_CORE()`/`UNLOCK_TCPIP_CORE()` wraps. **Both are present in the
  current tree** (`firmware/sdkconfig.defaults:155`; 9 wrap sites in
  `ml_wg_mgr.c`). v15.20 soaked 22+ min under direct-UDP load with no wedge.

### 2.3 Corroboration

Every bench observation where the direct path **did** form was on an interface
that *does* advertise a LAN endpoint:
`via 192.168.107.131:51820` (WiFi) and `via 10.42.0.80:51820` (USB-NCM
point-to-point, where the peer also learns the addr from the ping source on the
shared L2). The failing case is exactly the Ethernet/same-subnet topology where
the advertisement is empty. `docs/PEER_SCALING_DESIGN.md:108-116` independently
notes same-public-IP topologies fall back to DERP on the bench.

### 2.4 Why it kept recurring (so we don't circle again)

Past direct-path work happened over WiFi or USB-NCM, where a LAN endpoint was
either advertised (WiFi) or learned from the same-L2 ping source (USB). The
Ethernet + shared-subnet case — no WiFi netif, not point-to-point — was never the
bench target, so the `WIFI_STA_DEF`-only advertisement was never exercised as the
gap it is. The fix must therefore be **validated specifically on Ethernet with
both devices on the same subnet**, not on WiFi/USB.

---

## 3. The fix

### 3.1 Advertise the active uplink's LAN endpoint (core change)

Replace the `WIFI_STA_DEF`-only lookup in both sites with the **active uplink's**
LAN IPv4. Add one helper in microlink:

```c
/* Returns the active physical uplink's LAN IPv4 (host order), or 0.
 * The active uplink is the up, non-tunnel esp_netif with the default route
 * (highest route_prio) and a valid IPv4 — i.e. whatever eth/usb/wifi is live. */
uint32_t ml_active_lan_ip(void);
```

Implementation options, in order of preference:
1. `esp_netif_get_default_netif()` → `esp_netif_get_ip_info()` — returns the
   current default-route netif = active uplink. **Must verify it does not return
   the WG tunnel netif** (the WG netif is a plain lwIP netif, not an esp_netif,
   so it should be excluded — confirm during implementation).
2. Fallback / robustness: iterate esp_netifs (`esp_netif_next_unsafe`), pick the
   up interface with the highest `esp_netif_get_route_prio()` that has a non-zero
   IPv4. Fall back to `WIFI_STA_DEF` if all else fails.

Use `ml_active_lan_ip()` at both `ml_coord.c:1283` and `ml_wg_mgr.c:1632`;
port stays `ml->disco_local_port` (the magicsock/WG listener — proven reachable
per interface by the §2.3 bench evidence).

**Keep the coupling inside microlink** — do not reach for dcs_support's
`active_iface` enum or the `"DCS_ETH"` ifkey; `esp_netif` route priority is the
generic, decoupled signal.

### 3.2 Re-advertise on interface / IP change

On failover the LAN IP changes, so stale endpoints must be refreshed. Verify /
ensure that an `active_iface` change (or a netif `IP_EVENT_*_GOT_IP`) triggers
`do_send_endpoint_update()` and a CallMeMaybe to the priority peer + fleet. If no
such trigger exists, add one. Without this, a post-failover unit advertises the
wrong LAN endpoint until the next periodic update.

### 3.3 Optional hardening (decide during implementation, keep out of the first cut if it adds risk)

- Advertise **all** up physical interfaces' LAN IPs (Tailscale advertises every
  local interface address), not just the active one — marginally more robust
  across failover races, at the cost of advertising possibly-unreachable
  endpoints. Default recommendation: ship the active-uplink version first; add
  multi-interface only if validation shows a failover gap.
- IPv6 endpoint parse: the netmap endpoint parser is IPv4-only
  (`ml_coord.c:1138,1245`). Out of scope for the same-LAN (IPv4) fix; note only.

### 3.4 What we deliberately do NOT change

- **The passive/lazy WG-handshake policy** (`ml_wg_mgr.c:770-806`) — untouched.
  We are only fixing endpoint *advertisement/discovery*; the handshake-initiation
  logic already handles a direct endpoint once discovery succeeds.
- **The pstop data socket / source-binding / fail-safe-on-VPN-down** — untouched.
  This is still WG-encrypted transport; no plaintext bypass (that was Option A).
- **`pstop_c`** (submodule) and the lockstep safety core — untouched.
- **The v15.19 graded boot-count safety ladder** — stays as defense-in-depth.

---

## 4. Risks & mitigations

| Risk | Assessment | Mitigation |
|---|---|---|
| Re-triggering the direct-UDP wedge | LOW — root-caused + fixed in v15.20 (core locking present in tree). This change only *enables discovery to succeed on eth*, exercising the same data path that already soaked 22+ min clean. | Sustained-load soak on the fixed direct path (§5) before shipping. The boot-count ladder still auto-falls-back to DERP if a wedge recurs. |
| Plaintext exposure | NONE — direct path is still full WireGuard encryption. This is the whole reason to prefer B over A. | n/a |
| Advertising an unreachable/wrong LAN endpoint (e.g. after failover) | MED | §3.2 re-advertise on change; disco simply gets no pong from a dead endpoint and stays on DERP — fail-safe, not fail-dangerous. |
| Safety-path regression | LOW — change is confined to disco endpoint advertisement, off the pstop encode/compare/tx path. | MISRA sweep; confirm pstop heartbeat rate unchanged in soak. |
| More peers going direct increases per-packet work | LOW — direct is *cheaper* than DERP (no TLS relay hop). | Monitor heap floor + task stacks in soak. |

---

## 5. Validation plan

### 5.1 Pre-implementation — confirm the root cause on the real failing pair

Before coding, prove the diagnosis empirically on an Ethernet same-subnet pair
(e.g. the 100.122.25.95 remote + a same-LAN machine):

1. On the remote (eth): watch the boot log / `MapRequest includes N endpoint(s)`
   — expect **1** (STUN only), never a LAN endpoint. Confirms §2.1 on hardware.
2. `GET /admin/api/peers` on the remote — confirm the machine peer has
   `direct:false` and inspect whether its endpoint list includes the machine's
   LAN `192.168.x.x` (it should, from the netmap — that's the half that works).
3. On a full-tailscaled machine on the same LAN: `tailscale status --json` /
   `tailscale ping <remote>` — confirm it has **no LAN endpoint** for the remote
   and pongs only `via DERP`.

If 1–3 hold, the advertisement gap is confirmed as *the* cause and we proceed.
If the machine *does* already have the remote's LAN endpoint yet still can't go
direct, stop — the problem is environmental (AP/client isolation, subnet
routing) and no firmware change will help; document and switch that site to
Option A (LAN-IP peer) instead.

### 5.2 Post-implementation — functional

1. Flash the fix to the eth remote; repeat 5.1.1 — expect **2** endpoints
   (LAN + STUN) and a `CMM endpoint: LAN 192.168.x.x` log line.
2. `tailscale ping <remote>` from the same-LAN machine → expect
   `pong ... via 192.168.x.x:51820` (direct), not `via DERP`.
3. `GET /admin/api/peers` → machine peer `direct:true`; `pstop_sent`/`replies`
   advance at 10 Hz; ring goes blue→green on arm.
4. Repeat on **USB-NCM** and **WiFi** to confirm no regression on the transports
   that already worked.

### 5.3 Post-implementation — soak / safety

- 15–30 min direct-UDP soak on eth: no wedge, `bc` stays 0, heap floor stable,
  task stacks stable, pstop reply rate ≥ baseline.
- Failover test: force eth→usb→wifi while bonded; confirm the endpoint
  re-advertises and the path recovers (direct or DERP) with fail-safe behavior.
- Cross-site (different-network) pair: confirm DERP still works and nothing
  regressed for the non-same-LAN case.

---

## 6. Scope, non-goals, open questions

**In scope:** active-uplink LAN endpoint advertisement (both sites) +
re-advertise on interface change + validation.

**Non-goals:** IPv6 endpoints; raw-LAN plaintext bypass (Option A); any change to
handshake policy, pstop datapath, or pstop_c; DERP-homing/self-heal (orthogonal —
`ml_wg_mgr.c:530-564`).

**Open questions to resolve in implementation:**
- Does `esp_netif_get_default_netif()` ever return the WG tunnel netif? (Expected
  no; verify.)
- Is there already an endpoint re-advertise trigger on `active_iface` change, or
  must one be added? (§3.2.)
- Is the magicsock/WG UDP socket reachable at `disco_local_port` on the **eth**
  iface specifically? (Proven for wifi/usb; verify for eth in 5.2.2.)

---

## 7. Validation results (2026-07-22)

Implemented and bench-validated on two local units against a machine on this
host. **The fix is correct and works; the validation also surfaced that the
dominant real-world blocker is environmental, not firmware.**

**Fix confirmed working (device side):**
- Ethernet unit `pstop-01d7791c`: `advert_lan_ip = 192.168.107.203` (its eth
  LAN IP — previously it advertised nothing off WiFi). It learns and selects
  the machine's LAN endpoint: `pp_best_ip = 192.168.107.236`, `pp_has_direct =
  true`; `tailscale status` shows `direct 192.168.107.203:51820` with MB of
  data flowing.
- USB-NCM unit `pstop-01d778b4`: `advert_lan_ip = 10.42.0.106`, `pp_best_ip =
  10.42.0.1` (direct), `tailscale ping` 14 ms. Confirms `ml_active_lan_ip()`
  resolves the active uplink on eth **and** usb, not just WiFi.

**Three traps that nearly caused a wrong conclusion (recorded so we don't
re-run them):**
1. **`rtt_ms` is not a direct/DERP signal.** It read a steady ~108 ms on a
   ~13 ms direct path — a known counter-lag inflation (`main.c:619`). Judge the
   path with `tailscale status` (direct/relay), not `rtt_ms`.
2. **The real same-LAN blocker on the bench is a Tailscale subnet-router route
   hijack on the *machine host*:** `ip route get <chip-LAN-IP>` returned `dev
   tailscale0` because a subnet router advertises `192.168.107.0/24` into the
   tailnet. The direct path flapped (13→244 ms) until an `ip rule ... to
   <chip-LAN-IP> lookup main` exception was added (matching pre-existing
   exceptions for other bench IPs). This is host/network config, not firmware —
   see `TROUBLESHOOTING.md`. It is likely what bit the original framework case.
3. **`Endpoints None` in `tailscale status` is a control-plane redistribution
   quirk** of this tailnet, not a signal about what the chip advertises. Verify
   advertisement with the device's `advert_lan_ip`, not the peer's netmap.

**Honest scope of the benefit:** on the steady-state pstop bond with an
actively-probing priority peer and a full-tailscaled machine, the direct path
often forms even without the advertisement (disco learns the peer addr from the
pong source). The fix's value is correctness + robustness: peer-initiated
reachability, faster/more-reliable mutual discovery, and not depending on
disco-source-learning. It does **not** overcome environmental blockers
(subnet-router hijack, AP/client isolation) — those are host/network config.

**Build hygiene:** the repo lives under Nextcloud; sync rewrote source mtimes
and the build silently reused stale objects (green build, fix absent from the
binary, `-Werror` skipped). Always `touch` edited files and gate the flash on
`strings build/pstop_remote.elf | grep <unique-string>` before OTA.

## 8. Rollback

Single-commit, self-contained change. If validation regresses, revert the
advertisement helper + call sites; disco falls back to the prior
WiFi-only-advertise behavior (i.e. today's DERP-relay same-LAN result). The
boot-count ladder provides runtime fail-safe independent of this change.
