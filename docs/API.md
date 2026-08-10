# pstop remote — HTTP API

The remote firmware serves an HTTP interface on **port 80** over any active
link (Ethernet, USB-NCM tether, WiFi, or Tailscale). There are two route
groups on the same server:

- **Diagnostic / config routes** — unauthenticated.
- **Admin routes** (`/admin/...`) — HTTP Basic auth (`admin` : `CONFIG_ML_ADMIN_PASSWORD`).

Query parameters are shown where required. Unless noted, POST bodies are empty.

## Diagnostic / config (unauthenticated)

| Method | Route | Purpose |
|--------|-------|---------|
| GET  | `/` | Diagnostic HTML page |
| GET  | `/state.json` | Telemetry snapshot (uptime, ml_state, pstop counters, heap, E-stop channels, `public_ip`/`derp_region` for management-side geolocation (`derp_region` = the effective DERP home region in use; `derp_region_locked` = the configured region lock, `0` = auto, else the pinned region — set via `POST /admin/api/settings` `derp_region`), `local_ip` = active-uplink LAN address, `fw_ver`/`fw_sha` = running build identity — version tag + truncated ELF SHA-256 — so test harnesses can verify the DUT runs the build under test) |
| GET  | `/api/last_log` | Tail of the previous boot's log (RTC ring) |
| POST | `/api/derp` | Toggle the DERP TX worker |
| POST | `/api/derp_delay?ms=N` | Set the DERP loop yield (ms) |
| POST | `/api/wg` | Suspend / resume the WireGuard task |
| POST | `/api/wifi_tx_power?q=N` | Set WiFi max TX power (quarter-dBm) |
| POST | `/api/iface/eth` | Select the Ethernet uplink |
| POST | `/api/iface/wifi` | Select the WiFi uplink |
| POST | `/api/iface/usb` | Select the USB-NCM uplink |
| POST | `/api/usb_enable` | Flip the USB-NCM NVS flag and reboot |
| POST | `/api/ts_boot` | Flip the Tailscale-on-boot NVS flag (effective next reboot) |
| POST | `/api/pstop_peer?ip=A.B.C.D&port=N` | Set + persist the pstop machine target (= peer slot 0, legacy single-machine call) |
| POST | `/api/pstop_peers?slot=N&ip=A.B.C.D&port=P[&id=HEX]` | Multi-machine peer table: set slot 0..3 (id = the machine's `machine_device_id`, default `0x01020304`). `?slot=N&clear=1` empties a slot. Applies live within one 100 ms tick, persists to NVS |
| POST | `/api/pstop_num?n=N` | Set the USB "PSTOPxx" unit number (0 = auto) |
| POST | `/api/ring_offset?n=N` | Set + persist the LED-ring rotation offset (0..15) — which physical pixel is "LED 1". Applies immediately, survives reboots and firmware updates (NVS `ring_off`) |
| POST | `/api/ring_led1?on=0\|1` | Locate mode: light ONLY LED 1 solid white (overrides state colours) so the offset can be verified during install; auto-expires after 5 min |
| POST | `/api/enter_download` | Enter USB download (flashing) mode |

### Multi-machine (one remote, up to 4 machines)

The remote heartbeats every configured peer slot independently: per-machine
socket (local ports 8891..8894, slot order), per-machine bond/counter state
and reply-loss watchdog, so one dead machine never stalls the heartbeats to
the others. The lockstep comparator still gates ALL transmissions: a core
mismatch silences every session at once. A STOP (and the arming
press-and-release) is broadcast — one operator gesture arms every bonded
machine; each machine keeps enforcing its own `min_stop_ms` veto.

The machine governs each session's publish rate: pstop_c advertises the
machine's per-operator `heartbeat_ms` (machine.toml) in every reply's
`heartbeat_timeout` field, and the remote transmits at half that window
(clamped 100..1000 ms), advancing its counter only on transmitting ticks so
the machine sees contiguous counters. The E-stop sampling stays at the fixed
10 Hz lockstep tick regardless.

`/state.json` carries the per-machine detail in `pstop_machines`: an array
of all 4 slots (stable indices) with `cfg`, `ip`, `port`, `id`,
`state` (0 idle / 1 bonding / 2 bonded), `sent`, `replies`, `send_fail`,
`rebonds`, `rtt_ms`, `hb_ms` (the machine-requested heartbeat window),
`last_msg`, `last_reply_ms`. The legacy scalar
`pstop_*` fields remain as aggregates (worst-of `last_msg`, summed counters,
most recent reply) for old tooling.

The LED ring divides evenly among the configured slots, in slot order
starting at LED 1 (one machine = whole ring, matching the old display);
each segment shows its machine's state with the usual colours. HIL
validation: `tools/pstop_multi_machine_test.py`.

### LED-ring rotation (provisioning)

The 16-LED ring can be installed in any of 16 orientations, so "LED 1" is a
per-device setting. To calibrate from your provisioning tooling:

1. `POST /api/ring_led1?on=1` — one white pixel shows where the current offset
   puts LED 1 (current offset and state are in `/state.json` as `ring_offset` /
   `ring_led1`).
2. `POST /api/ring_offset?n=N` — the white pixel moves on the next repaint
   (≤250 ms); iterate until it sits on the bezel's LED-1 position. Pixel
   indices run in WS2812 data order (physical pixel `(logical + n) mod 16`).
3. `POST /api/ring_led1?on=0` — return the ring to the normal state display
   (it also auto-expires after 5 minutes as a safety backstop).

## Admin (`admin` : `CONFIG_ML_ADMIN_PASSWORD`)

| Method | Route | Purpose |
|--------|-------|---------|
| GET    | `/admin/` | Config panel (HTML) |
| GET    | `/admin/api/status` | Node status |
| GET    | `/admin/api/settings` | Read settings |
| POST   | `/admin/api/settings` | Update settings. The `derp_region` field (`0`..`4095`, `0` = auto) locks the DERP home region: it persists to NVS **and** applies live (updates the runtime override + kicks a slot-0 re-home, no reboot). Locking guards the priority path against a region misroute (e.g. region-9/dfw). Example: `{"derp_region":2}` locks to sfo; `{"derp_region":0}` clears to auto |
| GET    | `/admin/api/monitor` | Heap / DERP / per-task monitor + DERP re-home diagnostics (`derp_home_region`, `fleet_peer_region`, `rehome_*` counters) + same-LAN direct-path diagnostics (`advert_lan_ip`, `pp_has_direct`, `pp_best_ip`/`pp_best_port`, `pp_endpoints`) |
| GET    | `/admin/api/peers` | Active WireGuard peer table |
| GET    | `/admin/api/peers/allowed` | Read the peer allowlist |
| POST   | `/admin/api/peers/allowed` | Add an allowed peer (the configured management server, if any, is non-removable) |
| DELETE | `/admin/api/peers/allowed` | Remove an allowed peer |
| POST   | `/admin/api/restart` | Reboot the device |
| GET    | `/admin/api/wifi` | WiFi scan / status |
| POST   | `/admin/api/wifi` | Set WiFi credentials |
| POST   | `/admin/api/ota` | Direct firmware upload (OTA) |
| GET    | `/admin/api/ota/status` | Running version + OTA image state |
| GET    | `/admin/api/fleet-ota/status` | Managed-OTA status |
| POST   | `/admin/api/fleet-ota/check` | Trigger a check-in against the OTA backend (pull if an update is assigned) |
| POST   | `/admin/api/fleet-ota/toggle` | Toggle auto-update |
| POST   | `/admin/api/fleet-ota/interval` | Set the check-in interval |
| GET    | `/admin/api/verbose` | Verbose-logging status |
| POST   | `/admin/api/verbose/toggle` | Toggle verbose logging |

The `/admin/api/fleet-ota/*` endpoints and the periodic check-in are
intended to pair with a centralized configuration management and OTA
system of your own (configured via `CONFIG_ML_FLEET_SERVER_IP` /
`CONFIG_ML_OTA_BACKEND_URL`); no such backend is included in this
project. With no backend configured, the direct `/admin/api/ota` upload
is the update path.

## Machine (`machn`) — operator authorization

> These routes and `/state.json` fields belong to the ESP32 **machine** unit
> (`machn/`), not the remote documented above. They are grouped here as the
> single HTTP reference. (Endpoints are being added alongside this doc.)

A machine **accepts every remote that bonds** but by default treats it as
**stop-only** (`stop_only=true`): the remote may command STOP and is
heartbeat-monitored (its silence forces a fail-safe STOP), but it can **never**
re-arm the machine (STOP→OK). Only remotes on the machine's **operator
allowlist** (`stop_only=false`) may re-arm. **The operator allowlist is empty by
default** — out of the box every remote is stop-only (maximally safe); operator
remote IDs are added during commissioning. Enforced in `pstop_c`
(`machine.c:135-142`). Safety case: HARA H-13, SR-SYS-09, FMEA H02-4.

All routes require admin Basic-auth (enforced in-handler, same credential as `/api/enter_download`).

| Method | Route | Purpose |
|--------|-------|---------|
| GET    | `/api/operators` | List the operator allowlist — the remote device IDs permitted to re-arm |
| POST   | `/api/operators?add=ID` | Add a remote device ID (hex `0x..`/`..` or decimal) to the operator allowlist (that remote becomes `stop_only=false`). Persisted to NVS |
| POST   | `/api/operators?del=ID` | Remove a remote from the allowlist (it reverts to stop-only). Persisted to NVS |

The machine's `/state.json` exposes the authorization state: an `operators`
array (the configured operator device IDs) and, per bonded remote, a `stop_only`
boolean — `true` = accepted, stop-capable, heartbeat-monitored, re-arm-**IN**capable;
`false` = full operator (stop **and** re-arm). A remote absent from `operators`
reads `stop_only=true`.

## Source of truth

- Diagnostic/config routes: `firmware/components/dcs_support/src/dcs_admin_pages.c`
- Admin routes: `components/microlink/src/ml_config_httpd.c` (`/admin` prefix) and
  `components/microlink/src/ml_app.c` (managed OTA, verbose).

The app server's user-handler budget is set in `dcs_support.c`
(`cfg.max_user_uri_handlers`); adding routes beyond it fails silently, so bump
it when adding new `/api` handlers.
