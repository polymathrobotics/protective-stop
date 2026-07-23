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
| GET  | `/state.json` | Telemetry snapshot (uptime, ml_state, pstop counters, heap, E-stop channels, `public_ip`/`derp_region` for fleet-side geolocation, `local_ip` = active-uplink LAN address) |
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

`/state.json` carries the per-machine detail in `pstop_machines`: an array
of all 4 slots (stable indices) with `cfg`, `ip`, `port`, `id`,
`state` (0 idle / 1 bonding / 2 bonded), `sent`, `replies`, `send_fail`,
`rebonds`, `rtt_ms`, `last_msg`, `last_reply_ms`. The legacy scalar
`pstop_*` fields remain as aggregates (worst-of `last_msg`, summed counters,
most recent reply) for old tooling.

The LED ring divides evenly among the configured slots, in slot order
starting at LED 1 (one machine = whole ring, matching the old display);
each segment shows its machine's state with the usual colours. HIL
validation: `tools/pstop_multi_machine_test.py`.

### LED-ring rotation (fleet setup)

The 16-LED ring can be installed in any of 16 orientations, so "LED 1" is a
per-device setting. To calibrate from the fleet-setup tooling:

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
| POST   | `/admin/api/settings` | Update settings |
| GET    | `/admin/api/monitor` | Heap / DERP / per-task monitor + DERP re-home diagnostics (`derp_home_region`, `fleet_peer_region`, `rehome_*` counters) + same-LAN direct-path diagnostics (`advert_lan_ip`, `pp_has_direct`, `pp_best_ip`/`pp_best_port`, `pp_endpoints`) |
| GET    | `/admin/api/peers` | Active WireGuard peer table |
| GET    | `/admin/api/peers/allowed` | Read the peer allowlist |
| POST   | `/admin/api/peers/allowed` | Add an allowed peer (fleet server is non-removable) |
| DELETE | `/admin/api/peers/allowed` | Remove an allowed peer |
| POST   | `/admin/api/restart` | Reboot the device |
| GET    | `/admin/api/wifi` | WiFi scan / status |
| POST   | `/admin/api/wifi` | Set WiFi credentials |
| POST   | `/admin/api/ota` | Direct firmware upload (OTA) |
| GET    | `/admin/api/ota/status` | Running version + OTA image state |
| GET    | `/admin/api/fleet-ota/status` | Fleet OTA status |
| POST   | `/admin/api/fleet-ota/check` | Trigger a fleet check-in (pull if assigned) |
| POST   | `/admin/api/fleet-ota/toggle` | Toggle fleet auto-update |
| POST   | `/admin/api/fleet-ota/interval` | Set the fleet poll interval |
| GET    | `/admin/api/verbose` | Verbose-logging status |
| POST   | `/admin/api/verbose/toggle` | Toggle verbose logging |

## Source of truth

- Diagnostic/config routes: `firmware/components/dcs_support/src/dcs_admin_pages.c`
- Admin routes: `components/microlink/src/ml_config_httpd.c` (`/admin` prefix) and
  `components/microlink/src/ml_app.c` (fleet-ota, verbose).

The app server's user-handler budget is set in `dcs_support.c`
(`cfg.max_user_uri_handlers`); adding routes beyond it fails silently, so bump
it when adding new `/api` handlers.
