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
| GET  | `/state.json` | Telemetry snapshot (uptime, ml_state, pstop counters, heap, E-stop channels) |
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
| POST | `/api/pstop_peer?ip=A.B.C.D&port=N` | Set + persist the pstop machine target |
| POST | `/api/pstop_num?n=N` | Set the USB "PSTOPxx" unit number (0 = auto) |
| POST | `/api/enter_download` | Enter USB download (flashing) mode |

## Admin (`admin` : `CONFIG_ML_ADMIN_PASSWORD`)

| Method | Route | Purpose |
|--------|-------|---------|
| GET    | `/admin/` | Config panel (HTML) |
| GET    | `/admin/api/status` | Node status |
| GET    | `/admin/api/settings` | Read settings |
| POST   | `/admin/api/settings` | Update settings |
| GET    | `/admin/api/monitor` | Heap / DERP / per-task monitor |
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
