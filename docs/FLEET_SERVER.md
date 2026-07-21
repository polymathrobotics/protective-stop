# Fleet Coordination Server

`100.92.1.78` is the central server that updates and coordinates the pstop
fleet (fleet OTA + coordination). It is treated specially by the firmware.

## Permanent, non-removable allowlist peer

The fleet server is **always allowed** and **cannot be removed** from a remote's
peer allowlist. Every other peer remains user-removable/editable as normal.

- Baked in via `CONFIG_ML_FLEET_SERVER_IP` (default `100.92.1.78`,
  `components/microlink/Kconfig`).
- Enforced in `components/microlink/src/ml_config_httpd.c`:
  - `ml_config_peer_is_allowed()` returns `true` for the fleet IP before any
    filter logic — so it is reachable even with the allowlist filter on and the
    IP absent from the saved list.
  - `POST /api/peers/allowed` re-injects the fleet IP if a client omits it, so
    it always persists in the stored allowlist.

Rationale: the fleet server must always be able to reach every remote to push
updates and coordinate; a misconfigured or hostile allowlist edit must never be
able to strand a unit from central management.

## Fleet OTA build configuration

Build the remote firmware pointed at the fleet backend:

```
CONFIG_ML_OTA_BACKEND_URL="http://100.92.1.78"     # baked into sdkconfig.defaults
CONFIG_ML_OTA_API_KEY=<FLEET_API_KEY>              # secret → sdkconfig.credentials
```

`CONFIG_ML_OTA_BACKEND_URL` is committed in `firmware/sdkconfig.defaults`.
`CONFIG_ML_OTA_API_KEY` is a secret and belongs only in
`firmware/sdkconfig.credentials` (git-ignored) — copy `FLEET_API_KEY` from
`pstop-fleet/containers/pstop-fleet.env`. See
`firmware/sdkconfig.credentials.example`.
