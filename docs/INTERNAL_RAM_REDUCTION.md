# Internal-RAM Reduction — Research & Design (2026-07-21)

## Problem

ESP32-S3, 8 MB PSRAM (plentiful), but **internal RAM (DRAM / `MALLOC_CAP_INTERNAL`)
runs 5–13 KB free**. The **DERP TLS (re)connect fails** under this pressure: after
any DERP drop or a DERP re-home, `mbedtls_ssl_handshake` can't allocate its working
buffers → `derp_connected` stays false → the chip can't relay to the fleet server
(or to the safety peer when the direct WG path is down).

## Root cause (the important finding)

**`CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y`** — mbedTLS allocates all TLS *working*
buffers from **internal** RAM. A DERP handshake (`ml_derp.c:830,842`) demands the
RX record buffer (grows toward `MBEDTLS_SSL_IN_CONTENT_LEN=16384`), the TX buffer
(`OUT_CONTENT_LEN=4096`), and handshake scratch — **~20–30 KB internal, transiently,
exactly when only 5–8 KB is free**. It OOMs.

Not the task stacks (my earlier hypothesis). The mbedTLS *context structs* are
already in PSRAM (`microlink_t` is `ml_psram_calloc`'d); only the mbedTLS-internal
`calloc`'d working buffers are governed by this knob. WireGuard/pstop datapath
crypto is **not** mbedTLS (own ChaCha20/Poly1305/Curve25519), so this knob does not
touch the safety heartbeat crypto — only TLS (DERP, HTTPS).

## Design — phased, safety-first

### Phase 1 — the fix (one line, low risk) ✅ recommended now
`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y` (prereq `CONFIG_SPIRAM_USE_MALLOC=y` already
set). Moves TLS working buffers to PSRAM → the DERP handshake stops competing for
the last few KB of internal RAM.
- **Risk:** LOW–MOD. TLS runs in the derp_tx/net_io tasks, not ISR/cache-disabled.
  HW AES/SHA DMA from PSRAM is supported by IDF (esp-tls uses it widely); crypto is
  slightly slower, but DERP only carries tiny 10 Hz pstop frames — a *working*
  reconnect beats an OOM. Does NOT affect the WG/pstop datapath crypto.
- **Validate:** reproduce the low-internal-RAM condition, force a DERP drop, confirm
  `derp_connected` returns true and a fleet check-in lands.

### Phase 1b — NCM NTB buffers 4+4 → 2+2 (2026-07-22) ✅ done
TinyUSB NCM NTB buffers are DMA-pinned to internal SRAM. On the USB-NCM tether
the DERP TLS handshake still starved internal RAM to ~2.8 KB and OOM'd even with
Phase 1; dropping the NTB counts 4+4 → 2+2 (`sdkconfig.defaults`) returns ~6 KB
and the handshake low-water rose to ~15 KB. Negligible throughput impact on this
low-rate fallback link.

### Phase 2 — NOT SAFE as scoped (findings on implementation, 2026-07-21)
Both candidate tasks perform **flash operations**, which violate the PSRAM-stack
contract (a task with a PSRAM stack faults when the cache is disabled during a
flash write):
- **fleet_ota** calls `esp_ota_write` (`ml_app.c:577`) — PSRAM stack would fault
  mid-OTA-write, breaking OTA itself. **Must stay internal.**
- **ml_coord** invokes the app state-callback (`ml_coord.c:2416`) which calls
  `esp_ota_mark_app_valid_cancel_rollback()` (`ml_app.c:308`) on the coord stack —
  PSRAM stack would boot-loop on first connect and/or defeat rollback protection.
  Would require decoupling the mark-valid off the coord task (into an
  always-present internal context) — a risky change to the OTA-validity/rollback
  chain.

**Decision:** SKIP Phase 2. Phase 1 already raised internal free ~13 KB → ~32 KB,
which resolves the DERP-reconnect OOM with ample margin. The ~12–20 KB extra is not
worth risking the OTA/rollback safety chain. Revisit only if a future feature
re-pressures internal RAM, and then prefer Phase 3 (WiFi) or a proper mark-valid
decouple over PSRAM-stacking flash-op tasks.

### Phase 3 — biggest reservoir (design decision, deferred)
WiFi driver static buffers ≈ **40–50 KB internal, DMA-locked (cannot go to PSRAM)**.
Options: don't start WiFi when USB-NCM/Ethernet is the live link (lazy-start only for
the AP config portal); or trim `ESP_WIFI_STATIC_TX_BUFFER_NUM` 16→4–6,
`STATIC_RX_BUFFER_NUM` 10→6, `RX_BA_WIN` 16→6, TX→dynamic. **Risk:** MOD — WiFi is a
live STA transport + AP-fallback config portal; this is a product decision, so it's
deferred out of this pass.

## Explicitly NOT touched (safety / hardware constraints)
- **ml_wg_mgr stack (16 KB)** — THE pstop heartbeat datapath (full WG
  decrypt→ip_input→encrypt→udp_sendto chain per packet; 8 KB tripped the canary).
  Cannot shrink, cannot go to PSRAM (adds per-packet latency/jitter to the safety
  path).
- **ml_net_io stack (8 KB)** — core packet RX/TX hot path, keep internal.
- **WiFi/lwIP DMA buffers** — HW DMA can't source from PSRAM on S3.

## Already correct (keep)
`MBEDTLS_DYNAMIC_BUFFER=y` (+ free CA/config after handshake);
`SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`; `ml_psram_malloc/calloc` routing the DERP frame
buf, all coord H2/JSON buffers (incl. 512 KB H2 recv), nacl cache, and the WireGuard
device to PSRAM; `SPIRAM_MALLOC_RESERVE_INTERNAL=32768`, `ALWAYSINTERNAL=2048`.

## Non-findings (corrected)
- `crt_bundle_attach` on the plain-HTTP fleet clients is a **no-op** (esp-tls not
  invoked for `http://`) — no internal-RAM win; cosmetic cleanup only.

## Plan
Implement **Phase 1** and validate the DERP-reconnect fix under low-internal-RAM.
If headroom is still marginal, add **Phase 2**. Hold **Phase 3** for a product call.
