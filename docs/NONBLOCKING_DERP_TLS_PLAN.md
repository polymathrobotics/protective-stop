# Non-Blocking / Async DERP TLS Handshake Refactor — Implementation Plan

Branch: `derp/relay-recovery` (HEAD 24d5ae1) · ESP-IDF 5.5 · target machn (ESP32-S3)
Scope: `components/microlink` ONLY. **Does NOT touch `pstop_c`** (safety-certified).
This is a reliability fix (fail-safe nuisance-stop reduction), not a safety change.

---

## 0. TL;DR

- **Root mechanism (run-26):** a FORCED home-region switch (`POST /admin/api/settings
  {derp_region:N}`) is a **break-before-make** reconnect. `ml_config_httpd.c:647-649`
  sets `derp_region_override` and fires `ML_EVT_DERP_RECONNECT`; the DERP task
  (`ml_derp.c:1272-1309`) calls `ml_derp_disconnect(home)` and then a **fully
  blocking** `ml_derp_connect(home, newRegion)`. While that blocking connect runs
  there is **no connected home conn**, so `derp_pump_home_rx()` early-returns
  (`ml_derp.c:583`) and CANNOT help — the exact hole the prior fix (6832794) leaves.
  The blocking sequence monopolizes the DERP task and its CPU burst starves
  lwIP/httpd, gapping even the DIRECT-path heartbeat > 2 s → nuisance disarm ~1/5.

- **CPU-vs-network verdict (CRITICAL): the ~1-2 s stall is NETWORK-round-trip
  dominated, NOT raw CPU crypto.** Evidence from `machn/sdkconfig`:
  `CONFIG_MBEDTLS_HARDWARE_MPI=y`, `CONFIG_MBEDTLS_MPI_USE_INTERRUPT=y`,
  `CONFIG_MBEDTLS_HARDWARE_AES=y`, `CONFIG_MBEDTLS_HARDWARE_SHA=y` — bignum/RSA/AES/SHA
  are already HW-accelerated. And `mbedtls_ssl_conf_authmode(..., VERIFY_NONE)`
  (`ml_derp.c:1609`) means **no certificate-chain signature verification** — the
  single heaviest CPU item in a TLS handshake is skipped entirely. Residual per-handshake
  CPU (one ECDHE keygen + shared secret, a little SHA/AES) is tens of ms, not seconds.
  The seconds come from **serialized network flights** — `mbedtls_ssl_handshake()`
  blocks in the BIO on `SO_RCVTIMEO` waiting for each server flight (2+ RTT to a
  possibly-distant region, plus the 100 ms/`DERP_CONNECT_TIMEOUT_MS` polling rhythm),
  then DNS + TCP connect + HTTP-upgrade + DERP handshake add more serialized RTTs.
  **=> A non-blocking incremental state machine directly removes the dominant stall.**
  No CPU-offload/yielding is required for correctness; the small residual crypto burst
  is already minimized (HW accel + VERIFY_NONE). **No sdkconfig change is needed**
  (HW MPI interrupt is already on; recommend only *confirming* it stays enabled).

- **Two complementary fixes (do both, staged):**
  1. **Async connect state machine** so the DERP task never blocks > ~50-100 ms per
     iteration and keeps polling every CONNECTED conn's rx every iteration.
  2. **Make-before-break for FORCED region switches** — reuse the *existing gapless*
     MBB warm-standby-then-swap-index machinery (`ml_derp.c:930-1067`) instead of the
     break-before-make `ML_EVT_DERP_RECONNECT` teardown. The autoneg MBB path is
     already gapless; forced switches simply don't use it today. Fix #2 alone removes
     most of the run-26 nuisance stops; fix #1 hardens every reconnect (incl. RST/EOF
     home flap `ml_derp.c:1430-1432` and boot) and lifts the multi-remote scale ceiling
     (task #33).

---

## 1. Current architecture (as read)

### Single-task ownership
`ml_derp_tx_task()` (`ml_derp.c:1073-1457`) is the **sole owner** of every
`ml_derp_conn_t` and its 4 mbedTLS contexts (`microlink_internal.h:627-659`). The file
header (lines 8-15) states the no-mutex invariant: only this task touches the SSL
contexts. Any async design MUST preserve single-task ownership — do NOT move handshake
work to another task (that reintroduces the TLS mutex the design deliberately avoids).

### The per-iteration loop (order matters — safety path first)
1. pause/heartbeat/status (`1087-1138`)
2. **HOME connect / reconnect / periodic-retry / RECONNECT event** (`1140-1310`) —
   this is where the blocking `ml_derp_connect(home, …)` calls live: lines
   1161, 1227, 1239, 1258, 1295.
3. "nothing connected" fast-path (`1317-1335`)
4. Phase 1: drain TX prio+disco queues (`1337-1406`)
5. Phase 2: **poll rx from ALL connected conns, home index first** (`1408-1439`)
6. `derp_manage_aux()` at tail — at most **one** blocking aux connect/call (`1444`)
7. `derp_mbb_tick()` (`1449`) then `vTaskDelay(s_derp_loop_delay_ms)` (default 1  ms).

### The blocking connect sequence (`ml_derp_connect`, 1485-~1960)
Serialized, each stage blocks the whole task:
- DNS `ml_getaddrinfo` (1539)
- TCP `ml_connect`, 10 s SO_SND/RCVTIMEO (1553-1569)
- mbedTLS init of 4 contexts + `ctr_drbg_seed` + config (1579-1620)
- **TLS handshake loop** (1631-1648) — `mbedtls_ssl_handshake()` in a `while`, treating
  WANT_READ/WANT_WRITE/SSL_TIMEOUT as "keep going". For AUX it drops the BIO read
  timeout to 100 ms and calls `derp_pump_home_rx()` between flights (1628-1635). Home
  keeps the long timeout and does **not** pump (there is no *other* home to pump).
- switch socket to O_NONBLOCK + 100 ms timeouts (1740-1752)
- HTTP upgrade write + byte-at-a-time read to `\r\n\r\n` (1659-1738)
- DERP handshake: ServerKey → ClientInfo(NaCl box) → ServerInfo (1754-~1920)
- on any failure: `DERP_CONNECT_FAIL_CLEANUP()` frees all 4 ctxs + socket (1589-1603).

### BIO helpers (reusable — assessed)
- `ml_derp_bio_recv_timeout` (70-98): sets `SO_RCVTIMEO` then `ml_read_sock`; returns
  `MBEDTLS_ERR_SSL_TIMEOUT` on EAGAIN, `WANT_READ` on EINTR. **Reusable as-is** for a
  non-blocking handshake: set the socket timeout to ~0 (or a tiny value) and the BIO
  returns SSL_TIMEOUT immediately when a flight isn't ready — that is exactly the
  "yield back to the loop" signal we want. `ml_derp_bio_send` (103-122) already returns
  `WANT_WRITE` on EAGAIN. **No BIO changes required** — the async engine keys off the
  existing return codes.
- `poll_derp_read` (391-464) already O_NONBLOCK, treats SSL_TIMEOUT/WANT_* as "no data,
  return 0". **This is the rx primitive the loop keeps calling for every CONNECTED conn
  every iteration** — it is the piece that must never be starved.

### The gapless path that already exists (leverage it)
`derp_mbb_tick()` (`930-1067`): for autoneg home switches it (a) adds the target region
to the aux want-set so `derp_manage_aux` opens it as an ordinary aux, (b) PROVES it
(peer echo `rx_pkts` or server PONG), then (c) **swaps `ml->derp_home_slot` index only —
no conn teardown, bond gapless** (`1032-1059`). Forced switches bypass this and use
break-before-make instead. `derp_home_slot` is re-read every iteration (`1103`) so an
index swap is honoured immediately by connect/reconnect/keepalive/rx-order.

---

## 2. Design: per-conn async connect state machine

### 2.1 New per-conn state (add to `ml_derp_conn_t`, `microlink_internal.h:627-659`)
Add a small connect-progress sub-state, written ONLY by the DERP task (keeps the
no-mutex invariant). Keep it out of the hot `connected`/`sockfd` fields so existing
code paths are untouched when a conn is fully CONNECTED.

```c
typedef enum {
  DERP_CS_IDLE = 0,      /* not connecting; slot free or fully CONNECTED */
  DERP_CS_DNS,           /* resolving host (bounded / async-getaddrinfo, see 2.4) */
  DERP_CS_TCP_CONNECT,   /* non-blocking connect() in flight, poll writability   */
  DERP_CS_TLS_HANDSHAKE, /* mbedtls_ssl_handshake() incremental                  */
  DERP_CS_HTTP_UPGRADE,  /* GET /derp sent; reading 101 response incrementally   */
  DERP_CS_DERP_HS,       /* ServerKey -> ClientInfo -> ServerInfo                */
  DERP_CS_CONNECTED,     /* == today's connected=true                            */
  DERP_CS_FAILED         /* transient; cleanup + backoff, back to IDLE           */
} ml_derp_connect_state_t;

/* added fields (all DERP-task-owned): */
ml_derp_connect_state_t cstate;
uint64_t   cstate_deadline_ms;  /* overall connect deadline (DERP_CONNECT_TIMEOUT_MS) */
struct addrinfo *cs_res;        /* DNS result, freed on leave-DNS/fail                */
/* HTTP-upgrade incremental read buffer + len (was a stack local) */
uint8_t    cs_http_buf[512];
int        cs_http_len;
/* DERP-handshake sub-step cursor + a small scratch for partial frame header/key */
uint8_t    cs_derp_step;
/* connected_at_ms / last_recv_ms etc. already exist */
```

Memory: the 4 mbedTLS contexts already live for the conn's lifetime while `tls_inited`
(they persist across iterations by construction). The ONLY lifetime change is that they
now stay initialized across *many* task iterations DURING the handshake rather than
within one blocking call. `cs_res` (addrinfo) and `cs_http_buf` move from stack locals
to the struct so they survive the yields. `ML_DERP_MAX_CONNS==6`, so the added per-conn
footprint (~0.6 KB, dominated by cs_http_buf) × 6 is acceptable; if RAM-tight, allocate
cs_http_buf lazily on entering DERP_CS_HTTP_UPGRADE and free on leave.

### 2.2 The engine: `ml_derp_connect_step(ml, c)`
Replace the monolithic `ml_derp_connect()` with a **bounded single-step advancer** that
does *at most one* non-blocking action per call and returns quickly:

```c
/* Returns: 1 = became CONNECTED this call; 0 = still in progress (call again next
 * iteration); -1 = failed (cleaned up, backoff owned by caller). NEVER blocks more
 * than one BIO timeout (~0-50 ms). Single-task; no locking. */
static int ml_derp_connect_step(microlink_t *ml, ml_derp_conn_t *c);
```

Per-state behaviour (each is O(one syscall / one mbedtls call) then returns):
- **IDLE→DNS:** kick resolve. `ml_getaddrinfo` is blocking in lwIP; keep it but note it
  is normally fast (cached / local resolver). If field data shows DNS itself gapping
  (`[TIMING-DERP] DNS` log at 1545-1550), move to `lwip_getaddrinfo` on a worker or a
  cached-IP fast path (see 2.4). Set `cstate_deadline_ms = now + DERP_CONNECT_TIMEOUT_MS`.
- **DNS→TCP_CONNECT:** create socket **O_NONBLOCK from the start**; `connect()` returns
  EINPROGRESS; store fd; return 0.
- **TCP_CONNECT:** poll writability (non-blocking `select`/`poll` with 0 timeout, or a
  `getsockopt(SO_ERROR)`); if connected → init the 4 mbedTLS ctxs + set BIO (the current
  1579-1620 block, verbatim) → TLS_HANDSHAKE; if still pending → return 0; on error →
  FAILED.
- **TLS_HANDSHAKE:** call `mbedtls_ssl_handshake()` **once**. WANT_READ/WANT_WRITE/
  SSL_TIMEOUT → return 0 (resume next iteration — mbedTLS resumes cleanly on a stream,
  exactly as the current aux loop relies on at 1633-1641). 0 → HTTP_UPGRADE. other → FAILED.
  Set the BIO read timeout SHORT here (reuse the aux 100 ms path at 1629, but for HOME
  too now — safe because we no longer *loop* inside the call).
- **HTTP_UPGRADE:** on first entry send the GET (buffer the write; handle WANT_WRITE).
  Then read into `cs_http_buf` incrementally (the byte-loop at 1687-1728 becomes "read
  what's available this call, advance cs_http_len, check for `\r\n\r\n`"); WANT/TIMEOUT
  → return 0. On `101` → set O_NONBLOCK 100 ms socket opts (1740-1752) → DERP_HS.
- **DERP_HS:** drive ServerKey→ClientInfo→ServerInfo with `cs_derp_step`. The existing
  `derp_recv_frame_header`/`derp_tls_read_all`/`derp_write_frame` helpers each already
  return on WANT/TIMEOUT; wrap them so a not-yet-complete read just returns 0 and
  re-enters at the same `cs_derp_step` next iteration (they must become non-accumulating:
  either read-all-now-or-return-0; simplest is to bound them to "make progress or yield"
  and keep a small partial cursor). On ServerInfo done → set `connected=true`,
  `cstate=CONNECTED`, `connected_at_ms=now` → return 1.
- **any state past deadline** (`now > cstate_deadline_ms`) → FAILED.
- **FAILED:** run `DERP_CONNECT_FAIL_CLEANUP()` equivalent (free 4 ctxs + socket + free
  cs_res), set `cstate=IDLE`; caller applies existing per-slot backoff.

### 2.3 Task-loop integration (drives progress without ever blocking the rx poll)
Restructure the loop so **rx poll of all CONNECTED conns runs EVERY iteration, and every
in-progress connect advances by exactly one bounded step per iteration**:

1. Compute `home = &ml->derp[ml->derp_home_slot]` (unchanged, 1103).
2. **NEW ordering:** run **Phase 2 rx poll (1408-1439) FIRST-ish** for every conn whose
   `cstate==CONNECTED`, so a reconnecting home (or aux) never delays a *connected* conn's
   heartbeat. (Home index first is preserved.)
3. Drive at most one `ml_derp_connect_step()` per in-progress conn per iteration (home
   prioritized). Because each step is bounded, N simultaneously-connecting conns cost N
   bounded steps, not N × (1-2 s) — this is the multi-remote scale-ceiling fix.
4. Replace the blocking home connect sites (1161, 1227, 1239, 1295) with "ensure this
   conn's cstate is progressing toward CONNECTED for the intended region" — set the
   region + kick cstate to DNS if IDLE; the per-iteration step advances it. The 3-attempt
   `for` loops with `vTaskDelay(2s/4s/8s)` (1153-1167, 1290-1308) collapse into the
   existing per-slot backoff timers (no in-loop `vTaskDelay` that stalls rx).
5. `derp_manage_aux` (639-773) loses its "one BLOCKING connect per call, then return"
   hard cap (706-772) — replace with "kick/advance connect state for wanted regions"
   using the same stepper. The `return` after one connect (772) is no longer needed for
   starvation safety (steps are bounded), though keeping a small per-iteration cap on
   *new* handshakes is fine to bound crypto bursts.

### 2.4 CPU-burst handling (per verdict: minor, but bound it)
Because HW MPI/AES/SHA are on and VERIFY_NONE skips cert verify, the residual crypto is
small. Guard rails, cheapest first:
- **Bound concurrent handshakes:** advance at most K (e.g. 1-2) conns' *crypto-heavy*
  steps (TLS_HANDSHAKE) per iteration so K simultaneous ECDHE keygens don't stack.
- **Keep MPI interrupt mode on** (already `CONFIG_MBEDTLS_MPI_USE_INTERRUPT=y`): the
  bignum unit runs async to the CPU, so the task yields during modexp. Recommend
  *confirming* this stays set; do NOT need to add `MBEDTLS_ECP_RESTARTABLE` (currently
  off) unless post-change bench still shows a single-call crypto spike > ~50 ms — it is
  the escape hatch (restartable ECC lets `mbedtls_ssl_handshake` return
  `MBEDTLS_ERR_SSL_CRYPTO_IN_PROGRESS` mid-point-mul), but it adds complexity and is
  likely unnecessary given the network-bound verdict. **Call it out as a measured
  contingency, not a day-1 change.**
- The DERP task is **not** registered with the Task WDT (no `esp_task_wdt_add` in the
  component); the 5 s WDT (`CONFIG_ESP_TASK_WDT_TIMEOUT_S=5`) watches the idle tasks.
  Non-blocking steps + the existing `vTaskDelay` at loop tail keep idle fed — WDT margin
  IMPROVES. No WDT feeding code needed.

---

## 3. Make-before-break for FORCED region switch (the direct run-26 fix)

Today `ml_config_httpd.c:646-649` sets `derp_region_override` + `ML_EVT_DERP_RECONNECT`,
and the task tears down home then blocking-reconnects (1272-1309): break-before-make.

**Change:** route a forced/locked region change through the SAME warm-standby-then-swap
mechanism MBB already uses:
- Feed the pinned region as an MBB target (or an aux want-set entry) so
  `derp_manage_aux` opens it as an aux while the OLD home stays CONNECTED and keeps
  carrying the relay + rx.
- When the new conn reaches CONNECTED (and, if desired, passes the existing lightweight
  proof at 1000-1028), **swap `derp_home_slot` index** (1043) — gapless, no teardown.
- Only AFTER the swap does the old home become an ordinary aux and get reaped by the
  existing unwanted-aux logic (678-704).
- Note the existing MBB-abort-on-lock guard (`ml_derp.c:928-929`): today a lock ABORTS
  an in-flight MBB. The forced-switch integration must instead DRIVE the switch to the
  locked region via the same gapless path, not abort. Keep `derp_region_override` as the
  authoritative eff-home once committed (it already wins in `ml_effective_home_region`,
  microlink.c:719-736). The `microlink_request_announce()` re-advertise (658) stays.

This fix does not require the full async engine and removes the run-26 gap for the
*forced-switch* case on its own — hence it is staged first. The async engine then
generalizes gaplessness to the cases with no warm standby (RST/EOF home flap at
1430-1432, boot, and true home-server-down where make-before-break is impossible).

---

## 4. Risks

1. **Reentrancy / single-task invariant.** The whole point is to keep ALL connect work
   on the DERP task. As long as `ml_derp_connect_step` runs only from `ml_derp_tx_task`,
   the no-mutex invariant (file header 8-15) holds. RISK if anyone is tempted to move
   handshakes to a worker task — DO NOT; that needs the TLS mutex the design avoids.
   `derp_pump_home_rx` recursion is eliminated (rx poll is now top-level every iteration),
   removing the "re-entrant poll on a different conn" subtlety (573-590).
2. **mbedTLS ctx lifetime across iterations.** The 4 ctxs now live across many iterations
   mid-handshake. `tls_inited` already tracks liveness; ensure every FAILED/timeout/
   home-swap path frees them exactly once (mirror `DERP_CONNECT_FAIL_CLEANUP` 1589-1603
   and `ml_derp_disconnect` 1980-1989). The 2026-05-25 leak class (re-init over live
   ctxs) is the top regression risk — keep the idempotency guard (1492-1494) and assert
   `cstate==IDLE` before re-init.
3. **Partial-frame resumption in DERP_HS.** `derp_recv_frame_header`/`derp_tls_read_all`
   currently loop internally until complete or timeout. Making them yield-and-resume needs
   a per-conn cursor for the in-progress frame; a bounded partial read that has consumed
   N of M bytes must not be re-requested. Simplest safe form: keep these "read fully or
   return 0 (nothing consumed yet)" at frame granularity, accepting that a single DERP
   handshake frame read may block up to one 100 ms BIO timeout — still within the
   50-100 ms/iteration budget and vastly better than 1-2 s.
4. **Interaction with existing pump-home-rx (6832794).** Once rx poll is top-level every
   iteration, `derp_pump_home_rx` and the aux-handshake 100 ms/pump special-case
   (1622-1653) become **redundant** and should be removed to avoid two mechanisms. Do
   this only in the stage that lands the async engine (not in the make-before-break stage).
5. **Region-switch / rehome ownership.** `derp_home_region` / `priority_peer_region` have
   documented single-writer rules (wg_mgr negotiator vs DERP task, see 1055-1057,
   1220-1233). Make-before-break for forced switch must respect that the DERP task owns
   the index swap and `derp_region_override` is the lock; don't introduce a second writer.
6. **Home-conn reconnect with NO warm standby** (RST/EOF flap, boot, home-server-down).
   Make-before-break is impossible (nothing to hand rx to). Here the async engine's value
   is that the *task keeps iterating* — it stops monopolizing CPU/lwIP so the
   region-INDEPENDENT DIRECT hairpin heartbeat keeps flowing (the run-26 direct-path gap),
   and any *other* CONNECTED aux keeps being polled. The bond still can't relay via home
   until home is back, but the direct path — which is what the fail-safe watches — no
   longer gaps.
7. **DNS still blocking.** `ml_getaddrinfo` (1539) remains synchronous. If bench shows DNS
   as a residual gap source, add a cached-last-good-IP fast path for the home region (skip
   DNS on reconnect to the same host) — cheap, keeps us off a worker task.

---

## 5. Staged, independently bench-testable implementation order

Each stage is a separate commit, builds, and is validated against the run-26 repro
before proceeding. **Smallest safe increments first.**

**Stage 0 — Instrumentation only (no behaviour change).**
Add a per-connect wall-clock breakdown already partly present (`[TIMING-DERP]` DNS/TCP/
TLS logs 1545-1656) plus a max-single-iteration-duration gauge for the DERP task and a
"ms since last home rx poll" gauge, exposed like the existing reconnect diag
(`ml_derp_get_reconnect_diag`, 606). This quantifies the CPU-vs-network split on the
real machn and gives a regression baseline. Validate: forced switch, read timings.

**Stage 1 — Make-before-break for forced region switch (biggest bang, smallest change).**
Change `ml_config_httpd.c:646-649` + task RECONNECT handler (1272-1309) so a
`derp_region_override` change drives the region in as an aux/MBB-target and swaps the
home index when CONNECTED, instead of tearing down home first. Reuse `derp_mbb_tick`
swap (1032-1059) / want-set (652-667). Do NOT touch the async engine yet. Validate:
run-26 repro — POST `{derp_region:N}` (auth `admin:microlink`) to machn
`192.168.107.192`; green must hold through the switch; confirm no home teardown before
the new conn is up (log). This alone should kill the ~1/5 nuisance disarm for forced
switches.

**Stage 2 — Async engine behind a flag, AUX conns only.**
Introduce `ml_derp_connect_step` + `cstate` and route only AUX opens
(`derp_manage_aux`, 720-772) through it; HOME still uses the old blocking
`ml_derp_connect` (guarded by a compile/runtime flag). Validate: with 3-5 safety remotes
(multi-region), confirm no single DERP-task iteration exceeds ~50-100 ms (Stage-0 gauge)
and home heartbeat is uninterrupted while multiple aux conns open — the task #33 scale
ceiling test.

**Stage 3 — Async engine for HOME conn; retire pump-home-rx.**
Route HOME (re)connects (1161/1227/1239/1295, and the RST/EOF flap path 1430-1432)
through `ml_derp_connect_step`. Remove `derp_pump_home_rx` (580-595) and the aux-only
100 ms/pump handshake special-case (1622-1653) — rx poll is now top-level every
iteration. Validate: force a HOME RST/EOF flap AND a home-server-down (unreachable
region) — direct-path heartbeat must not gap > (fail-safe threshold); WDT margin
unchanged/improved.

**Stage 4 — Cleanup + optional contingency.**
Delete the old blocking `ml_derp_connect` body once all callers use the stepper; collapse
the 3-attempt `vTaskDelay` retry loops into per-slot backoff. ONLY IF Stage-2/3 bench
still shows a single-call crypto spike > ~50 ms: evaluate enabling
`CONFIG_MBEDTLS_ECP_RESTARTABLE` (currently off) as a measured follow-up. Expected: not
needed (network-bound verdict).

### Validation harness (run-26 repro, all stages)
- Trigger: `POST /admin/api/settings` body `{"derp_region":N}` (basic auth
  `admin:microlink`) to machn `192.168.107.192`; alternate N between two reachable
  regions to force home switches.
- Pass criterion: **green (armed) holds through the switch** — no fail-safe nuisance
  disarm; direct-path heartbeat gap stays under the pstop timeout (~2 s).
- Metrics: Stage-0 max-iteration gauge, `ml_derp_get_reconnect_diag` worst-reconnect ms
  (target well under 2000), home-rx-gap gauge, and the `[TIMING-DERP]` split.
- READ-ONLY / soak note: the target is mid-soak; do NOT build/flash from this plan.
  Bench on a spare unit first; land to the soak unit only between soak windows.

---

## 6. Files touched (implementation)
- `components/microlink/src/ml_derp.c` — connect state machine, task-loop reorder,
  manage_aux, remove pump-home-rx, forced-switch make-before-break, disconnect/cleanup.
- `components/microlink/include/microlink_internal.h` — `ml_derp_conn_t` cstate fields +
  `ml_derp_connect_state_t` enum.
- `components/microlink/src/ml_config_httpd.c` — forced region-switch → make-before-break
  trigger (Stage 1).
- (No change) `machn/sdkconfig` — HW crypto already enabled; only *confirm* MPI interrupt
  stays on.
- **Untouched:** `pstop_c/**` (safety-certified — explicitly out of scope).
