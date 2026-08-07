# MISRA C compliance review — pstop remote firmware (2026-07-21)

**Standard:** MISRA C:2012 rule set (checked with cppcheck 2.7 `misra`
addon; MISRA C:2023 is the current edition — same rule numbering for
everything cited here, with mandatory/required/advisory categories).
The code is C-only; MISRA C++:2023 does not apply.

**Scope:** `firmware/main/main.c` + `firmware/components/dcs_support/`
(the firmware we own). **Excluded:** `components/pstop/upstream`
(certification track — reviewed/complied separately by its own process;
deliberately untouched), ESP-IDF and managed components (third-party
platform), and `components/microlink` (comms library — recommended as a
follow-up sweep; the changes made there during this campaign were kept
minimal and reviewed by hand).

**Reproduce:** `sudo apt-get install -y cppcheck` then `./tools/misra_check.sh`
(free cppcheck misra addon; scans firmware/main + dcs_support, excludes
pstop_c and platform components). This is the engineering pre-check;
certification evidence needs a licensed checker.

**Method:** automated scan via the project `compile_commands.json` +
manual review; `dcs_net_liveness.c` crashes the addon (known cppcheck 2.7
rule-7.4 checker bug) and received a manual pass against the same rule
set. Rule texts are licensed and not redistributed here; findings cite
rule numbers.

## Results

| Scope | Before | After | Notes |
|---|---|---|---|
| firmware/components/dcs_support | 513 | 273 | residual = deviation register below |
| firmware/main/main.c (standalone scan) | 30 | 16 | residual = deviations + 3 false positives |

All fixes are behavior-preserving; the firmware builds clean and was
re-verified on the bench (bond, verdicts, policy veto) after the sweep.

### Fixed categories (~250 findings)
- 17.7 unused returns: safety-chain calls (NVS commits, boot-count writes,
  watchdog registration, task creation, rollback invocation) now CHECK and
  log on failure; deliberate ignores got explicit `(void)` casts.
- 21.7 `atoi` → `strtol` (4 sites).
- 15.6 missing braces (~40), 14.4 essentially-boolean conditions (~45),
  12.1 clarifying parentheses (~35), 15.7 final `else` (~6), 16.4 switch
  default, 13.3 side-effect separation (3), 2.2 dead code (1),
  8.1/8.4 declarations, 10.x explicit casts / unsigned literals where a
  single obvious cast resolves the essential-type mismatch (~40).
- One genuinely undefined-ish construct: `ring_state_t last_logged = -1`
  (out-of-range enum object) — replaced with an `int`.

### Real defects found by the review (beyond style)
1. **FIXED** — `dcs_admin_pages.c` reset-history JSON builder: unclamped
   `snprintf` offset could underflow `sizeof - rp` (size_t) and write out
   of bounds with corrupt NVS values; buffer also undersized for
   worst-case entries. Clamped per append + resized.
2. **FIXED** (since 2026-07-21) — `dcs_eth.c` now unwinds a full
   `goto fail_netif → fail_driver → fail_spi → fail_sem` cleanup ladder,
   freeing exactly what the call created in reverse order (netif, driver
   handle, phy/mac, SPI bus if owned, semaphore); no handle/semaphore
   leak on partial bring-up failure.
3. **FIXED** (since 2026-07-21) — `dcs_telemetry.c` raised `MAX_TASKS`
   40 → 64 **and** guards the overflow: when `uxTaskGetSystemState`
   returns 0 the sampler skips the sample (keeps `prev_*`) with a warning
   instead of computing a bogus 0 % load.
4. **FIXED** (2026-08-07) — `dcs_wifi_status()` cross-task shared reads
   are now atomic: `s_disc_reason`, `s_connected`, and `s_idx` are
   `atomic_int` (load/store/fetch_add); `s_list` is malloc'd once and
   never freed, so the pointer read is stable. No unlocked data race.
5. **FIXED** (since 2026-07-21) — `api_iface_usb` now reports the ACTUAL
   resulting state on failure (`iface_toggle_reply(req, r, (r==ESP_OK) ? on
   : dcs_usb_is_enabled())`), matching the eth/wifi twins.

## Deviation register (residual findings, with rationale)

| Rule | Cat. | Count | Rationale |
|---|---|---|---|
| 15.5 multiple returns | Advisory | ~108 | Disapplied: early-return-on-error is the codebase idiom; restructuring is higher risk than the rule's benefit. Recorded per MISRA Compliance for advisory disapplication. |
| 10.1/10.4/8.1 in macro expansions | Required | ~100 | Findings originate inside ESP_LOG*, FreeRTOS and IDF config macros; not fixable at call sites without forking platform headers. Platform-inherited deviation. |
| 11.5/11.6 void* conversions | Advisory | ~6 | FreeRTOS task args / IDF driver API idiom. |
| 17.1 stdarg | Required | 6 | panic_log's printf-style capture; varargs are intrinsic to the logging design. |
| 21.6 stdio (snprintf) | Required | few | Bounded formatting only; no streams/files. Common industry deviation. |
| 21.3 malloc/free | Required | few | Heap use confined to non-safety paths (HTTP response buffers, PSRAM task stacks); safety loop allocates nothing after init. |
| 22.8 errno before strtol | Required | 4 | Introduced by the 21.7 fix; errno is never consulted — parse tolerates malformed input by design (same semantics as the atoi it replaced). |
| 12.3/18.4/8.9/19.2/20.5/15.4/8.7/12.2 | Advisory/Req. | ~30 | Style/structure items where change adds risk without value; reviewed individually. |
| 18.8 (main.c ×3) | Required | 3 | False positives: `uint8_t buf[PSTOP_MESSAGE_SIZE]` — macro constant unresolved in the standalone scan; arrays are fixed-size. |

## Follow-ups recommended
- ~~Sweep `components/microlink`~~ **DONE 2026-08-07** — see below.
- ~~Fix the flagged defects~~ **DONE** — all five real defects above are
  now FIXED (dcs_eth cleanup ladder, telemetry cap+guard, wifi_status
  atomics, usb toggle reply); the earlier reset-history snprintf was
  already fixed.
- For certification evidence, re-run with a licensed MISRA checker
  (rule texts + official compliance report format); this review is the
  engineering pass that precedes it.

## `components/microlink` sweep (2026-08-07)

Swept all 17 `components/microlink/src/*.c` with the same free-cppcheck
method (`--addon=misra` + `--enable=warning,style`).

**Style/rule profile matches the existing deviation register** — no new
deviation categories. Top rules are the same platform/idiom-inherited
findings already disapplied for dcs: 2.5 unused-macros (~1087, header
constants), 10.4/10.1 essential-type (macro expansions), 15.5
multiple-returns (~368, codebase idiom), 17.7 unused-return (void-cast
candidates), 21.3 malloc (non-safety HTTP/PSRAM paths), 11.5 void\*
(FreeRTOS/IDF API). See the §"Deviation register" rationale — all apply.

**Genuine-defect classes (leak / null / uninit / buffer / race): none in
the network logic.** The only two warning-level findings are in the
vetted, KAT-verified crypto primitives, and both are confirmed
**false positives**:
- `nacl_box.c:454` `final_block[len]=1` (`arrayIndexOutOfBoundsCond`) —
  `len` is the post-`len -= (len & ~15UL)` remainder, so `len ∈ [0,15]`;
  writes indices 0–15, all in-bounds. Standard poly1305 final block;
  cppcheck can't prove the mask bound.
- `x25519.c:220` `mul(z2,x2,a24,nb=1)` (`argumentSize`) — the `nb`
  argument bounds `mul` to read only the single limb `a24` holds;
  curve25519-donna pattern, cppcheck's size heuristic ignores `nb`.

Byte-identical KAT output already proves both primitives correct.

**httpd URI-handler budget (was flagged at ceiling 16 in
`MULTI_REMOTE_VALIDATION_2026-07-22.md`) is resolved:** the app httpd is
now `16 + max_user_uri_handlers`, with `cfg.max_user_uri_handlers = 30`
(`dcs_support.c`) → 46 slots for microlink's ~20 + dcs's 21, with
headroom.
