# X25519 crypto speedup — measured report

**Branch:** `fast-crypto` (tip `0ca8b18`). **Silicon:** ESP32-S3 @ 240 MHz,
measured on live unit machn-01d77a1c via `GET /api/bench_x25519`.
**Method:** each candidate run in a tight loop, `esp_timer` around it,
50 iterations, LAN path (no tailnet jitter), outputs byte-compared. All
candidates produce identical results (`outputs_match: 1`).

## The delta (per X25519 scalar-multiply)

| implementation | opt | µs/op | vs baseline |
|---|---|---:|---:|
| **Hamburg/STROBE `refc/x25519.c` — as-deployed baseline** | `-Og` | **28,300** | **1.0×** |
| curve25519-donna (Google, 2008) | `-O3` | 38,000–150,000 | **0.1–0.7× (regression)** |
| Monocypher `crypto_x25519` | `-O3` | 18,000 | 1.6× |
| **Hamburg/STROBE — same code, higher opt** | `-O3` | **11,300** | **2.5×** |

**Winner: the code we already ship, compiled at `-O3`.** No new crypto in
the safety path, 2.5× faster. This is now live in the deployed firmware —
the "baseline" column of a fresh bench reads **11.5 µs** because
`x25519.c` itself is now `-O3`; the 28.3 µs figure is captured from the
prior `-Og` builds.

## Why not 10×

The 10× target came from web reports of "optimized X25519 ~7 ms vs
mbedtls ~100 ms" on ESP32-class parts. That comparison uses **mbedtls's
generic ECP ladder** as the baseline — which we do **not** use. Our
baseline is Mike Hamburg's STROBE implementation, which is already good;
at `-O3` it hits **11.3 ms ≈ 2.7 M cycles**, already inside the
optimized-C range the literature reports (~3–4 M cycles). The remaining
headroom to ~8 ms needs **hand-written Xtensa field arithmetic**
(the papers that hit ~7 ms use assembly), which:

- is only ~1.4× more on top of what we now have (not 4×),
- touches hand-vetted asm in the safety core,
- has no reference implementation we can drop in and byte-verify.

So it is scoped as **future work**, not landed tonight. Portable-C
library swaps are exhausted: donna is a regression, Monocypher is real
but worse than Hamburg-at-`-O3`.

## What shipped on the branch

- **`-O3` on the vetted primitives** (`x25519.c`, `nacl_box.c`, and the
  vendored per-packet `chacha20poly1305.c` / `chacha20.c` /
  `poly1305-donna.c` / `blake2s.c`). Project default is `-Og`. This is
  the entire real win; same code, verified-identical output.
- **donna reverted out of every hot path** — the WG-handshake macro,
  disco NaCl DH, and all four Noise-protocol sites are back to `x25519`.
  curve25519-donna, Monocypher, and `x25519_o3.c` remain **only** as
  inputs to the benchmark endpoint (never called on the safety path)
  and as a regression guard for the future asm effort.
- **`GET /api/bench_x25519?iters=N`** — the on-device shootout, kept so
  any future candidate is measured on real silicon, not extrapolated.

## Where the win lands

X25519 is **handshake-only**, not per-packet: it runs on first contact
with a peer and on every WireGuard handshake/rekey (~2 min, or on
reconnect) — a WG handshake is 4 scalar-mults. Computed from the
measured per-op delta:

- WG handshake X25519 cost: ~113 ms → ~45 ms (**−68 ms**)
- disco cold-contact DH: ~28 ms → ~11 ms (**−17 ms**)

Steady-state disco RTT is **unaffected** — it uses the cached shared key
(no X25519), and its latency was fixed separately by the queue-budget
change (`80e96fd`). The AEAD `-O3` (ChaCha20-Poly1305) is a per-packet
win in the same 2–3× class but was not A/B-measured here (no `-Og`
reference built); it ships applied.

## Recommendation

> **STATUS: MERGED + DEPLOYED.** The `-O3`-on-primitives change shipped in
> PR #74 (`7867d4b`, on `main`/`dev`): `set_source_files_properties(... -O3)`
> on the vetted AEAD/X25519 primitives in `components/microlink/CMakeLists.txt`
> and `components/microlink/components/wireguard_lwip/CMakeLists.txt`, with
> KATs guarding byte-identical output. The text below is the original
> pre-merge recommendation, retained for rationale.

Merge the **`-O3`-on-primitives** change (the 2.5×, zero-risk part) after
CI + a HIL soak — it is a pure compile-flag change to already-certified
code with byte-identical output. Drop donna and Monocypher from the tree
before merge (or keep behind the bench only). File the Xtensa-asm X25519
as a separate future ticket if handshake latency ever becomes a field
concern (it currently isn't — recovery is dominated by the ~28 s
boot-to-tailnet time, not crypto).

## Validation

Deployed to both live units (machn-01d77a1c, pstop-01d7eed0); both
re-bonded cleanly, both machines `BONDED`, handshakes succeed (donna's
removal verified by grep: zero `curve25519_donna` calls remain on any
hot path).
