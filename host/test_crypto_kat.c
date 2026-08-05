// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Golden-vector known-answer tests (KATs) for the safety-path crypto
 * primitives that were switched to -O3 in "crypto: -O3 on the vetted
 * primitives (2.5x X25519, byte-identical)".
 *
 * Purpose: lock in that these primitives produce bit-exact, RFC-correct
 * output so a future compiler / flag / toolchain change can never silently
 * miscompile the WireGuard / disco / Noise handshake crypto. The primitive
 * .c files are compiled INTO this test target at -O3 (see host/Makefile),
 * so the test itself would catch an -O3 miscompile of the shipped code.
 *
 * Pure host-runnable: no ESP-IDF, no network, no hardware. Style follows
 * host/test_clock_guard.c.
 *
 * Vectors and their sources (all cross-checked against an independent
 * implementation — Python `cryptography` / `hashlib` — at authoring time):
 *   - X25519            RFC 7748 section 5.2 (two single scalar-mults + the
 *                       1- and 1000-iteration iterated vector)
 *   - ChaCha20 block    RFC 8439 section 2.4.2 (keystream)
 *   - Poly1305          RFC 8439 section 2.5.2 (one-time MAC)
 *   - ChaCha20-Poly1305 RFC 8439 section 2.8.2 (AEAD ciphertext + tag),
 *                       driven through the shipped chacha20.c + poly1305.c
 *                       primitives; plus a round-trip + tamper-reject and a
 *                       byte-locked golden for the production AEAD wrapper.
 *   - BLAKE2s-256       official KAT ("abc", empty, and RFC 7693 keyed form)
 *
 * Primitives covered: components/microlink/src/x25519.c and the
 * wireguard_lwip refc chacha20 / poly1305 / chacha20poly1305 / blake2s.
 * NaCl crypto_box (nacl_box.c) is intentionally NOT covered here: it
 * unconditionally #includes <esp_heap_caps.h> (ESP-IDF) and will not build
 * standalone on the host without dragging in the SDK. Its curve25519 core is
 * the same primitive already locked by the X25519 KAT above.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../components/microlink/src/x25519.h"
#include "../components/microlink/components/wireguard_lwip/src/crypto/refc/blake2s.h"
#include "../components/microlink/components/wireguard_lwip/src/crypto/refc/chacha20.h"
#include "../components/microlink/components/wireguard_lwip/src/crypto/refc/chacha20poly1305.h"
#include "../components/microlink/components/wireguard_lwip/src/crypto/refc/poly1305-donna.h"

static int g_checks = 0;
static int g_fails = 0;

static void check_bytes(const char *what, const uint8_t *got, const uint8_t *exp, size_t n)
{
  g_checks++;
  if (memcmp(got, exp, n) != 0) {
    g_fails++;
    printf("  FAIL: %s\n    got: ", what);
    for (size_t i = 0; i < n; i++) printf("%02x", got[i]);
    printf("\n    exp: ");
    for (size_t i = 0; i < n; i++) printf("%02x", exp[i]);
    printf("\n");
  }
}

static void check_true(const char *what, int cond)
{
  g_checks++;
  if (!cond) {
    g_fails++;
    printf("  FAIL: %s\n", what);
  }
}

/* ---------------------------------------------------------------------------
 * X25519 — RFC 7748 section 5.2
 *
 * The shipped x25519() clamps the scalar when clamp=1 (matching RFC
 * decodeScalar25519) but, per its header WARNING, does NOT mask the high bit
 * of the u-coordinate. RFC decodeUCoordinate DOES mask it, so we clear
 * base[31] & 0x80 before each call to follow the RFC exactly.
 * ------------------------------------------------------------------------- */
static void test_x25519(void)
{
  /* Vector 1 */
  static const uint8_t s1[32] = {
      0xa5, 0x46, 0xe3, 0x6b, 0xf0, 0x52, 0x7c, 0x9d, 0x3b, 0x16, 0x15, 0x4b, 0x82, 0x46, 0x5e, 0xdd,
      0x62, 0x14, 0x4c, 0x0a, 0xc1, 0xfc, 0x5a, 0x18, 0x50, 0x6a, 0x22, 0x44, 0xba, 0x44, 0x9a, 0xc4};
  static const uint8_t u1[32] = {
      0xe6, 0xdb, 0x68, 0x67, 0x58, 0x30, 0x30, 0xdb, 0x35, 0x94, 0xc1, 0xa4, 0x24, 0xb1, 0x5f, 0x7c,
      0x72, 0x66, 0x24, 0xec, 0x26, 0xb3, 0x35, 0x3b, 0x10, 0xa9, 0x03, 0xa6, 0xd0, 0xab, 0x1c, 0x4c};
  static const uint8_t o1[32] = {
      0xc3, 0xda, 0x55, 0x37, 0x9d, 0xe9, 0xc6, 0x90, 0x8e, 0x94, 0xea, 0x4d, 0xf2, 0x8d, 0x08, 0x4f,
      0x32, 0xec, 0xcf, 0x03, 0x49, 0x1c, 0x71, 0xf7, 0x54, 0xb4, 0x07, 0x55, 0x77, 0xa2, 0x85, 0x52};
  /* Vector 2 (u has its high bit set -> must be masked to match RFC) */
  static const uint8_t s2[32] = {
      0x4b, 0x66, 0xe9, 0xd4, 0xd1, 0xb4, 0x67, 0x3c, 0x5a, 0xd2, 0x26, 0x91, 0x95, 0x7d, 0x6a, 0xf5,
      0xc1, 0x1b, 0x64, 0x21, 0xe0, 0xea, 0x01, 0xd4, 0x2c, 0xa4, 0x16, 0x9e, 0x79, 0x18, 0xba, 0x0d};
  static const uint8_t u2[32] = {
      0xe5, 0x21, 0x0f, 0x12, 0x78, 0x68, 0x11, 0xd3, 0xf4, 0xb7, 0x95, 0x9d, 0x05, 0x38, 0xae, 0x2c,
      0x31, 0xdb, 0xe7, 0x10, 0x6f, 0xc0, 0x3c, 0x3e, 0xfc, 0x4c, 0xd5, 0x49, 0xc7, 0x15, 0xa4, 0x93};
  static const uint8_t o2[32] = {
      0x95, 0xcb, 0xde, 0x94, 0x76, 0xe8, 0x90, 0x7d, 0x7a, 0xad, 0xe4, 0x5c, 0xb4, 0xb8, 0x73, 0xf8,
      0x8b, 0x59, 0x5a, 0x68, 0x79, 0x9f, 0xa1, 0x52, 0xe6, 0xf8, 0xf7, 0x64, 0x7a, 0xac, 0x79, 0x57};

  uint8_t u[32], out[32];

  memcpy(u, u1, 32);
  u[31] &= 0x7f;
  check_true("x25519 v1 returns success", x25519(out, s1, u, 1) == 0);
  check_bytes("x25519 RFC7748 5.2 vector 1", out, o1, 32);

  memcpy(u, u2, 32);
  u[31] &= 0x7f;
  check_true("x25519 v2 returns success", x25519(out, s2, u, 1) == 0);
  check_bytes("x25519 RFC7748 5.2 vector 2", out, o2, 32);

  /* Iterated vector: k = u = base(9); repeat { r = X25519(k,u); u = k; k = r } */
  static const uint8_t iter1[32] = {
      0x42, 0x2c, 0x8e, 0x7a, 0x62, 0x27, 0xd7, 0xbc, 0xa1, 0x35, 0x0b, 0x3e, 0x2b, 0xb7, 0x27, 0x9f,
      0x78, 0x97, 0xb8, 0x7b, 0xb6, 0x85, 0x4b, 0x78, 0x3c, 0x60, 0xe8, 0x03, 0x11, 0xae, 0x30, 0x79};
  static const uint8_t iter1000[32] = {
      0x68, 0x4c, 0xf5, 0x9b, 0xa8, 0x33, 0x09, 0x55, 0x28, 0x00, 0xef, 0x56, 0x6f, 0x2f, 0x4d, 0x3c,
      0x1c, 0x38, 0x87, 0xc4, 0x93, 0x60, 0xe3, 0x87, 0x5f, 0x2e, 0xb9, 0x4d, 0x99, 0x53, 0x2c, 0x51};

  uint8_t k[32] = {9}, uu[32] = {9}, ubase[32], res[32];
  for (int i = 1; i <= 1000; i++) {
    memcpy(ubase, uu, 32);
    ubase[31] &= 0x7f;
    (void)x25519(res, k, ubase, 1);
    memcpy(uu, k, 32);
    memcpy(k, res, 32);
    if (i == 1) check_bytes("x25519 RFC7748 5.2 iterated (1 iteration)", k, iter1, 32);
  }
  check_bytes("x25519 RFC7748 5.2 iterated (1000 iterations)", k, iter1000, 32);
}

/* ---------------------------------------------------------------------------
 * ChaCha20 block function — RFC 8439 section 2.4.2
 *
 * The WireGuard chacha20_init() forces the first nonce word (state[13]) to 0,
 * so the RFC's 96-bit nonce is set on the state directly (state[] is public),
 * then a 64-byte keystream is produced by encrypting zeros.
 * ------------------------------------------------------------------------- */
static void test_chacha20_block(void)
{
  static const uint8_t key[32] = {
      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
  static const uint8_t expect[64] = {
      0x10, 0xf1, 0xe7, 0xe4, 0xd1, 0x3b, 0x59, 0x15, 0x50, 0x0f, 0xdd, 0x1f, 0xa3, 0x20, 0x71, 0xc4,
      0xc7, 0xd1, 0xf4, 0xc7, 0x33, 0xc0, 0x68, 0x03, 0x04, 0x22, 0xaa, 0x9a, 0xc3, 0xd4, 0x6c, 0x4e,
      0xd2, 0x82, 0x64, 0x46, 0x07, 0x9f, 0xaa, 0x09, 0x14, 0xc2, 0xd7, 0x05, 0xd9, 0x8b, 0x02, 0xa2,
      0xb5, 0x12, 0x9c, 0xd1, 0xde, 0x16, 0x4e, 0xb9, 0xcb, 0xd0, 0x83, 0xe8, 0xa2, 0x50, 0x3c, 0x4e};

  struct chacha20_ctx ctx;
  chacha20_init(&ctx, key, 0);
  ctx.state[12] = 1u;          /* block counter = 1 */
  ctx.state[13] = 0x09000000u; /* nonce = 00 00 00 09 | 00 00 00 4a | 00 00 00 00 */
  ctx.state[14] = 0x4a000000u;
  ctx.state[15] = 0x00000000u;

  uint8_t zero[64] = {0}, ks[64];
  chacha20(&ctx, ks, zero, 64);
  check_bytes("chacha20 block RFC8439 2.4.2 keystream", ks, expect, 64);
}

/* ---------------------------------------------------------------------------
 * Poly1305 one-time MAC — RFC 8439 section 2.5.2
 * ------------------------------------------------------------------------- */
static void test_poly1305(void)
{
  static const uint8_t key[32] = {
      0x85, 0xd6, 0xbe, 0x78, 0x57, 0x55, 0x6d, 0x33, 0x7f, 0x44, 0x52, 0xfe, 0x42, 0xd5, 0x06, 0xa8,
      0x01, 0x03, 0x80, 0x8a, 0xfb, 0x0d, 0xb2, 0xfd, 0x4a, 0xbf, 0xf6, 0xaf, 0x41, 0x49, 0xf5, 0x1b};
  static const char *msg = "Cryptographic Forum Research Group";
  static const uint8_t expect[16] = {0xa8, 0x06, 0x1d, 0xc1, 0x30, 0x51, 0x36, 0xc6,
                                     0xc2, 0x2b, 0x8b, 0xaf, 0x0c, 0x01, 0x27, 0xa9};

  poly1305_context ctx;
  uint8_t mac[16];
  poly1305_init(&ctx, key);
  poly1305_update(&ctx, (const uint8_t *)msg, strlen(msg));
  poly1305_finish(&ctx, mac);
  check_bytes("poly1305 RFC8439 2.5.2 tag", mac, expect, 16);
}

/* RFC 8439 section 2.8.2 shared inputs. */
static const uint8_t aead_key[32] = {
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f};
static const uint8_t aead_aad[12] = {0x50, 0x51, 0x52, 0x53, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7};
static const char *aead_pt =
    "Ladies and Gentlemen of the class of '99: If I could offer you only one tip for the future, "
    "sunscreen would be it.";

/* ---------------------------------------------------------------------------
 * ChaCha20-Poly1305 AEAD — RFC 8439 section 2.8.2, exact ciphertext + tag.
 *
 * The production AEAD wrapper (chacha20poly1305_encrypt) hard-wires the first
 * 32 bits of the 96-bit nonce to zero (the WireGuard nonce form), so it cannot
 * express the RFC 2.8.2 nonce whose constant is 0x00000007. We therefore drive
 * the SHIPPED chacha20.c + poly1305-donna.c primitives with the exact RFC
 * nonce and reproduce the section-2.8 AEAD framing, asserting the published
 * 114-byte ciphertext and 16-byte tag byte-for-byte. This is the strongest
 * anti-miscompile anchor for the two primitives at this key/nonce.
 * ------------------------------------------------------------------------- */
static void test_aead_rfc_282(void)
{
  static const uint8_t exp_ct[114] = {
      0xd3, 0x1a, 0x8d, 0x34, 0x64, 0x8e, 0x60, 0xdb, 0x7b, 0x86, 0xaf, 0xbc, 0x53, 0xef, 0x7e, 0xc2,
      0xa4, 0xad, 0xed, 0x51, 0x29, 0x6e, 0x08, 0xfe, 0xa9, 0xe2, 0xb5, 0xa7, 0x36, 0xee, 0x62, 0xd6,
      0x3d, 0xbe, 0xa4, 0x5e, 0x8c, 0xa9, 0x67, 0x12, 0x82, 0xfa, 0xfb, 0x69, 0xda, 0x92, 0x72, 0x8b,
      0x1a, 0x71, 0xde, 0x0a, 0x9e, 0x06, 0x0b, 0x29, 0x05, 0xd6, 0xa5, 0xb6, 0x7e, 0xcd, 0x3b, 0x36,
      0x92, 0xdd, 0xbd, 0x7f, 0x2d, 0x77, 0x8b, 0x8c, 0x98, 0x03, 0xae, 0xe3, 0x28, 0x09, 0x1b, 0x58,
      0xfa, 0xb3, 0x24, 0xe4, 0xfa, 0xd6, 0x75, 0x94, 0x55, 0x85, 0x80, 0x8b, 0x48, 0x31, 0xd7, 0xbc,
      0x3f, 0xf4, 0xde, 0xf0, 0x8e, 0x4b, 0x7a, 0x9d, 0xe5, 0x76, 0xd2, 0x65, 0x86, 0xce, 0xc6, 0x4b,
      0x61, 0x16};
  static const uint8_t exp_tag[16] = {0x1a, 0xe1, 0x0b, 0x59, 0x4f, 0x09, 0xe2, 0x6a,
                                      0x7e, 0x90, 0x2e, 0xcb, 0xd0, 0x60, 0x06, 0x91};
  const size_t pt_len = strlen(aead_pt); /* 114 */

  struct chacha20_ctx ctx;
  chacha20_init(&ctx, aead_key, 0);
  ctx.state[13] = 0x00000007u; /* nonce = 07 00 00 00 | 40 41 42 43 | 44 45 46 47 */
  ctx.state[14] = 0x43424140u;
  ctx.state[15] = 0x47464544u;
  ctx.state[12] = 0u; /* block counter 0 for the Poly1305 key */

  /* 2.6: Poly1305 one-time key = first 32 bytes of keystream at counter 0. */
  uint8_t polykey[32] = {0};
  chacha20(&ctx, polykey, polykey, sizeof(polykey)); /* advances counter to 1 */

  /* 2.8: ciphertext = ChaCha20(plaintext) from counter 1. */
  uint8_t ct[114];
  chacha20(&ctx, ct, (const uint8_t *)aead_pt, (uint32_t)pt_len);
  check_bytes("chacha20-poly1305 RFC8439 2.8.2 ciphertext", ct, exp_ct, 114);

  /* 2.8: tag = Poly1305(aad | pad16 | ct | pad16 | len(aad)LE64 | len(ct)LE64). */
  static const uint8_t zero16[16] = {0};
  uint8_t lens[16] = {0};
  lens[0] = 12;  /* aad_len = 12, little-endian u64 */
  lens[8] = 114; /* ct_len  = 114, little-endian u64 */

  poly1305_context ps;
  uint8_t tag[16];
  poly1305_init(&ps, polykey);
  poly1305_update(&ps, aead_aad, sizeof(aead_aad));
  poly1305_update(&ps, zero16, 16 - (sizeof(aead_aad) % 16)); /* pad1: 4 bytes */
  poly1305_update(&ps, ct, pt_len);
  poly1305_update(&ps, zero16, (16 - (pt_len % 16)) % 16); /* pad2: 14 bytes */
  poly1305_update(&ps, lens, sizeof(lens));
  poly1305_finish(&ps, tag);
  check_bytes("chacha20-poly1305 RFC8439 2.8.2 tag", tag, exp_tag, 16);
}

/* ---------------------------------------------------------------------------
 * Production AEAD wrapper — chacha20poly1305_encrypt / _decrypt.
 *
 * Exercises the actual shipped API end to end (nonce plumbing, framing, tag
 * placement) with a byte-locked golden. The nonce is the WireGuard form
 * (constant 0x00000000 || LE64(counter)); the golden ciphertext+tag below was
 * cross-checked against Python `cryptography`'s ChaCha20Poly1305 for
 * nonce = 00000000 4041424344454647. Same key / AAD / plaintext as 2.8.2.
 * ------------------------------------------------------------------------- */
static void test_aead_wrapper(void)
{
  const uint64_t nonce = 0x4746454443424140ULL; /* LE64 -> 40 41 42 43 44 45 46 47 */
  static const uint8_t golden[130] = {
      0xa4, 0x79, 0xcb, 0x54, 0x62, 0x89, 0x46, 0xd6, 0xf4, 0x04, 0x2a, 0x8e, 0x38, 0x4e, 0xf4, 0xbd,
      0x2f, 0xbc, 0x73, 0x30, 0xb8, 0xbe, 0x55, 0xeb, 0x2d, 0x8d, 0xc1, 0x8a, 0xaa, 0x51, 0xd6, 0x6a,
      0x8e, 0xc1, 0xf8, 0xd3, 0x61, 0x9a, 0x25, 0x8d, 0xb0, 0xac, 0x56, 0x95, 0x60, 0x15, 0xb7, 0xb4,
      0x93, 0x7e, 0x9b, 0x8e, 0x6a, 0xa9, 0x57, 0xb3, 0xdc, 0x02, 0x14, 0xd8, 0x03, 0xd7, 0x76, 0x60,
      0xaa, 0xbc, 0x91, 0x30, 0x92, 0x97, 0x1d, 0xa8, 0xf2, 0x07, 0x17, 0x1c, 0xe7, 0x84, 0x36, 0x08,
      0x16, 0x2e, 0x2e, 0x75, 0x9d, 0x8e, 0xfc, 0x25, 0xd8, 0xd0, 0x93, 0x69, 0x90, 0xaf, 0x63, 0xc8,
      0x20, 0xba, 0x87, 0xe8, 0xa9, 0x55, 0xb5, 0xc8, 0x27, 0x4e, 0xf7, 0xd1, 0x0f, 0x6f, 0xaf, 0xd0,
      0x46, 0x47, 0x2d, 0xbf, 0x18, 0x9b, 0x66, 0x8b, 0xd4, 0x30, 0xae, 0xf9, 0x14, 0x7e, 0x99, 0xcb,
      0x6c, 0x89};
  const size_t pt_len = strlen(aead_pt); /* 114 */

  uint8_t out[130];
  chacha20poly1305_encrypt(out, (const uint8_t *)aead_pt, pt_len, aead_aad, sizeof(aead_aad), nonce, aead_key);
  check_bytes("chacha20-poly1305 wrapper encrypt golden", out, golden, sizeof(golden));

  /* Round-trip: decrypt recovers the plaintext and accepts the tag. */
  uint8_t dec[114];
  int ok = chacha20poly1305_decrypt(dec, out, sizeof(out), aead_aad, sizeof(aead_aad), nonce, aead_key);
  check_true("chacha20-poly1305 wrapper decrypt accepts valid tag", ok);
  check_bytes("chacha20-poly1305 wrapper round-trips plaintext", dec, (const uint8_t *)aead_pt, pt_len);

  /* Tamper the tag: decrypt must reject. */
  out[129] ^= 0x01;
  ok = chacha20poly1305_decrypt(dec, out, sizeof(out), aead_aad, sizeof(aead_aad), nonce, aead_key);
  check_true("chacha20-poly1305 wrapper rejects tampered tag", !ok);
  out[129] ^= 0x01;

  /* Tamper a ciphertext byte: decrypt must reject. */
  out[0] ^= 0x01;
  ok = chacha20poly1305_decrypt(dec, out, sizeof(out), aead_aad, sizeof(aead_aad), nonce, aead_key);
  check_true("chacha20-poly1305 wrapper rejects tampered ciphertext", !ok);
  out[0] ^= 0x01;
}

/* ---------------------------------------------------------------------------
 * BLAKE2s-256 — official KATs (cross-checked against Python hashlib.blake2s).
 * ------------------------------------------------------------------------- */
static void test_blake2s(void)
{
  uint8_t out[32];

  static const uint8_t exp_abc[32] = {
      0x50, 0x8c, 0x5e, 0x8c, 0x32, 0x7c, 0x14, 0xe2, 0xe1, 0xa7, 0x2b, 0xa3, 0x4e, 0xeb, 0x45, 0x2f,
      0x37, 0x45, 0x8b, 0x20, 0x9e, 0xd6, 0x3a, 0x29, 0x4d, 0x99, 0x9b, 0x4c, 0x86, 0x67, 0x59, 0x82};
  check_true("blake2s-256(abc) init", blake2s(out, 32, NULL, 0, "abc", 3) == 0);
  check_bytes("blake2s-256(\"abc\")", out, exp_abc, 32);

  static const uint8_t exp_empty[32] = {
      0x69, 0x21, 0x7a, 0x30, 0x79, 0x90, 0x80, 0x94, 0xe1, 0x11, 0x21, 0xd0, 0x42, 0x35, 0x4a, 0x7c,
      0x1f, 0x55, 0xb6, 0x48, 0x2c, 0xa1, 0xa5, 0x1e, 0x1b, 0x25, 0x0d, 0xfd, 0x1e, 0xd0, 0xee, 0xf9};
  check_true("blake2s-256(empty) init", blake2s(out, 32, NULL, 0, "", 0) == 0);
  check_bytes("blake2s-256(\"\")", out, exp_empty, 32);

  /* Keyed BLAKE2s (RFC 7693 style): key = 32 bytes 0x00..0x1f. */
  uint8_t key[32];
  for (int i = 0; i < 32; i++) key[i] = (uint8_t)i;
  static const uint8_t exp_keyed_abc[32] = {
      0xa2, 0x81, 0xf7, 0x25, 0x75, 0x49, 0x69, 0xa7, 0x02, 0xf6, 0xfe, 0x36, 0xfc, 0x59, 0x1b, 0x7d,
      0xef, 0x86, 0x6e, 0x4b, 0x70, 0x17, 0x3e, 0xce, 0x40, 0x2f, 0xc0, 0x1c, 0x06, 0x4d, 0x6b, 0x65};
  check_true("blake2s-256 keyed init", blake2s(out, 32, key, 32, "abc", 3) == 0);
  check_bytes("blake2s-256 keyed(\"abc\", key=00..1f)", out, exp_keyed_abc, 32);
}

int main(void)
{
  test_x25519();
  test_chacha20_block();
  test_poly1305();
  test_aead_rfc_282();
  test_aead_wrapper();
  test_blake2s();

  printf("crypto_kat: %d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
