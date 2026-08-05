idf_component_register(
    SRCS
        "src/wireguard.c"
        "src/wireguardif.c"
        "src/crypto.c"
        "src/wireguard-platform-esp32.c"
        "src/crypto/refc/blake2s.c"
        "src/crypto/refc/chacha20.c"
        "src/crypto/refc/chacha20poly1305.c"
        "src/crypto/refc/poly1305-donna.c"
        "src/crypto/refc/x25519.c"
    INCLUDE_DIRS
        "src"
        "src/crypto/refc"
    REQUIRES
        lwip
        esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    WIREGUARD_CRYPTO_REFC=1
)

# Disable stringop-overread warning for x25519.c (false positive in GCC array bounds check)
set_source_files_properties(
    src/crypto/refc/x25519.c
    PROPERTIES COMPILE_FLAGS "-Wno-stringop-overread"
)

# Crypto speedup (from fast-crypto, 2026-08-04): -O3 on the vetted per-packet AEAD
# primitives (project default -Og). Same code, output verified identical on-device.
# See docs/CRYPTO_SPEEDUP_REPORT.md.
set_source_files_properties(
    src/crypto/refc/chacha20poly1305.c
    src/crypto/refc/chacha20.c
    src/crypto/refc/poly1305-donna.c
    src/crypto/refc/blake2s.c
    PROPERTIES COMPILE_OPTIONS "-O3"
)
