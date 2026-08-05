#  Makefile — laptop-side pstop machine for the pstop remote.
#
#  No ESP-IDF needed. Plain C, links directly against the vendored pstop_c
#  sources at the top-level pstop_c/ folder.
#
#  Build:   make
#  Run:     ./machine_app_runner [port]   # default port 8890
#
#  Clean:   make clean

PSTOP_ROOT  := ../pstop_c
PSTOP_INC   := -I$(PSTOP_ROOT)/pstop/include -I$(PSTOP_ROOT)/transport/include

PSTOP_SRCS  := $(wildcard $(PSTOP_ROOT)/pstop/src/pstop/*.c)
TRANS_SRCS  := $(PSTOP_ROOT)/transport/src/transport/udp/udp_transport.c

CFLAGS   ?= -O2 -Wall -Wextra -Wno-unused-parameter -pthread
CPPFLAGS += $(PSTOP_INC)

# Note: upstream's pstop/time.c (Linux-only) is intentionally included for
# the host build — that's the path it was written for. The chip uses a
# port file under components/pstop/port/ instead; this Makefile doesn't.

# clock_guard.c: pure machine-side clock-freeze detector (SR-H-04b / DU-2).
machine_app_runner: machine_app_runner.c clock_guard.c $(PSTOP_SRCS) $(TRANS_SRCS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^

# Host unit test for the clock guard — no hardware, no pstop_c needed.
test_clock_guard: test_clock_guard.c clock_guard.c
	$(CC) $(CFLAGS) -o $@ $^

# Golden-vector KATs for the safety-path crypto primitives switched to -O3.
# The primitive .c files are compiled straight into the test at -O3 (matching
# production) so the test would catch an -O3 miscompile. No ESP-IDF: these are
# portable C and don't need the SDK. See test_crypto_kat.c for vector sources.
MICROLINK   := ../components/microlink
REFC        := $(MICROLINK)/components/wireguard_lwip/src/crypto/refc
CRYPTO_SRCS := \
	$(MICROLINK)/src/x25519.c \
	$(REFC)/chacha20.c \
	$(REFC)/poly1305-donna.c \
	$(REFC)/chacha20poly1305.c \
	$(REFC)/blake2s.c \
	$(MICROLINK)/components/wireguard_lwip/src/crypto.c

test_crypto_kat: test_crypto_kat.c $(CRYPTO_SRCS)
	$(CC) -O3 -Wall -Wextra -Wno-unused-parameter -o $@ $^

.PHONY: test clean
test: test_clock_guard test_crypto_kat
	./test_clock_guard
	./test_crypto_kat

clean:
	rm -f machine_app_runner test_clock_guard test_crypto_kat
