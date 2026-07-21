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

CFLAGS   ?= -O2 -Wall -Wextra -Wno-unused-parameter
CPPFLAGS += $(PSTOP_INC)

# Note: upstream's pstop/time.c (Linux-only) is intentionally included for
# the host build — that's the path it was written for. The chip uses a
# port file under components/pstop/port/ instead; this Makefile doesn't.

machine_app_runner: machine_app_runner.c $(PSTOP_SRCS) $(TRANS_SRCS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^

clean:
	rm -f machine_app_runner

.PHONY: clean
