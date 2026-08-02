# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Host unit tests + structural coverage (statement/branch/MC-DC) for the pure
# E-stop decision core firmware/main/estop_verdict.c. The on-target Xtensa build
# cannot be gcov-instrumented (its JTAG pins are the E-stop loop pins GPIO39-42),
# so the SIL-critical decision logic is covered here on the host with GCC-14
# (-fcondition-coverage = MC/DC). See docs/safety/COVERAGE.md.
#
#   make run        # build + run the tests
#   make coverage   # + line/branch/condition summary (needs gcc-14 + gcovr)

CC      ?= gcc-14
GCOV    ?= gcov-14
GCOVR   ?= gcovr
CFLAGS  ?= -O0 -g -Wall -Wextra --coverage -fcondition-coverage
INC     := -I../main -I../../pstop_c/pstop/include
SRCS    := ../main/estop_verdict.c test_estop_verdict.c

test_estop_verdict: $(SRCS)
	$(CC) $(CFLAGS) $(INC) -o $@ $(SRCS)

.PHONY: run coverage clean
run: test_estop_verdict
	./test_estop_verdict

coverage: run
	$(GCOVR) --root .. --gcov-executable $(GCOV) \
	  --filter '.*/estop_verdict\.c' \
	  --print-summary --txt --decisions \
	  --json-summary-pretty -o coverage-summary.json

clean:
	rm -f test_estop_verdict *.gcda *.gcno *.gcov coverage-summary.json
