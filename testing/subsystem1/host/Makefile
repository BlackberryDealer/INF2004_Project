################################################################################
# Host (laptop) tests of subsystem 1's pure modules. Owner: Buddy 1.
# Run alone with "make -C testing/subsystem1/host", or all buddies' tests
# with "make -C testing/host". Any FAIL fails make.
#
#   S1-U20..U23  test_app_bits_host.c  (app_bits.c + generated app_layout.h)
#   S1-U24       gen_layout.py rejects a broken copy of tools/layout.csv
################################################################################

CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -Werror -pedantic \
           -I../../host -I../../../app_program -I../../../app_program/subsystem1
PYTHON  := python3

APP_DIR := ../../../app_program
TOOLS   := ../../../tools
LAYOUT  := $(APP_DIR)/app_layout.h

# Every SB_/MSG_ field has one _POS macro; the test checks its table has
# the same number of entries.
SB_MSG_FIELDS := $(shell grep -cE '^\#define (SB|MSG)_[A-Z0-9_]+_POS ' $(LAYOUT))

.PHONY: all app_bits gen_layout_reject
all: app_bits gen_layout_reject

test_app_bits_host.out: test_app_bits_host.c $(APP_DIR)/app_bits.c \
                        $(APP_DIR)/app_bits.h $(LAYOUT) ../../host/host_test.h
	$(CC) $(CFLAGS) -DLAYOUT_SB_MSG_FIELDS=$(SB_MSG_FIELDS) -o $@ \
	    test_app_bits_host.c $(APP_DIR)/app_bits.c

app_bits: test_app_bits_host.out
	./test_app_bits_host.out

# S1-U24: widen SB_LINE0.STATUS from 2 to 3 bits; the generator must refuse
# it ("wider than needed") with exit code 1 and write nothing (--check).
gen_layout_reject:
	@tmp=$$(mktemp); \
	sed 's/^SB_LINE0,32,STATUS,2,/SB_LINE0,32,STATUS,3,/' \
	    $(TOOLS)/layout.csv > $$tmp; \
	if grep -q '^SB_LINE0,32,STATUS,3,' $$tmp; then :; else \
	    echo "[TEST] S1-U24 FAIL  could not build the broken CSV"; \
	    rm -f $$tmp; exit 1; fi; \
	$(PYTHON) $(TOOLS)/gen_layout.py --check $$tmp; rc=$$?; rm -f $$tmp; \
	if [ $$rc -eq 1 ]; then \
	    echo "[TEST] S1-U24 PASS  gen_layout.py rejects a too-wide field"; \
	else \
	    echo "[TEST] S1-U24 FAIL  gen_layout.py exit $$rc, expected 1"; \
	    exit 1; \
	fi
