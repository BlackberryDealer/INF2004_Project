################################################################################
# Host (laptop) tests of subsystem 1. Owner: Buddy 1.
# Run alone with "make -C testing/subsystem1/host", or all buddies' tests
# with "make -C testing/host". Any FAIL fails make.
#
#   S1-U20..U23  test_app_bits_host.c    (app_bits.c + generated app_layout.h)
#   S1-U24       gen_layout.py rejects a broken copy of tools/layout.csv
#   S1-U01, U02 (packed), S1-U10  test_telem_bin_host.c (binary builders;
#                writes the golden vectors in golden/)
#   S1-U11, U12, U30..U35  test_mqtt_bridge_host.c (mqtt_bridge.c + mock
#                kernel; links telem_bin.c, app_bits.c, odometry.c)
#   S1-U13, U15, U36, U37  test_mqtt_utk_host.c (radio-task mqtt_utk.c +
#                mock lwIP in mock_lwip/ + mock bridge)
#
# Kernel-side files (mqtt_bridge.c) are copied into build/ next to a mock
# app.h, so "../app.h" inside them resolves to the mock.
################################################################################

CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -Werror -pedantic \
           -I../../host -I../../../app_program -I../../../app_program/subsystem1
PYTHON  := python3

APP_DIR := ../../../app_program
S1      := $(APP_DIR)/subsystem1
TOOLS   := ../../../tools
LAYOUT  := $(APP_DIR)/app_layout.h
B       := build/app_program

# Every SB_/MSG_ field has one _POS macro; the test checks its table has
# the same number of entries.
SB_MSG_FIELDS := $(shell grep -cE '^\#define (SB|MSG)_[A-Z0-9_]+_POS ' $(LAYOUT))

.PHONY: all app_bits gen_layout_reject mqtt_bridge mqtt_utk tree \
        telem_bin clean
all: app_bits gen_layout_reject telem_bin mqtt_bridge mqtt_utk

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

test_telem_bin_host.out: test_telem_bin_host.c $(S1)/telem_bin.c \
                         $(S1)/telem_bin.h $(APP_DIR)/app_bits.c \
                         $(APP_DIR)/subsystem2/odometry.c $(LAYOUT)
	$(CC) $(CFLAGS) -o $@ test_telem_bin_host.c $(S1)/telem_bin.c \
	    $(APP_DIR)/app_bits.c $(APP_DIR)/subsystem2/odometry.c

telem_bin: test_telem_bin_host.out
	mkdir -p golden
	./test_telem_bin_host.out

tree:
	rm -rf build
	mkdir -p $(B)/subsystem1 $(B)/subsystem2 $(B)/subsystem5
	cp mock/app.h $(B)/app.h
	cp $(APP_DIR)/app_types.h $(APP_DIR)/app_layout.h $(APP_DIR)/app_bits.h \
	   $(APP_DIR)/app_bits.c $(B)/
	cp $(S1)/*.c $(S1)/*.h $(B)/subsystem1/
	cp $(APP_DIR)/subsystem2/fault.h $(APP_DIR)/subsystem2/odometry.h \
	   $(APP_DIR)/subsystem2/odometry.c $(B)/subsystem2/
	cp $(APP_DIR)/subsystem5/mission.h $(APP_DIR)/subsystem5/sys_tick.h \
	   $(B)/subsystem5/

test_mqtt_bridge_host.out: test_mqtt_bridge_host.c tree
	$(CC) -std=c99 -Wall -Wextra -Werror -pedantic -I../../host \
	    -I$(B)/subsystem1 -I$(B) -o $@ test_mqtt_bridge_host.c \
	    $(B)/subsystem1/telem_bin.c $(B)/app_bits.c $(B)/subsystem2/odometry.c

mqtt_bridge: test_mqtt_bridge_host.out
	./test_mqtt_bridge_host.out

# mqtt_utk.c is template code (tabs, lwIP types; deviation D3): built with
# the same warnings, in gnu99 like the rest of the template.
MQTT_UTK := ../../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c
test_mqtt_utk_host.out: test_mqtt_utk_host.c $(MQTT_UTK) $(S1)/mqtt_bridge.h \
                        $(wildcard mock_lwip/*.h mock_lwip/*/*.h \
                                   mock_lwip/*/*/*.h)
	$(CC) -std=gnu99 -Wall -Wextra -Werror -I../../host -Imock_lwip \
	    -o $@ test_mqtt_utk_host.c

mqtt_utk: test_mqtt_utk_host.out
	./test_mqtt_utk_host.out

clean:
	rm -rf *.out build
