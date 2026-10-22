################################################################################
# Host (laptop) tests of subsystem 3 (Buddy 3 — Lucas).
# Run alone with "make -C testing/subsystem3/host", or all buddies' tests
# with "make -C testing/host". Any FAIL fails make.
#
#   S3-U01, U02, U06, U14  test_line_track_host.c        (line_track.c)
#   S3-U03, U04, U05       test_code39_host.c            (code39.c)
#   S3-U07, U08, U09, U14  test_line_barcode_sim_host.c  (line.c + barcode.c)
################################################################################

CC      := gcc
APP_DIR := ../../../app_program
S2_DIR  := $(APP_DIR)/subsystem2
S3_DIR  := $(APP_DIR)/subsystem3
CFLAGS  := -std=c99 -Wall -Wextra -Werror -pedantic \
           -I../../host -I$(APP_DIR) -I$(S3_DIR)
MOCK    := -Imock

.PHONY: all line_track code39 sim clean
all: line_track code39 sim

test_line_track_host.out: test_line_track_host.c $(S3_DIR)/line_track.c \
                          $(S3_DIR)/line_track.h $(S3_DIR)/cfg_line.h \
                          $(APP_DIR)/app_types.h ../../host/host_test.h
	$(CC) $(CFLAGS) -o $@ test_line_track_host.c $(S3_DIR)/line_track.c

test_code39_host.out: test_code39_host.c $(S3_DIR)/code39.c \
                      $(S3_DIR)/code39.h $(S3_DIR)/cfg_barcode.h \
                      $(APP_DIR)/app_types.h ../../host/host_test.h
	$(CC) $(CFLAGS) -o $@ test_code39_host.c $(S3_DIR)/code39.c

test_line_barcode_sim_host.out: test_line_barcode_sim_host.c \
                                $(S3_DIR)/line.c $(S3_DIR)/barcode.c \
                                $(S3_DIR)/line_track.c $(S3_DIR)/code39.c \
                                $(S2_DIR)/odometry.c ../../host/host_test.h
	$(CC) $(CFLAGS) $(MOCK) -o $@ test_line_barcode_sim_host.c \
	    $(S3_DIR)/line_track.c $(S3_DIR)/code39.c $(S2_DIR)/odometry.c

line_track: test_line_track_host.out
	./test_line_track_host.out

code39: test_code39_host.out
	./test_code39_host.out

sim: test_line_barcode_sim_host.out
	./test_line_barcode_sim_host.out

clean:
	rm -f *.out
