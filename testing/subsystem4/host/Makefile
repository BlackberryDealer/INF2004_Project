################################################################################
# Host (laptop) tests of subsystem 4's pure modules. Owner: Buddy 4.
# Run alone with "make -C testing/subsystem4/host", or all buddies' tests
# with "make -C testing/host". Any FAIL fails make.
#
#   S4-U01 U02 U03 U06 U07 U12 U13   test_terrain_host.c   (terrain.c)
#   S4-U14                           test_drv_lsm303_host.c (fifo planner)
################################################################################

CC      := gcc
APP_DIR := ../../../app_program
S4_DIR  := $(APP_DIR)/subsystem4
S2_DIR  := $(APP_DIR)/subsystem2
CFLAGS  := -std=c99 -Wall -Wextra -Werror -pedantic -Wconversion \
           -Wsign-conversion -I../../host -I$(APP_DIR) -I$(S4_DIR) \
           -I$(S2_DIR)

.PHONY: all terrain fifo
all: terrain fifo

test_terrain_host.out: test_terrain_host.c $(S4_DIR)/terrain.c \
                       $(S4_DIR)/terrain.h $(S4_DIR)/cfg_imu.h \
                       $(S2_DIR)/odometry.c $(S2_DIR)/odometry.h \
                       $(APP_DIR)/app_types.h $(APP_DIR)/app_bits.c \
                       $(APP_DIR)/app_bits.h $(APP_DIR)/app_layout.h \
                       ../../host/host_test.h
	$(CC) $(CFLAGS) -o $@ test_terrain_host.c $(S4_DIR)/terrain.c \
	    $(S2_DIR)/odometry.c $(APP_DIR)/app_bits.c -lm

terrain: test_terrain_host.out
	./test_terrain_host.out

test_drv_lsm303_host.out: test_drv_lsm303_host.c $(S4_DIR)/drv_lsm303.h \
                          ../../host/host_test.h
	$(CC) $(CFLAGS) -o $@ test_drv_lsm303_host.c

fifo: test_drv_lsm303_host.out
	./test_drv_lsm303_host.out
