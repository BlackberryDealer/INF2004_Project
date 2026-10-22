################################################################################
# Host (laptop) tests of subsystem 2's pure modules. Owner: Buddy 2.
# Run alone with "make -C testing/subsystem2/host", or all buddies' tests
# with "make -C testing/host". Any FAIL fails make.
#
#   S2-U01, S2-U11  test_odometry_host.c     (odometry.c)
#   S2-U02          test_speed_pid_host.c    (speed_pid.c)
#   S2-U13, S2-U14  test_flags_host.c        (safety.c, fault.c on isolated
#                                             mock kernel event flags)
#   S2-U03, U12, U15..U19  test_motion_task_host.c  (motion_task.c simulated
#                   with mocks; complements the Phase 3 on-target runs)
# mock/tk/tkernel.h stands in for the kernel header on the laptop.
################################################################################

CC      := gcc
APP_DIR := ../../../app_program
S2_DIR  := $(APP_DIR)/subsystem2
CFLAGS  := -std=c99 -Wall -Wextra -Werror -pedantic -Wconversion \
           -Wsign-conversion -I../../host -I$(APP_DIR) -I$(S2_DIR)

MOCK    := -Imock

.PHONY: all odometry speed_pid flags motion_task
all: odometry speed_pid flags motion_task

test_odometry_host.out: test_odometry_host.c $(S2_DIR)/odometry.c \
                        $(S2_DIR)/odometry.h $(APP_DIR)/app_types.h \
                        ../../host/host_test.h
	$(CC) $(CFLAGS) -o $@ test_odometry_host.c $(S2_DIR)/odometry.c

test_speed_pid_host.out: test_speed_pid_host.c $(S2_DIR)/speed_pid.c \
                         $(S2_DIR)/speed_pid.h $(S2_DIR)/cfg_motion.h \
                         ../../host/host_test.h
	$(CC) $(CFLAGS) -o $@ test_speed_pid_host.c $(S2_DIR)/speed_pid.c

odometry: test_odometry_host.out
	./test_odometry_host.out

speed_pid: test_speed_pid_host.out
	./test_speed_pid_host.out

test_flags_host.out: test_flags_host.c mock_kernel.c mock/tk/tkernel.h \
                     $(S2_DIR)/safety.c $(S2_DIR)/fault.c \
                     $(S2_DIR)/safety.h $(S2_DIR)/fault.h ../../host/host_test.h
	$(CC) $(CFLAGS) $(MOCK) -o $@ test_flags_host.c mock_kernel.c \
	    $(S2_DIR)/safety.c $(S2_DIR)/fault.c

test_motion_task_host.out: test_motion_task_host.c mock/tk/tkernel.h \
                           $(S2_DIR)/motion_task.c $(S2_DIR)/odometry.c \
                           $(S2_DIR)/speed_pid.c $(S2_DIR)/cfg_motion.h \
                           ../../host/host_test.h
	$(CC) $(CFLAGS) $(MOCK) -o $@ test_motion_task_host.c \
	    $(S2_DIR)/odometry.c $(S2_DIR)/speed_pid.c

flags: test_flags_host.out
	./test_flags_host.out

motion_task: test_motion_task_host.out
	./test_motion_task_host.out
