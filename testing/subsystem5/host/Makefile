# Buddy 5 host (laptop) tests.
#   test_obs_profile_host : S5-U01..U03, pure obs_profile.c
#   sim_mission_host      : statechart simulation (mission.c, mission_run.c),
#                           plus S5-U04 and S5-U05 (-DMISSION_HOST_TEST)
#   sim_obstacle_host     : obstacle_task simulation (obstacle.c + profile)
# The simulations compile copies of the real sources in build/ next to a
# mock app.h, so "../app.h" inside them resolves to the mock.
# Run: make -C testing/subsystem5/host   (also called by testing/host)
CC      ?= gcc
CFLAGS  := -std=c99 -Wall -Wextra -Werror -pedantic -I../../../app_program
SIMFLAGS := -std=gnu99 -Wall -Wno-unused-function -Ibuild -Ibuild/app_program
APP     := ../../../app_program
S5      := $(APP)/subsystem5
B       := build/app_program

.PHONY: all clean tree

all: test_obs_profile_host.out sim_mission_host.out sim_obstacle_host.out
	./test_obs_profile_host.out
	./sim_mission_host.out
	./sim_obstacle_host.out

test_obs_profile_host.out: test_obs_profile_host.c $(S5)/obs_profile.c \
                           $(S5)/obs_profile.h $(S5)/cfg_obstacle.h
	$(CC) $(CFLAGS) -o $@ test_obs_profile_host.c $(S5)/obs_profile.c

tree:
	rm -rf build
	mkdir -p $(B)/subsystem1 $(B)/subsystem2 $(B)/subsystem3 $(B)/subsystem5
	cp mock/app.h $(B)/app.h
	mkdir -p build/hardware
	cp mock/hardware/gpio.h build/hardware/gpio.h
	cp $(APP)/app_types.h $(APP)/board_pins.h $(B)/
	cp $(APP)/subsystem1/telem.h $(APP)/subsystem1/telem_bin.h $(B)/subsystem1/
	cp $(APP)/subsystem2/safety.h $(APP)/subsystem2/fault.h \
	   $(APP)/subsystem2/motion.h $(APP)/subsystem2/odometry.h \
	   $(APP)/subsystem2/cfg_motion.h $(B)/subsystem2/
	cp $(APP)/subsystem3/barcode.h $(B)/subsystem3/
	cp $(S5)/*.c $(S5)/*.h $(B)/subsystem5/

sim_mission_host.out: sim_mission_host.c tree
	$(CC) $(SIMFLAGS) -DMISSION_HOST_TEST -o $@ sim_mission_host.c \
	    $(B)/subsystem5/mission_run.c $(B)/subsystem5/status_board.c

sim_obstacle_host.out: sim_obstacle_host.c tree
	$(CC) $(SIMFLAGS) -o $@ sim_obstacle_host.c \
	    $(B)/subsystem5/obs_profile.c $(B)/subsystem5/status_board.c -lm

clean:
	rm -rf *.out build
