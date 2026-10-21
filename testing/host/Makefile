################################################################################
# Runs every buddy's host (laptop) tests:   make -C testing/host
# Each testing/subsystemN/host/Makefile builds and runs that buddy's tests
# with gcc -std=c99 -Wall -Wextra -Werror -pedantic and fails on any FAIL.
# This target fails as soon as one of them fails.
################################################################################

SUBDIRS := ../subsystem1/host \
           ../subsystem2/host \
           ../subsystem3/host \
           ../subsystem4/host \
           ../subsystem5/host

.PHONY: all
all:
	@set -e; for dir in $(SUBDIRS); do $(MAKE) --no-print-directory -C $$dir; done
	@echo "host tests: all subsystems passed"
