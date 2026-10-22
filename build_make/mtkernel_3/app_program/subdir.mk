################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

# CSC28 PicoCar (C12): application code is compiled as strict C99
# (BARR-C 1.1.a). The later -std=c99 overrides the kernel's -std=gnu11.
# Every flag below was tried on the Phase 0 skeleton and kept: the kernel
# headers build clean with all of them (docs/BARR_C_DEVIATIONS.md, D8).
# Note: a file that includes app.h (kernel headers) must not also include a
# Pico SDK hardware/ header -- the two define size_t and the peripheral base
# addresses differently. SDK calls live in drv_*.c files without app.h.
APP_CFLAGS := -std=c99 -Wall -Wextra -pedantic -Wconversion -Wsign-conversion \
              -Werror

# Third-party headers (Pico SDK, TinyUSB, lwIP, CYW43 and their template
# ports) are not written for the flags above, so for app objects their
# include paths become system paths (-isystem): their own warnings are not
# reported, our code is still checked in full. The SDK's hardware/structs
# headers use C11 static_assert; in C99 that is GCC's _Static_assert.
# Kernel headers (INCPATH) stay -I and are checked like our own code.
APP_SYSCFLAGS = $(subst -I",-isystem ",$(CFLAGS)) \
                "-Dstatic_assert=__extension__ _Static_assert"

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(APP_SYSCFLAGS) $(APP_CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
