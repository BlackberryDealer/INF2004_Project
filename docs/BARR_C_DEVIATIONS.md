# BARR-C:2018 deviations — CSC28 PicoCar

Every place where the firmware knowingly departs from a BARR-C:2018 rule, why,
and who approved it. A deviation not listed here is a defect. New rows are
added only through a `SHARED:` pull request reviewed by the whole team.

Scope: application code in `app_program/` (compiled as C99, see D8). Kernel,
Pico SDK, lwIP and other template code under `kernel/`, `lib/`, `device/` and
`include/` is third-party and outside BARR-C scope, except the template file
`mqtt_utk.c` that Buddy 1 extends (D3).

| ID | Rule | Deviation | Justification | Approved by |
|---|---|---|---|---|
| D1 | 5.1.a, 5.2.a | µT-Kernel types (`ID`, `ER`, `INT`, `UINT`, `TMO`, `PRI`, `FP`, `SZ`, `UW`, `T_C*`, `T_R*`) are used, but only at kernel calls and in the signatures the kernel dictates. Everywhere else the code uses `<stdint.h>` / `<stdbool.h>` types. | The kernel API is defined with these types; converting at every call would add casts without adding safety. | Team, Phase 0 skeleton PR |
| D2 | 6.1.f, 7.1.e | Task entry functions have the fixed signature `void x_task(INT stacd, void * p_exinf)`: a kernel `INT` instead of a fixed-width type, and the kernel's parameter names. Each task casts the entry to `FP` once, in its `_start()` function, with a comment. | The signature is fixed by `tk_cre_tsk()` / `T_CTSK.task`. | Team, Phase 0 skeleton PR |
| D3 | 3.5.a and related layout rules | Unchanged template lines in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` keep their original style (tabs, brace placement). Only the lines Buddy 1 adds follow BARR-C. | Rewriting template code would hide the real diff in review and risk breaking the radio task. | Team, Phase 0 skeleton PR |
| D4 | — | *(retired)* | — | — |
| D5 | Project minimum-bit rule (Tribunal 1) | Exceptions E1–E8 are stored wider than the bits they need: E1 PID integrators, E2 Δh accumulator and calibration sums, E3 PIO count images, E4 accelerometer register image, E5 timer images whose interval can exceed 65 ms, E6 ring head/tail bytes, E7 per-task latency/overrun counters, E8 precomputed line reciprocals. Each has a class `E` row in `tools/layout.csv` stating the bits actually needed. | Arithmetic headroom (E1, E2, E8), hardware register width with modular subtraction (E3, E4, E5), and one-writer-per-word concurrency splits (E6, E7). The list is closed and goes to the professor as question 1; the team does not self-certify it. | Team; **pending professor** |
| D6 | 6.5.a | `gpio_bank0_isr` is a plain C function registered with `tk_def_int()`, not a compiler-specific interrupt function. | µT-Kernel dispatches interrupts itself and calls a normal C function; no `__attribute__((interrupt))` is needed or allowed (1.1.c). | Team, Phase 0 skeleton PR |
| D7 | 6.3.a | The template's `TEST_INIT` / `TEST_CHECK` / `TEST_SUMMARY` (`app_program/test_report.h`) and the host `HOST_INIT` / `HOST_CHECK` / `HOST_SUMMARY` (`testing/host/host_test.h`) are function-like macros. | They need `__FILE__` / `__LINE__` of the caller, which a function cannot obtain. Test code only. `test_report.h` is a course template kept unchanged (Part B.5), so `tools/barr_check.sh` skips it. | Team, Phase 0 skeleton PR |
| D8 | 1.1.a (compiler flags) | **No flag left out.** Application objects are built with `-std=c99 -Wall -Wextra -pedantic -Wconversion -Wsign-conversion -Werror` (`build_make/mtkernel_3/app_program/subdir.mk`). The Phase 0 skeleton builds with zero warnings in the normal, `TEST=1` and `PROBE=1` images. | Measured on the skeleton (21 Oct 2026, arm-none-eabi-gcc 13.2.1, Pico SDK 2.2.0). If a later kernel or SDK header makes one of these flags warn, remove only that flag and record it in this row. | Team, Phase 0 skeleton PR |

## Build rule found while measuring D8 (not a deviation)

A source file that includes `app.h` (and therefore the µT-Kernel headers) must
**not** also include a Pico SDK `hardware/*.h` or `pico/*.h` header. Both
define `size_t` and the RP2040 peripheral base addresses (`IO_BANK0_BASE`,
`SIO_BASE`, `PWM_BASE`, …) differently, and the file stops compiling. Keep SDK
calls in a `drv_*.c` file that includes only SDK headers, and give it a
`drv_*.h` that includes only `<stdint.h>` / `<stdbool.h>`. Kernel-side files
that only need a pin set or a register read can use the kernel's own helpers
(`<bsp/libbsp.h>` `gpio_set_pin()` / `gpio_set_val()`, or `in_w()` / `out_w()`
with the `sysdef.h` register macros), as `board_pins.c` and `app_main.c` do.

## SDK sources linked by Phase 0 (P0.1, C2)

| SDK source | Result | Consequence |
|---|---|---|
| `hardware_adc/adc.c` | Builds and links | line_task (Buddy 3) may call the SDK ADC functions from `drv_ir_adc.c` |
| `hardware_i2c/i2c.c` | **Not linkable**: its header needs `pico/time.h` (`pico_time`), which the template does not provide | Buddy 4 uses the register-level I2C fallback in `drv_lsm303.c` |
| `hardware_watchdog/watchdog.c` | **Not linkable**: needs `hardware/ticks.h` and `pico/bootrom.h`, not in the template | Buddy 5 drives the watchdog registers directly in `sys_health.c` |

The include paths of all three (plus `hardware_pwm`) stay in `WIFI_INCLUDES`
so their register definitions remain available.
