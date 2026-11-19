# Change Requirement 3: barcode read from the sensor's analog output

**Source:** on-car bench test of 19 Nov 2026: when the barcode paper is
not flat against the floor, the IR sensor misses bars.
**Owner of the change:** Buddy 3 (barcode), with a one-line note for
Buddy 5 (shared `board_pins.h`).
**Status:** in `main`, off by default (`BAR_AO=0`). Turn it on with
`make ... BAR_AO=1` after the bench check in section 6.

## 1. The problem

The barcode used the module's digital output (DO, GP7). The module's
comparator switches DO at one fixed level, set by its potentiometer.
When the paper lifts a few millimetres:

- less IR comes back, so white reads darker (closer to black), and
- the light spot is wider, so a narrow white gap between two bars is
  blurred and never gets back down to the fixed level.

The DO then stays "black" across the gap, two bars merge into one, and the
capture has 27 or 25 elements instead of 29. Code can do nothing with a gap
the comparator never reported.

The same module's analog output (AO) was already wired to GP28 (ADC2),
which the firmware did not use (`PIN_SPARE_28`).

## 2. What changed

| Area | Before (DO) | After (`BAR_AO=1`) |
|---|---|---|
| Input | GP7 DO, both edges, R1 `gpio_bank0_isr` | GP28 AO (ADC2), sampled every 500 us (2 kHz) by a new interrupt R4 `bar_ao_timer_isr` on RP2040 TIMER ALARM0 (IRQ 0, level 2 like R1) |
| Black / white decision | Module comparator, one fixed level | `bar_ao.c`: relative threshold (section 3) |
| Edge path | R1 -> `barcode_edge_isr` -> chatter filter -> K2 ring | R4 -> `barcode_ao_sample` -> **same** chatter filter and K2 ring (`barcode_ring_push`, split out of the DO ISR) |
| Decoder, rewind, retries | `barcode_task`, `code39.c` | Unchanged |
| ADC sharing | `line_task` only | `line_task` and R4 take one SIO hardware spinlock (31; the kernel uses 0..2). `line_task` holds it with its core's interrupts masked for its 4 conversions (about 8 us); R4 only tries it and skips that one sample if busy |
| `barcode_do_level()` (console, SENSORS telemetry) | DO pin | The detector's level |
| Console `BAR DO` line (motors off, every 0.5 s) | `BAR DO 0 (1 = black), edges N` | `BAR DO 0 (1 = black), edges N, AO 1234` (the last GP28 code) |
| `board_pins.h` | `PIN_SPARE_28` | `PIN_BAR_AO`, `BOARD_ADC_CH_BAR_AO`, `BOARD_IRQ_TIMER0`, `BOARD_IRQ_PRI_TIMER0`, `BOARD_SPINLOCK_ADC` |
| Build | - | `BAR_AO ?= 0` in `pico_rp2040.mk`, image suffix `_ao` |

With `BAR_AO=0` the images behave exactly as before: R4 and the analog
state are not linked, the ADC lock is not taken, RAM is byte-identical to
`0bfca24` and flash is 4 bytes smaller (the shared ring push), and the DO
simulation output is identical (89 lines; section 5). The build also now tracks `BAR_BENCH` and `BAR_AO` in its
profile, so switching flags rebuilds every object.

## 3. The detector (`bar_ao.c`)

One 32-bit state word (`SH_BAR_AO`: LEVEL 1, PEND 1, EXT 12, OPP 8, AMP 8
bits). Samples are flipped if needed so black is high.

```
delta = max(BAR_AO_DELTA_MIN, BAR_AO_DELTA_PCT % of AMP)    // 100 codes, 35 %
on white: EXT = lowest code so far
          code >= EXT + delta for BAR_AO_CONFIRM samples -> black   // 2
on black: EXT = highest code so far
          code <= EXT - delta for BAR_AO_CONFIRM samples -> white
          AMP = (3 AMP + (this peak - last low)) / 4
```

So a switch is a **rise or dip relative to the last extreme**, not a
crossing of one fixed level. A shallow white gap between two bars (for
example from 3300 down to 2600, which a fixed midpoint would never see) is
still a gap. AMP follows the real swing, so the delta shrinks when the
contrast does. Two samples (1 ms) of confirmation stop single-sample
noise; the edges stay at least `BAR_EDGE_REJECT_US` apart (build check).

Values marked TUNE in `cfg_barcode.h`: `BAR_AO_DELTA_MIN` (100),
`BAR_AO_DELTA_PCT` (35), `BAR_AO_AMP_INIT` (800). `BAR_AO_BLACK_IS_HIGH`
(true) is MEASURE: confirm it with the console line (section 6).

## 4. Data representation (Tribunal 1)

New rows in `tools/layout.csv` (generated into `app_layout.h` and
`docs/data_representation.md`):

| Group | Bits | Writer -> readers |
|---|---|---|
| `SH_BAR_AO` (LEVEL, PEND, EXT, OPP, AMP) | 30 of 32 | R4 -> `barcode_do_level()` (line_task, telem) |
| `SH_BAR_AOS` (CODE) | 12 of 16 | R4 -> console line |
| `E_BAR_AO_US` | E5 timer | R4 only (next ALARM0 time) |

Build checks in `cfg_barcode.h` stop the build if a field cannot hold its
range. RAM in an AO image: +15 bytes (state word, last code, alarm time,
callback pointer, ADC-ready flag).

## 5. Verification (host)

`testing/subsystem3/host/test_line_barcode_sim_host.c`, recorded in
`testing/subsystem3/TEST_LOG.md`:

- **S3-U52**, detector: no edge in 10 s of noise (+-60 codes) on white or
  black; a 700-code dip from a 3300 bar is a gap; one sample alone does not
  switch.
- **S3-U53**, lifted paper, full `barcode.c` path: all 43 characters
  through a sensor model (box-blurred spot, white/black codes, noise).

| Case | DO (fixed level) | AO |
|---|---|---|
| Flat: 4 ms spot, white 400 / black 3400 | 43 / 43 | 43 / 43 |
| Lifted: 14 ms spot (wider than a 12 ms narrow bar), 1800 / 3300 | **0 / 43** | **43 / 43** |
| Lifted further: 14 ms spot, 2200 / 3200 | - | 42 / 43 |
| 440 reads, contrast 3000..500 codes x spot 4..18 ms | - | **0 wrong characters** |

**Limit:** once the spot is wider than about 1.15 narrow bars, every
threshold sees the narrow element as wide as the spot, and the read fails.
It always fails safely: no event or INVALID, never a wrong character. Delta
25, 35, 40 and 50 % all hit the same limit, so it is optical, not the
detector. Driving slower does not help, because the blur is a distance.

DO path: the old and new simulations print identical output (89 lines).
All host suites: 890 pass. All six images (`make`, MQTT, debug, debug AO,
bench, bench AO) build with `-Werror` and 0 warnings.

## 6. Car check (to do)

Flash the bench image built with
`make NET=lwip MQTT=1 DEBUG_MQTT=1 BAR_BENCH=1 BAR_AO=1`.

1. **Polarity and contrast, motors off.** Hold the car still over white,
   then over a black bar. Read the dashboard console line `BAR DO ...
   AO N`. AO must be clearly higher on black (if it is lower, set
   `BAR_AO_BLACK_IS_HIGH` to false). Write the two values here; the black
   and white codes should differ by several hundred at least.
2. **Flat paper:** bench runs as before; expect `BENCH BARCODE: 'A'
   decoded` on pass 1.
3. **Lifted paper:** the case that failed on 19 Nov. Same strip, lifted as
   it was. Expect a decode within the 4 passes.
4. If a pass fails, export the console (`Car console -> Export .txt`) and
   keep the `BAR edge dump` lines.

When 1-3 pass, make `BAR_AO=1` the default for the course image (or build
the course image with it) and add a TEST_LOG row.

## 7. Requirement text

FR2 (barcode detection) is met with either input; the decoder and the
FR3 command map are unchanged. Add to the FR2 implementation notes:
*"Barcode bars are found from the IR module's analog output (GP28,
sampled at 2 kHz) with a relative threshold, so paper that is not flat
against the floor is still read; the digital output (GP7) remains as a
build option."* The hardware/wiring table gains GP28 = barcode AO
(it was already wired), and the interrupt table gains R4 (TIMER ALARM0,
2 kHz, level 2).
