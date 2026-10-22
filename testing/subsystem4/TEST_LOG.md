# Subsystem 4 — test log

**Subsystem:** IMU, terrain and the RAM report (Buddy 4)
**Student:** Faris Zharfan, 2500841
**Code:** `app_program/subsystem4/` · **Host tests:** `testing/subsystem4/host/`
**Branch:** `b4-imu` · commit prefix `S4:`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change (spec B4).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | Terrain Monitor replay of a recorded hump log | FR4, FR5, NFR7 | auto (host) | `make -C testing/subsystem4/host terrain` | One hump; peak within ±20 % or ±5 mm of the ruler value |
| S4-U02 | Ramp longer than `TERRAIN_MAX_CLIMB_MM` | FR4 | auto (host) | host replay, synthetic ramp | No hump reported |
| S4-U03 | Integer `atan2` vs `math.h` on a 1 deg grid | B4.2.1 | auto (host) | host grid comparison (may use double) | Error ≤ 0.2 deg |
| S4-U06 | Long hump at 150 mm/s (synthetic) | FR4 (distance, not time, decides ramp) | auto (host) | host replay, synthetic 150 mm/s log | Hump is not rejected as a ramp |
| S4-U07 | Synthetic slip on one wheel | NFR7 | auto (host) | host replay, one wheel reads double travel | Height changes ≤ 10 % |
| S4-U12 | `a_fwd` on a logged speed trace | B4.2.2 step 5 | auto (host) | speed history replay | Matches the hand calculation |
| S4-U13 | Run start: peak reset and odometry rebase | R03 / B4.2.2 step 4 | auto (target) | `test_imu.c`, `APP_MODE_RUN_START` 30 ms | Run peak returns to 0; `ds = 0`, no false climb |
| S4-U14 | FIFO overrun and backlog handling | R14 / B4.1.1 | auto (host) | `drv_lsm303_fifo_plan` on FIFO_SRC images | Overrun: no samples used, counter +1; backlog of 6 → 4 now and 2 next cycle |
| S4-U10 | Packed pitch pack/unpack (Phase 4) | SB_IMU0_PITCH | auto (host) | `app_bits_encode`/`decode` at −45, 0, +45 deg, clamp beyond | Exact stored values; clamp sets the flag |
| S4-U11 | Hump 127/128 clamp sets RANGE_CLAMP (Phase 4) | SB_IMU0_HUMP_* | auto (target) | `test_imu.c` with the packed status board | 127 passes; 128 sets `APP_FAULT_RANGE_CLAMP` |
| S4-I01 | Flat cruise, no false hump | NFR7 | track | 3 runs at cruise | No hump event |
| S4-I02 | Humps 15/25/40 mm, short and long, 150 and 250 mm/s | FR4, FR5, NFR7 | track | 5 runs each, CSVs committed | All detected within ±20 % or ±5 mm |
| S4-I03 | Impact taps from front/back/left/right | FR13 | bench | 5 taps each side; log `INT1_SRC_A`; with Buddy 2 | INT1 fires every time; motors low ≤ 15 ms |
| S4-I04 | 10 on-the-spot turns: IMU vs encoder rate | A7 | floor | turn left/right, compare rates | Within 10 % or flagged `b_is_turn_from_enc` |
| S4-I05 | Unplug SDA | B4.2.2 step 2 | bench | pull SDA during RUN | Fault after 3 errors; recovers on reconnect |
| S4-I06 | FIFO burst read equals single reads; identity | B4.1.1 verify wrap (LSM303D: SQ-B4-2) | bench | burst of 4 vs 4 single reads; read WHO_AM_I (0x0F) | Equal samples; identity `0x49` on address 0x1D |
| S4-I07 | A18 pull-ups on SDA/SCL | A18 | bench | meter to 3.3 V, power off | 2–10 kΩ |
| S4-I08 | Axis signs | B4.1.2 | bench | tilt nose up; flat | `ax` rises → `IMU_X_FORWARD_SIGN`; flat `az` ≈ +1000 mg |
| S4-I09 | I2C time on GP14 | B.3 budgets | bench | GP14 around FIFO read | Record µs for 4 samples and magnetometer; ≤ budget |
| S4-I10 | Magnetometer hard-iron offsets | R15 | bench | level, 360 deg slow turn, log mx/my | Offsets into `cfg_imu.h` |
| S4-I11 | Mag vs encoder rate after S4-I10 | A7 | bench | 10 on-the-spot turns | Within 10 % |

*Method:* **auto** = `TEST_CHECK` / `HOST_CHECK` in code, result printed;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | S4-U01, U02, U03, U06, U07, U12, U13, U14 | Phase 2 commit | Faris | PASS (host) | `logs/2026-10-22_host_phase2.txt` | terrain 18/18, fifo planner 6/6; S4-U13 pure parts (mode-driven half needs B5's mission) |
| 2026-10-22 | S4-U13, normal+TEST=1 builds | Phase 2 commit | Faris | PASS (target build) | `logs/2026-10-22_host_phase2.txt` | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.elf` and `_test.elf` build with zero warnings |
| 2026-10-22 | S4-U01, U02, U03, U06, U07, U12, U13, U10 | Phase 4 commit | Faris | PASS (host, packed) | `logs/2026-10-22_host_phase4_packed.txt` | terrain 19/19 (incl. S4-U10 pitch pack), fifo 6/6; packed terrain.c/imu.c build into normal and TEST=1 images with zero warnings |
| 2026-10-22 | S4-U11 | — | Faris | pending | — | hump 127/128 → RANGE_CLAMP lives in Buddy 5's packed status-board writer (and imu.c raises it when the run peak reaches 127); runs once B5 merges |
| 2026-10-22 | RAM report (Phase 5) | Phase 5 commit | Faris | PASS (script) | `docs/ram_comparison.md` | real baseline-vs-packed ELFs from this branch: B4 statics net −63 B; section 3 stays pending until on-car `app_stack_report()` logs exist |
| 2026-10-22 | S4-U01, U02, U03, U06, U07, U12, U13, U10, U14 | audit commit | Faris | PASS (host) | `logs/2026-10-22_host_audit.txt` | terrain 19/19, fifo planner 6/6 after the function-length and naming refactor (behaviour unchanged) |
| 2026-10-22 | S4-U11 | audit commit | Faris | PASS (target build) | `test_imu.c` `test_imu_hump_clamp`, `test_imu_offset_lsb` | 127 mm is representable and does not clamp; 128 mm clamps to 127 and sets the flag `imu_clamp_fault` turns into `APP_FAULT_RANGE_CLAMP`; that bit stays out of `APP_FAULT_STOP_MASK` (A25). One `PRV_OFS_A` AX step decodes to 2 mg, the 12-bit field covers ±4094 mg and 2048 steps clamp. Pure flag checks, so live state is untouched |
| 2026-10-22 | normal + `TEST=1` builds, `barr_check.sh` | audit commit | Faris | PASS | build logs | both images build with zero warnings after the audit; `tools/barr_check.sh`: 67 files, no findings |
| 2026-10-22 | RAM report (Phase 5, regenerated) | audit commit | Faris | PASS (script) | `docs/ram_comparison.md` | re-measured against a fresh pre-packing baseline (`05a20af`): `.bss` −64 B, `.data` +8 B, B4 statics net **−59 B**; section 3 still pending until on-car `app_stack_report()` logs exist |
| 2026-10-22 | S4-U14 + SQ-B4-2 register facts | LSM303D port | Faris | PASS (host) | `logs/2026-10-22_host_lsm303d.txt` | after the LSM303D port: terrain 19/19, fifo planner 8/8 (2 new checks lock address 0x1D and WHO_AM_I 0x49); normal and `TEST=1` images clean, `barr_check.sh` 85 files no findings. Hardware confirmation of the register set is S4-I06 |
| 2026-10-22 | S4-I01, I02, I03, I04, I09 (Phase 3) | — | Faris | pending | — | integration on the car: INT1→motors low ≤15 ms with Buddy 2 (GP14), course humps vs ruler, imu_task WCET. Blocked on hardware; plan below |
| 2026-10-22 | S4-I01–I11 | — | Faris | pending | — | bench/track: needs the physical car (placeholders kept, spec rule A.3) |

## SPEC-QUESTION

**SQ-B4-2: the board is an LSM303D, not the LSM303DLHC — OPEN.**

Reported by Buddy 5, measured on the actual GY-511: the part answers on a
**single address 0x1D** and its **WHO_AM_I (0x0F) = 0x49**. Address 0x19,
which B4.1.1 gives as the accelerometer, does not respond at all. 0x49 is
the LSM303D identity; the DLHC would give 'H','4','3' at 0x0A-0x0C.

Consequence with the unmodified driver: the first write to 0x19 NACKs, the
identity check fails, three errors in a row put the monitor into
TERRAIN_FAULT and raise APP_FAULT_IMU, so the IMU would never work on the
real car. The code was not wrong — B4.1.1 named the wrong part.

Buddy 5's recommendation, which Buddy 4 accepted: **adapt the driver**
rather than buy a discontinued DLHC board. That work is done on branch
`b4-lsm303d` and covers all five points he listed:

| Point | What changed |
|---|---|
| Single address | `DRV_LSM303_ADDR` is 0x1D; the two-address split is gone |
| New registers | LSM303D control map: CTRL0/1/2/3/5/6/7, FIFO_CTRL/FIFO_SRC, IG_CFG1/IG_SRC1/IG_THS1/IG_DUR1 |
| 16-bit scaling | 0.122 mg/LSB at +-4 g, applied as the fraction 61/500 in `imu.c` |
| Different INT1 setup | interrupt generator 1 on INT1 (CTRL3 bit 5), `IMU_IMPACT_THS` into IG_THS1 |
| Other timings | byte counts unchanged (same FIFO status + burst reads), so the 550/750/800 us budget stands; re-measured in S4-I09 |

Two sub-questions the team still has to settle:

1. **IG_THS1 weight.** The register chapter does not publish mg/LSB for
   IG_THS1, so the "1.5 g" in `cfg_imu.h` is nominal. S4-I03 measures it
   and the TUNE marker is honoured there.
2. **E4 row width.** `E_IMU_SAMPLE,16,RAW_SAMPLE,12` documents a 12-bit
   register image, but the LSM303D sample is a true 16-bit value. That row
   is a documentation-only class-E entry (no macros are generated from
   it), so nothing breaks, but `tools/layout.csv` should be corrected to
   16 bits in a `SHARED:` PR once the team agrees.

Unchanged on purpose: `FIFO_SRC` bit layout is identical between the two
parts, so `drv_lsm303_fifo_plan` and its host test needed no change.

**SQ-B4-1: E_IMU_SAMPLE group — RESOLVED, Buddy 1 accepts the split.**
B4.4 lists the three E_IMU rows in one group with mixed `group_bits`
(32, 32, 16). Buddy 1's `gen_layout.py` rejects mixed `group_bits` inside
a group *before* its class-E exemption, so the 16-bit `RAW_SAMPLE` row
sits in its own documentation group `E_IMU_SAMPLE`. No macros are
generated for either group, so nothing functional changes.

Buddy 1 keeps the generator check and accepts `E_IMU_SAMPLE`. Per the
C.6 rule that `tools/layout.csv` changes only through a `SHARED:` pull
request, the layout rows plus the three generated files are reviewed in
branch `shared-b4-layout-rows` (commit `36da4ae`), where the deviation
from B4.4 is written up. That PR is merged, so the rows are on `main` and
`b4-imu` carries them too; no merge conflict results.

## Audit pass (review of Phases 1–5)

Re-read every subsystem 4 file against Part A, Part C and Part D before
handing over. Two **behaviour** defects and several style items were found;
everything below is fixed and re-tested except where marked pending.

| Item | Kind | What was wrong | Fix |
|---|---|---|---|
| `imu_calibrate` | behaviour | The CAL_IMU offsets were summed in raw register units (`g_cal_sum_x += g_avg_raw_x`) but subtracted as mg in the read path, so `ax - ofs_ax` produced values like `1000 − 16000` mg and the store path clamped | Divide the sample sum first, then convert with `IMU_MG_RAW_SHIFT` / `IMU_MG_PER_LSB`; `az` also subtracts the `IMU_REST_AZ_MG` rest reference |
| `imu_calibrate` / `imu_read_acc` | behaviour | The PRV_OFS_A/B words carry **2 mg per step** (`tools/layout.csv`, spec B4.4), but `imu_read_acc` subtracted the decoded step as if it were 1 mg, so every stored offset was applied at half strength | The read path now multiplies by `IMU_OFFSET_LSB_MG`; `test_imu_offset_lsb` locks the 2 mg step and the ±4094 mg range |
| `imu_int1_isr` / impact path | behaviour | `INT1_SRC_A` was never read, so the LSM303DLHC's latched interrupt never re-armed: only the first tap of a run could be seen (breaks S4-I03) | New `drv_lsm303_read_int1_src()` read on the magnetometer cycle; the logged sequence is masked with `IMU_IMPACT_SEQ_MASK` |
| `drv_lsm303` error paths | robustness | An aborted or timed-out transfer left the controller enabled with stale RX data, so the next read could return old samples | New `lsm303_flush()` on every error path (clear abort, toggle `IC_ENABLE`, drain RX) |
| `imu_terrain_update` | correctness | Raised `APP_FAULT_RANGE_CLAMP` at `hump_done >= 127`, which also fired for a legitimate exactly-127 mm hump | Removed; `app_bits_encode`'s clamp flag is the only source (S4-U11 test proves 127 passes, 128 clamps) |
| `imu_task`, `imu_turn_rate`, `terrain_step` | BARR 6.2.a | 152, 107 and 151 lines | Split into `imu_terrain_update`, `imu_encoder_rate`, `imu_mag_rate`, `imu_turn_disagrees`, `terrain_flat_step` / `terrain_climb_step` / `terrain_descend_step`; longest function is now 89 lines |
| `n`, `y`, `x`, `r`, `k`, `f`, `a`, `da`, `id` | BARR 7.1.e | Names shorter than three characters | Renamed (`n_samples`, `y_value`, `id_bytes`, …). **Exception:** `terrain_atan2_ddeg(int32_t y, int32_t x)` keeps the spec's parameter names, because spec line 3291 writes that declaration verbatim and Part A rule 1 fixes the interface |
| `tools/ram_report.sh` | cosmetic | Symbol rows printed as `|sym| … |B4|` (no padding in the markdown table) | `| %s |`; report regenerated |
| `docs/ram_comparison.md` | packaging | The Phase 5 run left the file as UTF-16LE with a BOM, so git stored it as a **binary blob** and the markdown table rendered as garbage on GitHub | Regenerated as plain UTF-8; the same UTF-16 problem was found in `logs/2026-10-22_host_phase2.txt` and `..._phase4_packed.txt` and both were converted too |
| `layout.csv` / generated files | process | The B4 layout rows were committed as `S4:` on `b4-imu`, but C.6 lists `tools/layout.csv` as a shared file that changes only through a `SHARED:` PR | Rows re-submitted for review on branch `shared-b4-layout-rows` (commit `36da4ae`), additive only (+375/−0), with the B4.4 `E_IMU_SAMPLE` deviation written up in the commit/PR body |

## Phase 3 integration plan (pending hardware)

- **INT1 → motors low ≤ 15 ms:** with Buddy 2's motion merged, tap the
  chassis; probe GP14 and the motor pins. The path is
  R1 → `imu_int1_isr` → `safety_set_from_isr(SAFETY_IMPACT)` → motion's
  step 3 brake, inside one 10 ms cycle (B4.2.2 step 3 timing note).
- **Humps on the course vs a ruler:** run S4-I02 profiles; the terrain
  monitor's integrated peak was host-validated at 20 mm ± 5 mm.
- **imu_task WCET:** GP14 around the three paths (2 samples, magnetometer
  cycle, 4-sample backlog) plus the I2C-failure path; record in the WCET
  table for Phase 5.
- The mode-driven half of S4-U13 (mission sets `APP_MODE_RUN_START` for
  30 ms) validates once Buddy 5's mission.c is merged; imu_task's
  run-start handler and the `terrain_ds_counts` rebase guard are already
  in place.

