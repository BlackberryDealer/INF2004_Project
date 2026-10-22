# Team and subsystems — CSC28 PicoCar

The single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.
Every value here comes from the team spec (`CSC28 PicoCar — AI Implementation
Spec`, Part B and Part D); if the two disagree, the spec wins and this file is
fixed by pull request.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.
Shared files (`app_program/app.h`, `app_main.c`, `app_main.h`, `app_types.h`,
`board_pins.*`, `tools/layout.csv`, `config/`, `build_make/`, `lib/`, this
file) change only through a pull request titled `SHARED: …` reviewed by
every affected buddy.

## 1. Members and subsystems

| # | Subsystem (what it does) | Name | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Communications: MQTT telemetry, remote START/STOP/RESET, packing helpers (`app_bits`), layout generator, laptop decoder | Hanzalah | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: closed-loop wheel speed, move/turn, odometry, stall detection, safety interlock (K3), shared fault bitmask (K12) | Dylan | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following, junction classification, Code 39 barcodes | Lucas | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU, terrain (humps, motion class, turn rate), impact interrupt, RAM report | Faris | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Shared services (status board, mission queue, ticks, watchdog, GPIO IRQ), obstacles, mission statechart, HMI | Afif | | | `app_program/subsystem5/` | `testing/subsystem5/` |

Each member fills in their own Student ID and GitHub username.
Integrator (Phase 0 skeleton): Dylan (`phase0-skeleton` pull request,
tag `phase0-pre`).

## 2. Pins

One owner per pin. The pin map is frozen (Week 6 design review); the only file
in the firmware that names a GPIO number is `app_program/board_pins.h`.
`board_pins_check()` prints `PIN CLASH GP<n>` at boot if a pin is listed twice.

| Pin | Macro | Owner (task, buddy) | Used for | Mode |
|---|---|---|---|---|
| GP0 | `PIN_US_TRIG` | obstacle_task, S5 | HC-SR04P trigger (Grove 1) | GPIO out, 10 µs pulse |
| GP1 | `PIN_US_ECHO` | R1 → obstacle_task, S5 | HC-SR04P echo (Grove 1) | GPIO IRQ, both edges |
| GP2 | `PIN_IMU_SDA` | imu_task, S4 | LSM303DLHC SDA | I2C1 |
| GP3 | `PIN_IMU_SCL` | imu_task, S4 | LSM303DLHC SCL | I2C1, 400 kHz |
| GP4 | `PIN_ENC_L_A` | motion_task, S2 | Left encoder A | PIO quadrature (B = A + 1) |
| GP5 | `PIN_ENC_L_B` | motion_task, S2 | Left encoder B | PIO quadrature |
| GP6 | `PIN_LED_ALIVE` | mission_task, S5 | Alive LED (Grove 5) | GPIO out, **SIO only** (shares PWM 3A with the buzzer) |
| GP7 | `PIN_BAR_DO` | R1 → barcode_task, S3 | Barcode IR sensor DO | GPIO IRQ, both edges |
| GP8 | `PIN_MOTOR_L_A` | motion_task, S2 | Left motor M1A | PWM slice 4 A, 10 kHz |
| GP9 | `PIN_MOTOR_L_B` | motion_task, S2 | Left motor M1B | PWM slice 4 B, 10 kHz |
| GP10 | `PIN_MOTOR_R_A` | motion_task, S2 | Right motor M2A | PWM slice 5 A, 10 kHz |
| GP11 | `PIN_MOTOR_R_B` | motion_task, S2 | Right motor M2B | PWM slice 5 B, 10 kHz |
| GP12 | `PIN_SPARE_12` | — | free | — |
| GP13 | `PIN_SPARE_13` | — | free | — |
| GP14 | `PIN_PROBE` | any (via `app_probe_set`) | Scope timing probe | GPIO out, `PROBE=1` builds only |
| GP15 | `PIN_SERVO` | obstacle_task, S5 | SG90 servo | PWM slice 7 B, 50 Hz |
| GP16 | `PIN_ENC_R_A` | motion_task, S2 | Right encoder A | PIO quadrature (B = A + 1) |
| GP17 | `PIN_ENC_R_B` | motion_task, S2 | Right encoder B | PIO quadrature |
| GP18 | `PIN_RGB` | mission_task, S5 | On-board WS2812 RGB LED | PIO |
| GP19 | `PIN_IMU_INT1` | R1 → FLG_SAFETY, S4 callback | LSM303 INT1 (impact) | GPIO IRQ, rising |
| GP20 | `PIN_BTN_RUN` | mission_task, S5 | RUN button | GPIO in, polled every 10 ms |
| GP21 | `PIN_BTN_RESET` | mission_task, S5 | RESET button | GPIO in, polled every 10 ms |
| GP22 | `PIN_BUZZER` | mission_task, S5 | Buzzer | PWM slice 3 A |
| GP23, GP24, GP25, GP29 | — | *reserved* (`cyw43_utk_task`) | Pico W radio | template only |
| GP26 | `PIN_IR_LINE_L` | line_task, S3 | Left line sensor | ADC0 |
| GP27 | `PIN_IR_LINE_R` | line_task, S3 | Right line sensor | ADC1 |
| GP28 | `PIN_SPARE_28` | — | reserved for a 2nd barcode DO | ADC2 capable |

**Never build with `CRASH_REPORT=1` or `CONSOLE=uart`:** they put UART0 on
GP0/GP1, which the ultrasonic owns. `board_pins.c` stops the build with an
`#error` if that is tried.

## 3. Peripherals

**Each peripheral is driven by exactly one task, its owning task.** Every
other task asks the owner through its API (section 5) and never touches the
peripheral itself. Drivers are `drv_<name>.c/.h` in the owner's subsystem
folder, never under `device/`.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART0, UART1 | — (unused; UART0 would take GP0/GP1) | — | — |
| I2C0 | — (unused) | — | — |
| I2C1 | S4 | imu_task (register-level driver `drv_lsm303.c`) | GP2, GP3 |
| SPI0, SPI1 | — (unused) | — | — |
| ADC (channels 0, 1) | S3 | line_task (`drv_ir_adc.c`, SDK `hardware_adc`) | GP26, GP27 |
| PWM slice 3 A | S5 | mission_task (buzzer) | GP22 (GP6 must stay SIO) |
| PWM slices 4, 5 | S2 | motion_task (`drv_motor.c`) | GP8–GP11 |
| PWM slice 7 B | S5 | obstacle_task (`drv_servo.c`) | GP15 |
| PIO: quadrature encoder program (K9) | S2 | motion_task (`drv_encoder.c`) | GP4/5, GP16/17 |
| PIO: WS2812 program | S5 | mission_task (`drv_ws2812.c`) | GP18 |
| GPIO bank-0 interrupt (R1) | S5 | `drv_gpio_irq.c` dispatches to S3, S4, S5 callbacks | GP1, GP7, GP19 |
| Timer (1 MHz time base, read only) | S5 | `sys_tick.c` (`sys_tick_now_us()` for everyone) | — |
| Watchdog | S5 | mission_task via `sys_health.c` (registers directly) | — |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (template; S1 code in `mqtt_utk.c`) | GP23–25, GP29 |

Kernel drivers switched **off** in `config/config_device.h`: ADC
(`DEVCNF_USE_ADC 0`, line_task owns the ADC) and I2C (`DEVCNF_USE_IIC 0`, it
would claim GP6–GP9). `board_pins.c` enforces both at compile time.

No application task calls `cyw43_*`, lwIP or MQTT functions. The radio task
calls only the functions in `subsystem1/mqtt_bridge.h`.

## 4. Task priorities

1 is the highest. The kernel uses 3 (USB console) and 4 (WiFi). The build is
single core (`make SMP=0 NET=lwip MQTT=1 -j8`). Every task stack is a static
`uint64_t g_<task>_stack[]` (visible in the map file) and includes the
kernel's 128-byte system part.

| ID | Task | Subsystem | Priority (macro) | Trigger | Stack bytes | WCET budget |
|---|---|---|---|---|---|---|
| — | USB console | kernel | 3 | — | — | — |
| T8 | `cyw43_utk_task` | kernel (+S1 code) | 4 | 2 ms delay loop | template | measure (A16) |
| T1 | `line_task` | S3 | 12 (`APP_PRI_LINE`) | R2, 10 ms | 1152 | 300 µs |
| T2 | `motion_task` | S2 | 13 (`APP_PRI_MOTION`) | R2, 10 ms | 1152 | 300 µs |
| T3 | `imu_task` | S4 | 14 (`APP_PRI_IMU`) | R2, 10 ms | 1152 | 550 µs (750 magnetometer, 800 backlog) |
| T4 | `obstacle_task` | S5 | 15 (`APP_PRI_OBSTACLE`) | R3, 60 ms | 1152 | 200 µs |
| T5 | `mission_task` | S5 | 16 (`APP_PRI_MISSION`) | K4 message or 10 ms timeout | 2176 | 200 µs |
| T6 | `barcode_task` | S3 | 17 (`APP_PRI_BARCODE`) | SEM_BAR or 10 ms timeout | 1152 | 500 µs per decode |
| T7 | `telem_task` | S1 | 20 (`APP_PRI_TELEM`) | K7 message or 200 ms timeout | 2176 | 500 µs |
| — | `usermain` (TEST=1 suites) | team | 21 (`APP_PRI_TEST`) | after boot | kernel initial task | — |

Handlers: **R1** `gpio_bank0_isr` (S5, interrupt level 2) serves GP1, GP7 and
GP19 through registered callbacks. **R2** 10 ms cyclic handler wakes T1–T3.
**R3** 60 ms cyclic handler wakes T4. R2 and R3 live in `sys_tick.c` (S5).

## 5. Interfaces between subsystems

Every kernel object has exactly **one creator**; everyone else uses only its
access API (the headers in Part D of the spec, already in the repo).

| ID | From → To | Mechanism | Object (creator file) | Access API | Data / meaning |
|---|---|---|---|---|---|
| K1 | R1 → S5 obstacle_task | semaphore | `SEM_ECHO` (`obstacle.c`) | internal, R1 callback `obstacle_echo_isr` | echo edge captured |
| K2 | R1 → S3 barcode_task | semaphore + 64-entry edge ring | `SEM_BAR` (`barcode.c`) | internal, R1 callback `barcode_edge_isr` | 16-bit edge words (`MSG_EDGE`) |
| K3 | S4 (R1), S5 → S2 motion_task | event flag | `FLG_SAFETY` (`safety.c`) | `safety_*()` | `SAFETY_ESTOP / IMPACT / FAULT`: motors brake while any bit is set |
| K4 | all, radio task → S5 mission_task | message buffer | `MBF_MISSION_EVT` (`mission.c`) | `mission_post_event()` | 16-bit event word (`MSG_EVT`, Part D.13) |
| K5 | S5 mission_task → S2 motion_task | message buffer | `MBF_MOTION_CMD` (`motion.c`) | `motion_*()` commands | 16-bit command word (`MSG_CMD`) |
| K6 | each task → all | shared data + mutex (TA_INHERIT) | status board + `MTX_STATUS` (`status_board.c`) | `status_board_*()` | one writer per part; readers copy a snapshot |
| K7 | S5 mission_task → S1 telem_task | message buffer | `MBF_TELEM_EVT` (`telem.c`) | `telem_post_*()` | events, profile, summary |
| K8 | T1–T4 → S5 mission_task | event flag | `FLG_ALIVE` (`sys_health.c`) | `sys_health_*()` | per-task check-in bits; watchdog feed |
| K9 | PIO → S2 motion_task | PIO RX FIFOs (hardware) | encoder SMs (`drv_encoder.c`) | `drv_encoder_read()` | quadrature counts |
| K10 | S5 mission_task → all | event flag | `FLG_MODE` (`mission.c`) | `mission_mode_*()` | `APP_MODE_*` requests |
| K11 | S1 telem_task → radio task | message buffer | `MBF_TELEM` (`telem.c`) | `mqtt_bridge_take()` | ready-to-publish payloads |
| K12 | all → S5 mission, S1 | event flag | `FLG_FAULT` (`fault.c`) | `fault_*()` | shared fault bitmask `APP_FAULT_*` (10 bits) |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. Planned use when every subsystem is
complete (the Phase 0 skeleton creates only the seven tasks):

| Object | Limit | S1 | S2 | S3 | S4 | S5 | Total |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 (telem) | 1 (motion) | 2 (line, barcode) | 1 (imu) | 2 (obstacle, mission) | 7 |
| Semaphore | 16 | | | 1 (K2) | | 1 (K1) | 2 |
| Event flag | 16 | | 2 (K3, K12) | | | 2 (K8, K10) | 4 |
| Mailbox | 12 | | | | | | 0 |
| Mutex | 12 | | | | | 1 (K6) | 1 |
| Message buffer | 12 | 2 (K7, K11) | 1 (K5) | | | 1 (K4) | 4 |
| Fixed memory pool | 12 | | | | | | 0 |
| Variable memory pool | 12 | | | | | | 0 |
| Cyclic handler | 12 | | | | | 2 (R2, R3) | 2 |
| Alarm handler | 12 | | | | | | 0 |
| Interrupt handler (`tk_def_int`) | — | | | | | 1 (R1) | 1 |

The kernel's own USB console and WiFi service tasks use a few more of these;
every application limit above still has ample headroom. Task stacks and
message-buffer storage are static arrays (`TA_USERBUF`), so the kernel heap
is not used by the application.
