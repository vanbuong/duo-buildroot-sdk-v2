# 05 – Test plan, unit tests and CI

What is tested, at which level, with which pass criteria, and how it runs automatically.

Legend: **[now]** implemented and passing in this repository, **[plan]** to implement, **[est]** threshold
to confirm with the first hardware data.

---

## 1. Strategy

A balance robot cannot be validated on the floor alone: failures there are fast, expensive and hard to
reproduce. The plan is a pyramid, pushing as much as possible down to levels that run in seconds on a
laptop or in CI, and keeping hardware time for what only hardware can show.

```
                     L6  Field / soak            (FLD-*)   slow, real battery, real floor
                    L5  System: video+Wi-Fi+RTOS (VID-*, NET-*, PERF-*, SAF-*)
                  L4  Hardware-in-the-loop bench  (HIL-*, IPC-*)    robot on a stand
                L3  Closed-loop simulation        (test_sim.c)     [now]  seconds, CI
              L2  Host unit tests of pure logic   (test_*.c)       [now]  milliseconds, CI
            L1  Static checks: -Wall -Wextra -Werror, sanitizers, coverage      [now]
          L0  Review + design rules (doc 01 §1: pure logic has no MMIO)
```

| Level | Runs where | Trigger | Time | Gate |
|-------|------------|---------|------|------|
| L0–L3 | GitHub runner, laptop | every push/PR touching the balance code or docs | ≈ 1 min | **blocking** |
| Firmware cross-build | GitHub runner with SDK toolchain | PR, nightly | 15–40 min | blocking once added |
| L4 bench | self-hosted runner + DuoS on a bench | nightly, release candidates | ≈ 30 min | blocking for release |
| L5 system | bench + Wi-Fi AP + client | release candidates | ≈ 2 h | blocking for release |
| L6 field | humans | before tagging a release | ≈ ½ day | blocking for release |

## 2. Testability rules (design constraints)

1. **Pure logic never touches MMIO.** `pid.c`, `encoder.c` decode, estimator math, mixers, state machine, command
   shaping and the shared-memory protocol are written against small interfaces (function pointers or link-time
   stubs) so they compile on the host.
2. **Link-time seams, not `#ifdef`s.** The host build links `test/stubs/fake_hw.c` (GPIO + I2C) instead of the
   real `gpio.c`/`poll_i2c.c`; the production code has no test hooks. The stub headers `printf.h`/`delay.h`
   shadow the SDK ones through `-Istubs`.
3. **Time is injected.** Functions take `dt` (or a timestamp); no function reads a clock internally except the
   thin `timer_us()` wrapper.
4. **Each safety rule has a test that makes it fire.** A safety feature without a test that triggers it is
   considered unimplemented.
5. **Counter-examples are kept.** Tests such as `sim_rationale_low_alpha_without_gate_falls`,
   `sim_wrong_encoder_sign_runs_away_or_falls` and `sim_speed_gain_magnitude_limit` assert that a *wrong*
   configuration fails, so the design rationale in docs 03/04 stays executable. (The `*_KNOWN_DEFECT_*` tests that
   pinned the first firmware version were removed when its defects were fixed.)

## 3. L1–L3 – host tests **[now]**

Location: `freertos/cvitek/task/balance_car/test/` (C) and `.../linux_tests/` (Python).

```
test/
├── Makefile              make test | make test SAN=1 | make coverage | make tools | make pytest
├── unity_lite.h          CHECK / CHECK_NEAR / CHECK_EQ, RUN()
├── stubs/                printf.h delay.h  fake_hw.{h,c}   (GPIO levels, I2C register file, read hook, fault injection)
├── sim_plant.{h,c}       wheeled inverted pendulum + motor deadband/lag (ASSUMED parameters)
├── test_pid.c test_encoder.c test_mpu.c test_params.c test_estimator.c test_control.c
├── test_state.c test_health_cmd.c test_shm.c test_sim.c test_main.c
├── layout_dump.c         prints the C compiler's view of every shared offset (for the Python cross-check)
└── shm_tool.c            plays the RTOS side of the shared window on a file, using the real bc_shm.h
linux_tests/              test_layout.py test_shm_interop.py test_bcd.py test_camera.py test_bcctl_scripts.py
tools/                    gen_bc_layout.py  rtos_build_check.sh  dts_check.sh
```

Run: `make -C freertos/cvitek/task/balance_car/test test` (needs only a C compiler and libm/pthread) and `… pytest`.
Current result: **154 C tests / 745 checks and 86 Python tests, all passing**; clean under AddressSanitizer + UBSan (gcc);
line coverage `pid` 92 %, `encoder` 100 %, `mpu60x0` 94 %, `bc_params` 96 %, `estimator` 99 %, `control` 99 %, `bc_state` 99 %,
`imu_health` 100 %, `bc_cmd` 100 %, `bc_core` 90 %, `i2c_recover` 100 %, `dmp612` 91 %.

### 3.1 C inventory (154 tests)

| Suite | Tests | What it proves | Plan IDs |
|-------|-------|----------------|----------|
| `test_pid.c` (10) | proportional, clamp, integral + anti-windup, derivative, **setpoint kick (documents the generic form)**, reset, dt guard, **rate-form (no kick)**, **rate-form P/clamp**, **I-limit in output units + freeze** | PID mathematics | `UT-PID-01..07`, `CTL-01`, `CTL-03` |
| `test_encoder.c` (6) | 4× decode, signed counting, no spurious counts, bounce, **skipped state dropped (the under-sampling failure mode)**, reset | quadrature decode | `UT-ENC-01..06` |
| `test_mpu.c` (19) | **variant table and every 6500-family id, reset sequence, read-back mismatch → -2, clone ignoring `ACCEL_CONFIG2`, unknown id, bus error on `WHO_AM_I`, temperature per variant**, 6050/6500 register setup (±500 °/s, 1 kHz, `ACCEL_CONFIG2`), scale constants tied to the registers, big-endian decode, bus error, scaling + bias, 3-axis calibration, **motion rejected, bad gravity rejected**, calibration bus error | IMU driver | `UT-MPU-*` |
| `test_params.c` (7) | defaults valid, **every field rejects out-of-range/NaN**, set-by-name, sign fields ±1 only, first-bad-field report, dense 32-bit layout, CRC-32 vector | parameter set | `SHM-03` (C side) |
| `test_estimator.c` (16) | **spike filter (reject/count/escape/off), Kalman (convergence, bias, gate, covariance over 10⁶ steps, agreement with the complementary filter)**, init from accel, convergence, gyro integration, **accel gate**, zero vector finite, **bias tracking**, bias frozen while gated, sign flip | estimator | `EST-01..04` |
| `test_control.c` (13) | counts→m/s, slew, deadband table, zero drive upright, **angle PD formula + sign**, trim, saturation, **speed loop leans back when moving forward**, encoder sign, target clamp/slew, turn, **mixer keeps balance drive over turn**, reset | controller | `CTL-01..06` |
| `test_state.c` (21) | calibration/arming flow, 1 s upright window and restart, timeout, refusal with IMU fault, fall, fallen recovery, each IMU fault, fault needs clean sensor, saturation (long vs short), lift (and *not* lift while leaning), timing fault, **ESTOP latch**, disarm, motors only in BALANCING, names | state machine | `ST-01`, `ST-02`, `SAF-01/04/05/07` (logic) |
| `test_health_cmd.c` (11) | stuck frame, gyro saturation, norm window, bus-error policy, target pack/unpack, state pack, clamp, **watchdog zeroes targets (never disarms)**, stale target, hb-disarm + wraparound | `imu_health`, `bc_cmd`, `bc_proto` | `FLT-01..04`, `CMD-01/02` |
| `test_shm.c` (11) | header written last, layout offsets + cache-line separation, telemetry round trip/order, **wrap → lost**, corruption/torn detection, events, parameter block round trip + rejection, echo, **two-thread seqlock stress (300 000 records, 0 torn deliveries)**, parameter pair coupling stress | shared memory | `SHM-01..03` |
| `test_sim.c` (10) | start-up sequence + 1 s arming, **balances for plant gain 0.4×…4×**, sensor noise, push recovery, 12° start, speed gain ≤ 4 °/(m/s) stable, **magnitude limit**, **wrong encoder sign diverges**, **α 0.98 without gate falls**, **IMU unplug → coast within 4 cycles, motor 0** | closed loop through the real `bc_core` and MPU driver | `SIM-01..`, `HIL-06` (sim) |

How the simulation is wired: the test loop mirrors `balance_ctrl_task()` (200 Hz). It writes the plant's accelerometer and
gyro values into the **fake I2C register file** (the accelerometer includes the wheel acceleration), calls the *real*
`mpu60x0_read()`/`mpu60x0_scale()`, feeds raw encoder counts, and runs the *real* `bc_core_step()` – including health
monitor, estimator, state machine and controller. The "hand holds the car upright while arming, then lets go at a
lean" start is modelled explicitly.

Limits of the simulation (be honest about them): the plant parameters are assumptions, there is no motor back-EMF
model, no wheel slip, no I2C/printf timing, and the pitch dynamics are linear-plus-`sin`. Passing SIM tests means "the
structure and signs are right and there is margin in the *model*", not "the car will balance". Hence the
system-identification step in doc 04 §12 and the HIL/field levels below.

### 3.2 Python inventory (57 tests)

| File | Tests | What it proves |
|------|-------|----------------|
| `test_layout.py` (6) | window offsets, record sizes/field offsets, status fields, **parameter table (names, types, ranges, offsets) identical to C**, parameter blocks, protocol ids | generated `bc_layout.py` matches the C compiler's view |
| `test_shm_interop.py` (10) | header/telemetry/events **written by the real C code, read by Python**; CRC validation and corruption; ring overrun; parameters **written by Python, validated and echoed by the real C code**; corrupt block rejected; Python validation = C ranges; heartbeat visible to C; mailbox packing/ioctl number | cross-language compatibility |
| `test_bcd.py` (21) | heartbeat ticks, clamp + dead-man, commands, ESTOP ordering, **parameter push confirmed by the real RTOS C code**, invalid values never sent, timeout, save/load, **emergency when the RTOS heartbeat freezes (fires once, re-arms)**, telemetry JSON, event JSON, message handling, **real WebSocket round trip (RFC 6455 accept example, ping/pong, bad JSON)**, client disconnect → dead-man, HTTP endpoints, framing edge cases | `bcd` |
| `test_camera.py` (11) | selection table, J1/J2/both/none/forced, **IMX219 reported as unsupported**, missing ini, dry run, sysfs GPIO writes, ini names exist in the SDK overlay | `bc_camera` |
| `test_dmp.c` (17) | **experimental DMP**: image write + read-back verify, wrong size, memory that does not stick, packet decode/quaternion check, gravity→pitch sign, FIFO partial/bad/overflow/lag/bus error, gyro scale switch, shm image block round trip + corruption, estimator mode 2 (agree/disagree/missing/drift), core integration | `est_mode 2` |
| `test_dmp_linux.py` (14) | image extraction ignoring comments, checksum guard, Python↔C image block (both directions), corruption, bcd offers the image once per RTOS boot, `bcctl dmp-pack/dmp-load/dmp-status` | DMP hand-over |
| `test_i2c_recover.c` (4) | free bus → STOP only; slave releasing after n clocks; give up after 9 clocks (bounded time); null counter | I2C bus recovery logic |
| `test_log_video.py` (13) | JSONL logger rotation/size bound/never raises (found a real bug: telemetry `pad` bytes were not serialisable), video profile save/load/env, adaptive-rate hysteresis and manual override, bcd `video` message/broadcast, odometry in telemetry JSON, calibration block read from the C writer | logging + video profiles |
| `test_bcctl_scripts.py` (9) | `bcctl` status/events/get/set rejection; shell scripts parse; Python modules compile; **`duo-init.sh` does not load the PWM module and starts the services; DTS release `&i2c4`/`&spi3`**; web UI uses the protocol | CLI + integration files |

Quality gates: **line coverage ≥ 85 % per file** (`make coverage`); every `FAULT_*` has a test (`st_names_exist_for_every_value`
plus the individual cases above).

Still to add: a unit test per new `FAULT_*` when one is introduced. Spike filter and bus-recovery logic are now covered; the bus recovery on real pads is a hardware test (`HIL-06`).

## 4. L4 – hardware-in-the-loop (bench) **[plan]**

Fixture: DuoS + IMU + TB6612 + motors on a stand (wheels free), bench supply 12 V with current limit
(1.5 A), logic analyser/scope on PWM, STBY, encoder lines, and a spare GPIO toggled by the control loop;
USB-serial to the RTOS console and SSH/USB-NCM to Linux.

| ID | Test | Procedure | Pass criterion |
|----|------|-----------|----------------|
| HIL-01 | IMU bring-up | boot; read 1000 frames | WHO_AM_I OK; accel norm 1 ± 0.05 g still; gyro noise < 0.5 °/s RMS; 0 bus errors |
| HIL-02 | Encoder edge rate | signal generator / second MCU emulates A/B at 1, 5, 10 kHz edge rate both directions | zero lost counts over 60 s (final count = expected exactly); direction correct |
| HIL-03 | PWM waveform | scope on PWMA/PWMB for duty 0, 1, 10, 50, 90, 100 % | frequency 20 kHz ± 1 %, duty error < 1 % abs, no glitch when duty is rewritten at 200 Hz (**checks the `hw_pwm_enable` restart quirk**) |
| HIL-04 | TB6612 logic | all (IN1, IN2, STBY) combinations from the debug shell | outputs match the truth table; STBY=0 gives high-Z; boot state has motors off |
| HIL-05 | Estimator on a rotary jig | IMU on a servo/turntable swinging ±20° at 0.2–2 Hz, plus a vibration source; compare with encoder-measured angle | RMS error < 1° (< 2° with vibration); lag < 20 ms; gate rejects shaken data without drift > 1°/min |
| HIL-06 | I2C fault injection | pull SDA low / disconnect IMU / power-cycle IMU while armed (wheels free) | motors coast within 20 ms (4 cycles); fault code `IMU_BUS`; bus recovery attempted; clean re-arm after reconnect |
| HIL-07 | Control timing | scope the spare GPIO (high during `bc_ctrl_step`), log `period_us`/`exec_us` for 10 min | period 5.000 ms ± 0.1 ms p99, ± 0.3 ms max; exec < 1.0 ms max; 0 deadline misses |
| HIL-08 | Sign bring-up | doc 04 §7 procedure, recorded | all four checks pass; signs committed in `board_pins.h` |
| HIL-09 | Actuator identification | step response, doc 04 §12 | `b`, deadband, motor lag recorded in `results/`; `sim_plant.c` updated |
| IPC-01 | Mailbox ping RTT | `BC_CMD_PING` ×10 000 | 0 lost; RTT p99 < 1 ms |
| IPC-02 | Command latency | RTOS timestamps `SET_TARGET` arrival vs Linux send time | p99 < 1 ms (**[est]**) |
| IPC-03 | Shared-memory stress | RTOS writes telemetry at 50 Hz while Linux reads at 1 kHz and writes params at 10 Hz for 30 min with video running | zero torn records, zero rejected-valid params, `lost` = 0 |
| IPC-04 | Parameter validation | bad CRC/NaN/out-of-range via `bcd` | rejected, `PARAM` event, old values intact |
| IPC-05 | Heartbeat loss | `kill -STOP bcd` during stand test | targets → 0 within 500 ms; robot keeps balancing; recovery on `kill -CONT` |
| IPC-06 | Mailbox flood | 1000 commands back-to-back | no RTOS crash/hang; ESTOP still processed; `bcd` handles "no slot" errors |
| IPC-07 | RTOS reload | `rproc-start.sh` stop/start ×50 while DISARMED | always comes back; Linux and RTOS agree on state; no stale shm accepted (magic/version/boot_count) |

## 5. L5 – system tests **[plan]**

| ID | Test | Procedure | Pass criterion |
|----|------|-----------|----------------|
| VID-01 | Pipeline soak | `fpv-med` for 30 min | no crash/leak; fps ≥ 24; RSS growth < 5 MB |
| VID-05 | Camera J1 (GC2083) | forced `j1`, `fpv-med` 10 min; also with IMU running on its (new) bus | stream OK; IMU 0 bus errors; `bc_camera` reports `j1` |
| VID-06 | Camera J2 (OV5647, 15-pin) | forced `j2`, same | stream OK **and** IMU on I2C0 unaffected (guards the `MIPIRX4` conflict) |
| VID-07 | Auto-select and switching | each camera alone, both fitted (prefers J1), none fitted; switch J1↔J2 ×10 via `bcctl camera` | correct selection; unused sensor held in reset; balance loop unaffected (`period_us`); no module reload needed (or documented) |
| VID-02 | Glass-to-glass latency | film a running clock beside the client screen | `fpv-low` ≤ 250 ms median (stretch 150 ms) |
| VID-03 | Profile switch | switch low↔med↔hq 20× | each < 3 s to first IDR; no pipeline restart of `bcd` |
| VID-04 | Adaptive rate | `tc netem loss 3 % delay 40 ms jitter 20 ms` | automatic downshift within 5 s; recovery after link restored |
| NET-01 | Throughput | `iperf3` TCP/UDP both directions, 5 GHz and 2.4 GHz | numbers recorded; ≥ 3× the video bitrate headroom |
| NET-02 | Control RTT | `ping` field round trip through `bcd` → RTOS → telemetry | p95 < 60 ms on good link |
| NET-03 | Degraded link | netem 10 % loss | control remains usable (dead-man not falsely triggering); video degrades gracefully |
| NET-04 | Link drop | disconnect client for 10 s while balancing and driving | robot stops and stands still ≤ 500 ms, does not fall; resumes when client returns |
| NET-05 | AP/STA fallback | boot with/without the configured AP | falls back to own AP within 20 s |
| PERF-01 | RTOS CPU load | telemetry `cpu_load_pct` over 30 min | < 25 % |
| PERF-02 | Stack margin | `stack_min` | every task ≥ 25 % free |
| PERF-03 | **Jitter under load** | HIL-07 while `bc-video` at 720p25 + iperf3 at full rate + `bcd` logging | period 5 ms ± 0.1 ms p99, 0 deadline misses in 30 min |
| PERF-04 | A53 load | `top` | `bc-video` + `bcd` < 60 % of one core; no thermal throttling |
| SAF-01 | Fall detection | tip the car by hand past 45° | coast + `STBY=0` ≤ 20 ms; state `FALLEN` |
| SAF-02 | IMU unplug | same as HIL-06 on the ground (padded) | coast, fault, no runaway |
| SAF-03 | `bcd` crash | `kill -9 bcd` | RTOS zeroes targets, keeps balancing |
| SAF-04 | Lift detection | pick the car up while balancing | wheels coast ≤ 200 ms |
| SAF-05 | Saturation fault | add 300 g to one side or weaken the battery | fault after 300 ms of saturation, no oscillating runaway |
| SAF-06 | Low battery | supply sweep 12 → 9 V | warning at threshold, coast at cut-off, no reset loop |
| SAF-07 | ESTOP | command and hardware button | coast ≤ 20 ms; latches until explicit DISARM |
| SAF-08 | **RTOS crash / halt** | `echo stop > /sys/class/remoteproc/remoteproc0/state` (or hang the RTOS) **while driving** | motors off within 300 ms through the Linux emergency path / hardware watchdog (doc 01 §7.3); the PWM block and GPIO latches are expected to **keep their last state when the core stops** (peripheral state is independent of the CPU; confirm in this test), so it must pass before any floor test with Linux connected |

## 6. L6 – field tests **[plan]**

| ID | Test | Pass criterion |
|----|------|----------------|
| FLD-01 | Tethered stand balancing, gains per doc 04 §8, small pushes | reaches steady state; **measured behaviour compared with the simulation** (settling time, oscillation frequency within 30 %); if not, update the plant (HIL-09) |
| FLD-02 | Free standing, level floor, 5 min | max lean < 3°, drift < 0.5 m, no faults |
| FLD-03 | Drive / turn | follows ±0.3 m/s and ±1 rad/s within 10 %; stops without falling |
| FLD-04 | Push recovery | recovers from a firm hand push (≈ 15° transient lean) |
| FLD-05 | Surface variety | carpet, tile, 5° ramp: no fall |
| FLD-06 | Battery sweep | stable from 100 % to 30 % charge without retuning (or battery scaling enabled) |
| FLD-07 | Payload | ± 100 g, ± 2 cm CoM shift handled by trim/speed integral |
| FLD-08 | Soak | 30 min continuous driving with video: no faults, temperature of driver/motors/DuoS within limits |

## 7. Requirements traceability

| Requirement | Verified by |
|-------------|-------------|
| R-01 Stay upright with Linux hung / Wi-Fi lost | IPC-05, SAF-03, NET-04, FLD-02 |
| R-02 Loop 200 Hz, jitter ≤ 0.1 ms p99 | HIL-07, PERF-03 |
| R-03 No encoder count loss to the maximum motor speed | UT-ENC-05, HIL-02 |
| R-04 Estimator error < 1° static, tolerant to wheel acceleration | UT-MPU-04/05, EST-01..04, SIM-01..04, HIL-05 |
| R-05 Motors never run on stale/faulty sensor data | FLT-*, HIL-06, SAF-02 |
| R-06 Safe state on every fault, latched where specified | ST-01, SAF-* |
| R-07 Wrong wiring/signs are detected before floor tests | SIM-05, SIM-07, HIL-08 |
| R-08 Video does not affect control | PERF-03 |
| R-09 Parameters changeable at run time and validated | SHM-03, IPC-04, BCD-01 |
| R-10 Video latency ≤ 250 ms | VID-02 |
| R-11 Control via Wi-Fi with dead-man | CMD-02, NET-02/04 |

## 8. CI

### 8.1 Existing workflows (unchanged)

| Workflow | Purpose |
|----------|---------|
| `.github/workflows/ci.yml` (`duo-ci`) | full SDK image builds for all board/toolchain/flash combinations (long, downloads `dl.tar`) |
| `.github/workflows/arduino-sd.yml` | SD images with burnd for ARM and RISC-V boards |
| `.github/workflows/release.yml` | release packaging |

### 8.2 Workflow `.github/workflows/balance-car-tests.yml` **[now]**

Triggers: push/PR touching the RTOS tasks, the `rtos_cmdqu` headers, the linker script/kernel config, the DuoS overlay
and DTS, `docs/balance_car/**` or the workflow; manual dispatch.

| Job | What | Gate |
|-----|------|------|
| `unit` matrix: gcc, gcc + ASan/UBSan, clang | `make test` (`-std=c99 -Wall -Wextra -Werror`) – 154 tests | any failure or warning |
| `coverage` | `make coverage` – twelve source files ≥ 85 % lines; uploads `.gcov` | below threshold |
| `pytest` | `make pytest` (builds `layout_dump`/`shm_tool`, runs 86 tests) and `gen_bc_layout.py --check` | any failure, or generated layout out of date |
| `fw-build` matrix: `cv181x` DuoS, `cv180x` Duo | `tools/rtos_build_check.sh`: **real RTOS image build** with the Ubuntu RISC-V GCC + picolibc (T-Head CSR names patched in a scratch copy), fails on any warning in `task/balance_car`, checks `_bc_shm_base`/`prvBalanceCommTask` are linked for cv181x, enforces a **firmware size budget** (footprint up to `_end` ≤ 2 MiB − 64 KiB window − 256 KiB headroom; currently ≈ 0.8 MB), and that `balance_car_start` is **not** linked for cv180x | build failure, warning, missing symbol |
| `dts` | `tools/dts_check.sh`: cpp + `dtc` on the four DuoS device trees | any parse error |
| `cppcheck` (report only, `continue-on-error`) | static analysis of the pure-logic modules; becomes a gate once the first run is clean | – |
| `docs` | `docs/balance_car/check_links.py` | broken link |

All of these were run locally (gcc, gcc+ASan/UBSan, clang, coverage, both RTOS builds, all four DTS, pytest). The workflow ran on
GitHub for the previous head and was green (31 checks including the SDK's own full image builds); the jobs added in the latest
change (`cppcheck`, size budget, new tests) are verified by their first run. clang with sanitizers is not in the matrix because
the authoring sandbox lacks its runtime.

What `fw-build` does *not* prove: it uses a different toolchain/libc than the SDK's (`milkv-duo/host-tools`), so the
SDK's own `ci.yml` builds remain the authoritative full-image check.

### 8.3 Still planned

| Job | Description | Notes |
|-----|-------------|-------|
| `fw-size` | fail if the image grows beyond a budget | the linker `ASSERT` already protects the shm window; the current image is 99 KB text |
| `static` | `cppcheck`/`clang-tidy` on the pure-logic files | report-only first |
| `hil-nightly` | self-hosted runner next to a DuoS bench: build → `/lib/firmware` → `rproc-start.sh` → HIL/IPC tests, KPI regression check | needs the fixture (relay, USB-serial, encoder emulator, IMU jig) |
| `release-gate` | tag build requires the above plus signed-off field checklist | |

### 8.4 Branch and review policy

* PRs touching `balance_car/` need the `balance-car-tests` checks green.
* Any change to a gain default, filter constant, fault threshold or state-machine rule must update the
  matching test **and** the parameter table in doc 04 §10 in the same PR.
* A change that makes a counter-example test pass (see §2 rule 5) must say why in the PR; findings are tracked as C1…C10 (doc 04 §9) and S1…S18 (doc 06).

## 9. Test data and reporting

* Telemetry CSV schema = `bc_telem_t` ([01 §6.2](01-system-architecture.md)); every HIL/FLD run stores the CSV
  plus a short `results/<date>-<test-id>.md` (setup, firmware build id, result, plots).
* A small Python script (`tools/bc_report.py` **[plan]**) renders θ(t), motor %, speed, jitter histogram and
  checks the KPIs of §4–§6 so pass/fail is computed, not eyeballed.
* The firmware build id (git SHA) is in `bc_shm` header and in each CSV, making every result traceable.

## 10. How to run everything that exists today

```sh
T=freertos/cvitek/task/balance_car
make -C $T/test test                  # 154 C tests (unit + closed-loop simulation)
make -C $T/test clean test SAN=1      # same under AddressSanitizer + UBSan
make -C $T/test clean coverage        # per-file line coverage gate
make -C $T/test pytest                # 86 Python tests (builds the C helper tools first)
python3 $T/tools/gen_bc_layout.py --check
$T/tools/dts_check.sh                 # four DuoS device trees (needs device-tree-compiler)
$T/tools/rtos_build_check.sh          # RTOS image, cv181x (needs gcc-riscv64-unknown-elf + picolibc)
$T/tools/rtos_build_check.sh cv1800b_milkv_duo_musl_riscv64_sd cv180x
python3 docs/balance_car/check_links.py
```
