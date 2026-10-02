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
5. **Known defects are pinned, not hidden.** Tests named `*_KNOWN_DEFECT_*` assert the current (wrong)
   behaviour and carry a comment pointing to the fix; fixing the code intentionally breaks them and the
   author flips/deletes them in the same commit.

## 3. L1–L3 – host tests **[now]**

Location: `freertos/cvitek/task/balance_car/test/`

```
test/
├── Makefile              make test | make test SAN=1 | make coverage
├── unity_lite.h          CHECK / CHECK_NEAR / CHECK_EQ, RUN()
├── stubs/                printf.h delay.h  fake_hw.{h,c}   (GPIO levels, I2C register file, fault injection)
├── sim_plant.{h,c}       wheeled inverted pendulum + motor deadband/lag (ASSUMED parameters)
├── test_pid.c  test_encoder.c  test_mpu.c  test_sim.c  test_main.c
```

Run: `make -C freertos/cvitek/task/balance_car/test test` (no dependencies beyond a C compiler and libm).
Current result: **30 tests, 78 checks, 0 failures**; also clean under AddressSanitizer + UBSan (gcc);
coverage `pid.c` 100 %, `encoder.c` 100 %, `mpu60x0.c` ≈ 88 % (uncovered: the unknown-WHO_AM_I warning path).

### 3.1 Inventory

| ID | Test | What it proves |
|----|------|----------------|
| UT-PID-01 | `pid_p_only_is_proportional` | `out = Kp·(sp − meas)` |
| UT-PID-02 | `pid_output_is_clamped` | asymmetric limits respected |
| UT-PID-03 | `pid_integral_accumulates_and_is_clamped` | `∫e dt` rate, saturation at `i_max`, recovers after sign reversal (anti-windup bound) |
| UT-PID-04 | `pid_derivative_acts_on_error_change` | D term magnitude and decay |
| UT-PID-05 | `pid_setpoint_step_causes_derivative_kick` | **pins defect C5** (derivative on error) |
| UT-PID-06 | `pid_reset_clears_state` | integrator and previous error reset |
| UT-PID-07 | `pid_nonpositive_dt_is_safe` | no NaN/inf for `dt ≤ 0` |
| UT-ENC-01 | `encoder_counts_4x_per_cycle_forward` | 4 counts per quadrature cycle, sign convention |
| UT-ENC-02 | `encoder_direction_is_signed` | up/down counting is symmetric |
| UT-ENC-03 | `encoder_no_motion_no_counts` | no spurious counts over 1000 polls |
| UT-ENC-04 | `encoder_jitter_is_cancelled` | contact bounce nets the right count |
| UT-ENC-05 | `encoder_skipped_state_is_dropped` | illegal transition counts 0 (**documents the under-sampling failure mode**, doc 03 §4.3) |
| UT-ENC-06 | `encoder_reset_zeroes_count` | reset |
| UT-MPU-01 | `mpu_init_accepts_known_who_am_i` | register writes: `PWR_MGMT_1=1`, `CONFIG=3`, gyro/accel ±250 dps/±2 g |
| UT-MPU-02 | `mpu_read_decodes_big_endian_signed` | byte order, sign, full-scale −32768 |
| UT-MPU-03 | `mpu_read_propagates_i2c_error` | bus error → non-zero return |
| UT-MPU-04 | `mpu_filter_converges_to_accel_tilt` | steady state at +10°, −25°, 0° within 0.1° |
| UT-MPU-05 | `mpu_gyro_integrates_between_accel_corrections` | gyro-only lag = ω·τ (≈ 24.5° at 100 °/s) |
| UT-MPU-06 | `mpu_gyro_bias_calibration_removes_offset` | 1.5 °/s offset → residual drift < 0.05° over 10 s |
| UT-MPU-07 | `mpu_calibration_fails_cleanly_on_bus_error` | no hang, error returned |
| SIM-01 | `sim_recommended_balances_across_plant_gains` | recommended gains/estimator hold for plant gain 0.02…0.2 m/s²/% (0.4×…4×), lean ≤ 12°, steady lean < 3°, drift < 1.5 m in 15 s |
| SIM-02 | `sim_recommended_tolerates_sensor_noise` | accel ±0.03 g, gyro ±0.5 °/s uniform noise |
| SIM-03 | `sim_recommended_recovers_from_push` | 0.6 rad/s (34 °/s) impulse at t = 2 s |
| SIM-04 | `sim_recommended_starts_from_larger_lean` | released at 12° |
| SIM-05 | `sim_wrong_speed_sign_diverges` | guards the sign convention (doc 04 §7) |
| SIM-06 | `sim_speed_loop_stable_up_to_4_deg_per_mps` | speed-loop gain upper bound |
| SIM-07 | `sim_speed_gain_magnitude_matters_not_just_sign` | `−0.05` (right sign, shipped size) still diverges |
| SIM-08 | `sim_KNOWN_DEFECT_shipped_config_falls` | **pins defects C1+C2**: shipped `balance_main.c`/`mpu60x0.c` configuration falls |
| SIM-09 | `sim_KNOWN_DEFECT_shipped_speed_gains_fall_even_with_good_estimator` | isolates C1 |
| SIM-10 | `sim_KNOWN_DEFECT_shipped_filter_falls_even_without_speed_loop` | isolates C2 |

How the simulation is wired: the test loop mirrors `balance_ctrl_task()` (200 Hz): it writes the plant's
accelerometer/gyro values into the **fake I2C register file**, calls the *real* `mpu60x0_read()` and
`mpu60x0_update_angle()`, feeds encoder counts through the same `Δcounts/dt` formula, runs the *real*
`pid_update()` twice, and applies the result to the plant with a 20 ms actuator lag and a 6 % deadband. So
what is tested is the firmware's own arithmetic, not a re-implementation – except the **reference
estimator** (`est_update()` in `test_sim.c`), which is the planned replacement and moves into `src/` with
its own unit tests when implemented (§3.2).

Limits of the simulation (be honest about them): the plant parameters are assumptions, there is no motor
back-EMF model, no wheel slip, no I2C/printf timing, and the pitch dynamics are linear-plus-`sin`. Passing
SIM tests means "the structure and signs are right and there is margin in the *model*", not "the car will
balance". Hence the system-identification step in doc 04 §12 and the HIL/field levels below.

### 3.2 Unit tests to add together with the planned code **[plan]**

| ID | Module | Assertions |
|----|--------|-----------|
| EST-01 | gated complementary filter in `src/` | accel norm gate rejects 1.2 g sample; α/τ; gyro-only fallback; no NaN with ‖a‖ = 0 |
| EST-02 | bias tracking | converges to injected 2 °/s bias within 30 s while still; does not move while the gate is closed |
| EST-03 | Kalman variant | converges; covariance stays positive definite over 10⁶ steps; matches complementary within 1° on benign data |
| EST-04 | initialisation from accel | θ(0) = θ_acc to 0.1° |
| FLT-01 | stuck-frame detector | 20 identical frames → fault; one differing byte → no fault |
| FLT-02 | gyro saturation | |raw| ≥ 32 700 for 4 cycles → fault, 3 cycles → none |
| FLT-03 | accel-norm window | 0.4 g for 100 ms → warn; 500 ms → fault |
| FLT-04 | I2C error policy | 1–2 consecutive errors reuse last sample; 3 → coast + fault; recovery routine called once |
| CTL-01 | angle PD with rate input | no kick on setpoint step (inverse of UT-PID-05); output equals analytic value |
| CTL-02 | speed PI sign/units | `+0.1 m/s` error → lean-back command of `Kv_p·0.1` degrees |
| CTL-03 | integrator anti-windup | frozen while saturated; separate `i_max` honoured |
| CTL-04 | slew limiter | step 0 → 0.5 m/s takes ≥ 1 s at `acc_max = 0.5` |
| CTL-05 | mixer + deadband compensation | table-driven: (u, turn) → (L, R) incl. saturation priority (keep `u`, reduce `turn`) |
| CTL-06 | kinematics | encoder deltas → v, ω, x, ψ for straight, arc, spin |
| ST-01 | state machine | every transition in doc 01 §7.1 incl. illegal ones; outputs are zero in all non-BALANCING states |
| ST-02 | arming check | needs 1 s of |θ−trim| < 3° and |ω| < 10 °/s; interrupted window restarts |
| SHM-01 | seqlock reader/writer | 10⁷ iterations with two threads and random yields: zero torn reads |
| SHM-02 | ring buffer | wraparound, overrun counter, consumer slower than producer |
| SHM-03 | parameter block | bad CRC, NaN, out-of-range, wrong version → rejected, old values kept, error code returned |
| CMD-01 | `SET_TARGET` pack/unpack | int16 range, sign, clamping |
| CMD-02 | heartbeat watchdog | targets zero after `hb_zero_ms`; restored on resume; never disarms |
| BCD-01 | `bcd` against fake RTOS | host process + memfd shm + thread emulating the RTOS: command round trip, dead-man, parameter push/ack, telemetry decimation |

Quality gates: **line coverage ≥ 85 % per file now, 90 % for new files**; branch coverage reported;
every `FAULT_*` has a test (list generated from the enum and compared with the test names – a tiny script in CI).

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

### 8.2 New workflow **[now]**: `.github/workflows/balance-car-tests.yml`

Triggers: push/PR touching `freertos/cvitek/task/balance_car/**`, `docs/balance_car/**` or the workflow; manual dispatch.

| Job | What | Gate |
|-----|------|------|
| `unit` matrix: gcc, gcc + ASan/UBSan, clang | `make test` (`-std=c99 -Wall -Wextra -Werror`) – 30 tests | any failure or warning fails |
| `coverage` | `make coverage` (gcc `--coverage`, gcov) – `pid`, `encoder`, `mpu60x0` ≥ 85 % lines; uploads `.gcov` | below threshold fails |
| `docs` | `docs/balance_car/check_links.py` – relative links and anchors | broken link fails |

Verified locally with gcc (plain, ASan+UBSan, coverage) and clang (plain); the clang+sanitizer combination is deliberately
not in the matrix because it could not be verified in the authoring environment (missing compiler-rt).
The workflow file itself has not yet run on GitHub – the first run is the real verification.

### 8.3 Planned additions

| Job | Description | Notes |
|-----|-------------|-------|
| `fw-build` | Cross-compile the RTOS image (`freertos/cvitek/build_cv181x.sh`, toolchain `riscv64-unknown-elf-gcc` from `milkv-duo/host-tools`) and fail on warnings in `task/balance_car` | Needs the same environment preparation as `ci.yml`; unverified. Artifact: `cvirtos.elf`. |
| `fw-size` | `riscv64-unknown-elf-size` of the image; fail if text+data+bss grows beyond a budget; check the `bc_shm` window assertion from the linker script | Budget set after the first successful build |
| `static` | `cppcheck --enable=warning,performance,portability` and `clang-tidy` (bugprone, cert) on the pure-logic files | Start report-only, then gate |
| `fault-coverage` | script: every `FAULT_*` enum value appears in a test | cheap, catches untested safety rules |
| `bcd` tests | Python/C unit tests for the Linux daemon (BCD-01) | once `bcd` exists |
| `hil-nightly` | self-hosted runner next to a DuoS bench: build → copy `cvirtos.elf` to `/lib/firmware`, `rproc-start.sh`, run HIL-01…07, IPC-01…07, collect `period_us` histogram and telemetry CSV, upload artifacts, fail on KPI regressions | Fixture: relay for power cycling, USB-serial, encoder emulator (second MCU), IMU rotary jig |
| `release-gate` | tag build requires green `unit`, `coverage`, `fw-build`, `hil-nightly` of the last 3 days, and signed-off FLD checklist | |

### 8.4 Branch and review policy

* PRs touching `balance_car/` need the `balance-car-tests` checks green.
* Any change to a gain default, filter constant, fault threshold or state-machine rule must update the
  matching test **and** the parameter table in doc 04 §10 in the same PR.
* Flipping a `KNOWN_DEFECT` test requires a link to the finding ID (C1…C10, doc 04 §9; S1…, doc 06).

## 9. Test data and reporting

* Telemetry CSV schema = `bc_telem_t` ([01 §6.2](01-system-architecture.md)); every HIL/FLD run stores the CSV
  plus a short `results/<date>-<test-id>.md` (setup, firmware build id, result, plots).
* A small Python script (`tools/bc_report.py` **[plan]**) renders θ(t), motor %, speed, jitter histogram and
  checks the KPIs of §4–§6 so pass/fail is computed, not eyeballed.
* The firmware build id (git SHA) is in `bc_shm` header and in each CSV, making every result traceable.

## 10. How to run everything that exists today

```sh
# unit + simulation tests
make -C freertos/cvitek/task/balance_car/test test
# with sanitizers
make -C freertos/cvitek/task/balance_car/test clean test SAN=1
# coverage gate
make -C freertos/cvitek/task/balance_car/test clean coverage
# docs links
python3 docs/balance_car/check_links.py
```
