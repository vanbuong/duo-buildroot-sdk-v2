# 06 – Findings, development roadmap, risks and decisions

Turns the design in docs 01–05 into an ordered work plan, starting from what the code in
`freertos/cvitek/task/balance_car` actually does today.

Legend: **[now]** exists, **[plan]** proposed, **[est]** estimate (person-days for one engineer
who knows the SDK; widen by 1.5× for someone new to it).

---

## 0. Implementation status (read this first)

| Milestone | Status | Notes |
|-----------|--------|-------|
| M0 Hardware/ownership validation | **partly done** | software side done: IMU on I2C0 (`BC_MPU_I2C_ID`), Linux PWM module not loaded, `&i2c4`/`&spi3` disabled, image size measured (99 KB text, headroom 1.2 MiB before the shm window). **Hardware items open:** IIC0 pad routing, STBY pull-down, e-stop, battery sense, HIL-01/03/04 |
| M1 Safety/correctness fixes | **done** | S1, S3, S5, S11, S13, C1–C3 fixed (sign/units/gains, boot to IDLE, coast + STBY off outside BALANCING, printf out of the loop) |
| M2 Estimator + controller as pure modules | **done** | gated filter + bias tracking, derivative-on-rate, speed PI in m/s, turn PI, slew, mixer, state machine, health monitor; **plus** spike filter, optional robust Kalman, odometry, optional position hold, MPU6050/6500 variant detection with read-back-verified init; 154 host tests |
| M3 Real-time structure | **mostly done** | blocking task, measured dt, timing statistics, console task, IRQ encoders (`BC_ENC_USE_IRQ`), IMU burst. I2C bus recovery and `bc_ctrl` task-load measurement added (both unverified on hardware). **Not done:** HW-timer pacing (tick-paced), PWM update-in-place (S12); IRQ path unverified on hardware |
| M4 Linux↔RTOS interface | **done** | `IP_BALANCE` (both headers), `bc_shm` window + linker assert, seqlock rings, parameter block/echo, heartbeat watchdog, Linux emergency STBY clear, `bcd`/`bcctl`; RTOS→Linux mailbox events deliberately off (shm ring instead) |
| M5 Wi-Fi teleoperation | **done in software** | `bc-net.sh` (STA, AP fallback if hostapd present), WebSocket server, web UI, dead-man; **not run**; `BR2_PACKAGE_HOSTAPD=y` is in the four DuoS defconfigs; telemetry logging (rotating JSONL) added |
| M6 Video | **partly done** | camera select (`bc_camera.py`), `bc-video.sh`; profiles + adaptive-rate logic + web selector done in software (effect depends on the RTSP binary, unverified); **not done:** OSD, embedded video, jitter test, IMX219 driver (needs the sensor/driver) |
| M7 Identification, tuning, field validation | **not done** | needs the robot |
| M8 CI maturation | **mostly done** | `unit`, `coverage`, `pytest`, layout drift, **real RTOS image builds**, DTS; firmware size budget in the RTOS build check, `cppcheck` job (report only); open: HIL nightly runner (needs a board), release gate |

Verification status of everything marked done: compiled/tested in CI as described in [05 §8](05-test-plan-and-ci.md); **no
code has run on a DuoS board**.

## 1. State before this work (kept for context)

The first firmware version ran a polled-I2C IMU, TB6612 with HW PWM, polled encoders and a cascaded PID from `main_cvirtos()`,
with no Linux interface, no safety state machine beyond a 45° latch, and constants that did not balance in simulation.

## 2. Findings on the first version and their status

Control/algorithm findings C1–C10 are in [04 §9](04-pid-and-motion-control.md). System and software findings:

| ID | Sev | Finding | Evidence | Resolution |
|----|-----|---------|----------|--------------------------|
| S1 | **High** | IMU read failure executes `continue` before the motor update → motors keep the **last PWM** with a stale angle | `balance_main.c` loop | coast on any IMU fault, 3-strikes policy (M1) |
| S2 | **High** | If the RTOS crashes/stops, PWM and GPIO latches keep their last state | design review; verify with SAF-08 | Linux emergency STBY clear + hardware watchdog on STBY (M4, hw) |
| S3 | High | Boot arms the motors immediately (`g_armed = 1`); estimator starts at θ = 0 | `balance_car_start()` | boot to `IDLE`; arm check; init θ from accel (M1) |
| S4 | High | Control task busy-waits (`udelay(40)`, never blocks) at priority idle+5; period is the 200 Hz FreeRTOS tick; `dt` assumed constant | `balance_ctrl_task()`, `FreeRTOSConfig.h` | HW-timer-notified task, measured dt (M3) |
| S5 | Med | `printf` (≈ 10 ms at 115 200 baud **[est]**) executes inside the control loop every 2 s | `balance_ctrl_task()` | console task + log ring (M1/M3) |
| S6 | High | Encoders are polled only between I2C transfers (≈ 0.4 ms blind **[est]**) → edge loss at speed | code + UT-ENC-05 | GPIO-edge ISR or timer-ISR polling (M3) |
| S7 | High | No communication with Linux: no commands, telemetry, parameters; `g_target_speed/turn` never written | code | `IP_BALANCE` + `bc_shm` + `bcd` (M4) |
| S8 | High | Linux DTS enables `&i2c1` (the IMU bus) and `duo-init.sh` loads `cv181x_pwm.ko` (the PWM block) → ownership conflict | `sg2000_milkv_duos_*_sd.dts`, `duo-init.sh` | disable in DTS/scripts (M0) |
| S9 | **High (confirmed)** | `PAD_MIPIRX4P/N` (IMU I2C1) are CSI lane pad 4, used by the 15-pin camera connector J2 (`lane_id = 5,3,4`) | `sensor_cfg_OV5647_J2.ini`, `func.h` | move IMU to I2C0; confirm header pins and clock gates (M0) |
| S17 | High | Firmware remaps `VIVO_D0..D8` to GPIO but U-Boot muxes them as I2C4 (touch) / SPI3 and the DuoS DTS enables `&i2c4`, `&spi3` | `cvi_board_init.c`, DuoS DTS | disable those nodes in the balance image or relocate signals (M0) |
| S18 | Med | No IMX219 sensor driver in the SDK (15-pin connector supports OV5647 only) | `cvi_mpi/component/isp/sensor` | optional port (M6) |
| S10 | Med | Gyro range ±250 °/s clips in a fall/push; MPU6500 accel DLPF register (0x1D) never written; sample clock 200 Hz asynchronous to the loop | `mpu60x0.c` | ±500 °/s, 1 kHz output, 6500 config (M2) |
| S11 | Med | No sign macros for IMU/motor/encoder – wrong wiring cannot be corrected without code changes | headers | `BC_*_SIGN` + bring-up procedure (M1) |
| S12 | Low | `hw_pwm_enable()` stops/restarts the channel on every duty update | `hw_pwm.c` | update-in-place with `PWMUPDATE` (M3) |
| S13 | Med | `tb6612_brake()` and `STBY` handling: STBY cleared only on the fall path, not on other stops | `balance_main.c` | all non-BALANCING states force coast + STBY=0 (M1) |
| S14 | Low | No watchdog, no stack/CPU monitoring, no timing statistics | code | idle hook + telemetry fields (M3) |
| S15 | Low | Gains/limits are `#define`s | `balance_main.c` | parameter block pushed from Linux (M4) |
| S16 | Info | The firmware has never been run against a model before this branch; plant values are assumed | doc 04 §11 | system identification (M7) |

**Status of the findings:** fixed in software – S1, S3, S4, S5, S6 (IRQ path unverified on hardware), S7, S8, S9 (IMU moved to I2C0, pad
routing to confirm), S10, S11, S13, S15, S17; mitigated in software, needs a hardware supervisor for full cover – S2; partly done –
S14 (stack/timing telemetry yes, CPU load/watchdog no); open – S12 (PWM restart quirk), S16 (system identification), S18 (IMX219).

## 3. Milestones

Dependencies are strictly top-down unless stated. "Exit tests" refer to IDs in [doc 05](05-test-plan-and-ci.md).

### M0 – Hardware and ownership validation (≈ 2–3 d)

| Task | Output |
|------|--------|
| Move the IMU to I2C0 (confirm the pads reach the 40-pin header; find the I2C0 clock gates); resolve the `VIVO_D0..D8` overlap with I2C4/SPI3 (S9, S17) | `BC_MPU_I2C_ID`, `board_pins_init()`, DTS note |
| Remove Linux ownership: `&i2c1`/chosen bus disabled, no `cv181x_pwm.ko`, GPIO 461–469 untouched (S8) | DTS + `duo-init.sh` patch (balance image variant) |
| Build `cvirtos.elf`, record `size`, check free heap/stack, link the 64 KiB `bc_shm` window assertion | numbers in doc 01 §2.2 |
| Hardware: STBY pull-down, e-stop button, power switch, battery sense divider to an ADC pin if available | wiring note |
| Run HIL-01, HIL-03, HIL-04 on the existing firmware (bench, wheels free) | `results/` entries |

Exit: IMU reads clean, PWM/STBY waveforms correct, no Linux/RTOS resource conflicts.

### M1 – Safety and correctness quick fixes in the existing firmware (≈ 3 d)

Small, low-risk edits to `balance_main.c`/headers; no architecture change.

| Task | Fixes | Tests |
|------|-------|-------|
| IMU failure → coast, 3-strike counter | S1 | extend fake I2C in `test_mpu.c`; HIL-06 |
| Boot to `IDLE`; arm only after upright check (UART command `arm`) | S3 | ST-02 (host, once state machine is extracted) |
| `BC_IMU_SIGN`, `BC_MOTOR_SIGN_*`, `BC_ENC_SIGN_*` macros; bring-up procedure | S11 | HIL-08 |
| STBY/coast forced in every non-balancing state | S13 | HIL-04 |
| Move `printf` out of the loop (ring buffer + low-priority drain, or temporary rate limit) | S5 | HIL-07 |
| Interim gains from doc 04 §10; speed loop in m/s; **flip SIM-08..10** | C1, C2, C3 | SIM-01..07 |

Exit: the car can be hand-balanced on the stand with correct signs; `KNOWN_DEFECT` tests removed/inverted.

### M2 – Estimator and controller rewrite as pure modules (≈ 5–6 d)

| Task | Output |
|------|--------|
| `estimator.c`: gated complementary filter, bias tracking, init from accel; optional Kalman | EST-01..04 |
| Split `mpu60x0` into driver (I2C) and math; ±500 °/s, 1 kHz output, MPU6500 `ACCEL_CONFIG2` | S10 |
| `pid.c`: derivative-on-rate, separate `i_max`, freeze when saturated; speed PI / turn PI | CTL-01..03, C5, C6 |
| `cmd.c` slew limiter, `mixer.c` deadband/clamp/priority, `kinematics.c` | CTL-04..06 |
| `state.c`: full state machine and fault table of doc 01 §7 | ST-01, ST-02, FLT-01..04 |
| Move the reference estimator from `test_sim.c` into `src/` and test it directly | |

Exit: ≥ 90 % line coverage on every new file; all SIM tests green with the production estimator.

### M3 – Real-time structure (≈ 4–5 d)

| Task | Fixes | Exit tests |
|------|-------|-----------|
| Hardware-timer ISR → `bc_ctrl` notify; measured `dt`; remove busy-wait | S4 | HIL-07 |
| Encoder edge ISRs (extend `gpio.c` with IRQ support) | S6 | HIL-02 |
| Console task + log ring; idle hook (CPU load, stack watermark, status GPIO) | S5, S14 | PERF-01/02 |
| PWM update in place | S12 | HIL-03 |
| Timing statistics (`period_us`, `exec_us`, deadline miss counter) + `TIMING` fault | | HIL-07 |

Exit: period 5 ms ± 0.1 ms p99 with Linux idle.

### M4 – Linux ↔ RTOS interface (≈ 6–7 d)

| Task | Output |
|------|--------|
| `IP_BALANCE` in both `rtos_cmdqu.h` copies, `E_QUEUE_BALANCE`, dispatcher case, `prvBalanceCommTask` | doc 01 §6.1 change set |
| `bc_shm.h` shared header, RTOS writer/reader with cache rules, linker assertion | SHM-01..03 |
| Linux access: reserved-memory node + small UIO/char driver (dev: `/dev/mem`) | |
| `bcd` v0: CLI (`bcctl arm/disarm/set/get/watch`), heartbeat, params JSON, telemetry logging | BCD-01 |
| Heartbeat watchdog in RTOS; Linux emergency STBY path (S2) | CMD-02, IPC-05, SAF-08 |

Exit: IPC-01…07 pass; the car is fully controllable from Linux CLI; killing `bcd` does not make it fall.

### M5 – Wi-Fi teleoperation (≈ 4–5 d)

| Task | Output |
|------|--------|
| STA bring-up script, power-save off, `iperf3` baseline; add `hostapd` to defconfig; AP + fallback | NET-01, NET-05 |
| `bcd` WebSocket server, dead-man, rate limiting, parameter/tuning messages | NET-02..04 |
| Web UI page (joystick, gauges, plots, tuning sliders) served by `bcd` | |

Exit: phone drives the car over Wi-Fi; link drop → stands still.

### M6 – Video (≈ 4–5 d)

| Task | Output |
|------|--------|
| `bc-camera` probe/select for J1 (GC2083, 16-pin) and J2 (OV5647, 15-pin), one active at a time; reset-hold of the unused sensor | VID-05..07 |
| (optional) IMX219 sensor driver for J2 (S18) | |
| Camera + `rtsp_server_video` on DuoS, profiles `fpv-low/med/hq`, `bc_video.json` | VID-01, VID-03 |
| Adaptive bitrate/fps from link statistics | VID-04 |
| OSD telemetry overlay | |
| Init-script ordering (`S90`–`S97`), video failure isolated from `bcd` | |
| Jitter test with everything running | PERF-03, PERF-04, VID-02 |

Exit: FPV on the phone with ≤ 250 ms latency; control jitter unchanged.

### M7 – Identification, tuning, field validation (≈ 5–7 d, hardware-bound)

| Task | Output |
|------|--------|
| System identification (doc 04 §12); update `sim_plant.c`; re-run sweeps | HIL-09, `results/` |
| Gain tuning on stand then floor (doc 04 §8); store defaults | FLD-01..04 |
| Battery/payload/surface sweep; soak 30 min with video | FLD-05..08 |
| Freeze parameter defaults; write the operator's guide | |

Exit: all FLD pass; release candidate.

### M8 – CI maturation (≈ 3–4 d, parallel from M2 on)

`fw-build` and `fw-size` jobs, `static` analysis, fault-coverage script, `bcd` tests, then the self-hosted
`hil-nightly` runner and the `release-gate` (doc 05 §8.3).

### Timeline (single engineer, [est])

```
week:  1        2        3        4        5        6
M0 ██
M1  ███
M2     ██████
M3           █████
M4                 ███████
M5                        █████
M6                             █████
M7                                  ███████ (hardware bound; M7 id/tuning can start after M2)
M8        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ (parallel)
```

Roughly 6–7 weeks for one engineer to a field-validated release; two engineers (RTOS/control vs Linux/network)
can overlap M2–M3 with M5–M6 after M4's interface is agreed and shave ≈ 2 weeks.

## 4. Work breakdown by owner

| Area | Tasks |
|------|-------|
| RTOS / control | M1, M2, M3, RTOS side of M4, M7 tuning |
| Linux / network / video | Linux side of M4, M5, M6, init scripts, Buildroot packages (`hostapd`), DTS |
| Hardware | M0 wiring items, e-stop, STBY safeguard, battery sense, IMU jig, encoder emulator |
| QA / CI | test harness extensions, M8, HIL fixture, reporting script |

## 5. Definition of done (per feature)

1. Design note updated in `docs/balance_car/`.
2. Unit tests for the pure logic, meeting the coverage rule (≥ 90 % new files).
3. A test that makes each new safety rule fire.
4. Parameter table (doc 04 §10 / doc 01 §6.4) updated.
5. HIL/system test executed and recorded in `results/` for anything touching timing, IPC or actuators.
6. `balance-car-tests` workflow green; no new compiler warnings.

## 6. Risk register

| ID | Risk | L | I | Mitigation | Trigger / owner |
|----|------|---|---|------------|-----------------|
| K1 | IMU on `MIPIRX4` pads blocks the 15-pin camera (**confirmed**); new I2C0 pads may not be on the header | H | H | move IMU to I2C0; check schematic; last resort: IMU on a bit-banged GPIO I2C | before M0 exit |
| K2 | Plant model is wrong → gains from the sim do not transfer | H | M | treat sim as structure/sign validation; HIL-09 identification; tune on stand first | M7 |
| K3 | DDR contention from video stretches RTOS timing | M | H | PERF-03 early (M6 starts with it); reduce resolution/fps; keep I2C/PWM code small and in cache | M6 |
| K4 | RTOS crash leaves motors running | M | **Critical** | S2 mitigations; SAF-08 must pass before untethered tests | M4 |
| K5 | Cache-coherency bugs in `bc_shm` | M | M | non-cacheable Linux mapping, explicit flush/invalidate on RTOS, SHM-01 stress, IPC-03 | M4 |
| K6 | GPIO IRQ support missing/awkward on C906L HAL | M | M | timer-ISR polling alternative (doc 03 §4.3) | M3 |
| K7 | Wi-Fi latency spikes / AIC8800 driver quirks | M | M | USB-NCM fallback; power-save off; DSCP; adaptive video; control independent of link | M5 |
| K8 | 650 KiB heap / 2 MiB carve-out too small after additions | L | M | `size` budget check in CI (`fw-size`); shrink stacks/heap | M2+ |
| K9 | Motor/encoder variant differs (gear ratio, PPR) | M | L | constants in `board_pins.h`; sysid step | M7 |
| K10 | Battery sag changes loop gain | H | M | battery scaling, FLD-06, ADC measurement | M7 |
| K11 | Unverified CI workflow fails on first run | M | L | first run is the verification; fix-forward | now |
| K12 | Safety: spinning wheels injure people | L | H | tethered first tests, current-limited supply, e-stop, lift detection | all hardware tests |

(L = likelihood, I = impact: L/M/H.)

## 7. Decision log

| ID | Decision | Rationale | Revisit when |
|----|----------|-----------|--------------|
| D1 | All balance-critical code runs on the C906L; Linux only requests setpoints | Linux is not hard real-time; P1 | never |
| D2 | Mailbox for commands/events, shared memory for bulk data | Mailbox is 8 slots × 8 B; telemetry needs ~12 KB/s | if rpmsg/virtio is ported |
| D3 | Derivative on gyro rate, not on error | no setpoint kick, less noise | – |
| D4 | Gated complementary filter first, Kalman optional (**now implemented as `est_mode = 1`, default stays complementary**) | simple, testable, simulation-backed | if tuning logs show drift problems |
| D5 | Speed loop in m/s, gains in °/(m/s) | physically meaningful, transfers across gear ratios/wheels | – |
| D6 | HW-timer driven control task instead of changing the global tick | keeps the rest of the image unchanged | if the HAL timer is impractical → 1 kHz tick |
| D7 | Heartbeat loss zeroes targets but keeps balancing | disarming a moving robot is more dangerous than standing still | product safety review |
| D8 | Develop over USB-NCM, ship AP+STA | repeatable benchmarking before radio variables | – |
| D9 | Video stays entirely on Linux (cvi_mpi + cvi_rtsp) | existing hardware encoder and samples | – |
| D10 | Host tests link fakes instead of using `#ifdef` hooks | production code stays clean; fakes double as HIL fixtures' reference | – |

## 8. Open questions (need an answer from the hardware owner)

1. Are the `IIC0_SCL/SDA` pads routed to the DuoS 40-pin header (new IMU bus)? Which I2C0 clock gate bits apply? (K1)
   *Answered:* the CSI connectors use lane pads 2/0/1 (16-pin J1) and 5/3/4 (15-pin J2), so `MIPIRX4` is not free if J2 must work.
2. Exact motor variant (gear ratio, rated voltage), wheel diameter, track width, battery chemistry/cell count.
3. Is a battery-voltage sense wired to an ADC pin? If not, add a divider.
4. Is an e-stop button / STBY supervisor acceptable on the chassis?
5. Target use: indoor teleop only, or with AI (follow-me)? (decides whether `tdl_sdk`/TPU load must be in PERF-03)
6. Preferred client: native app, browser (WebRTC), or ROS bridge?
7. AIC8800D80 band/capabilities as reported by `iw phy` on the production image.
8. Does the SoC watchdog reset only the C906L, or the whole chip? (affects the RTOS-death policy)

## 9. Deliverables of this branch

| Path | Purpose |
|------|---------|
| `docs/balance_car/` | design + status documents 01–06, `check_links.py` |
| `freertos/cvitek/task/balance_car/src, include` | RTOS application: pure core (`bc_params`, `estimator`, `control`, `bc_state`, `imu_health`, `bc_cmd`, `bc_core`, `bc_shm.h`, `bc_proto.h`), drivers/glue (`mpu60x0`, `encoder`, `enc_irq`, `tb6612`, `hw_pwm`, `board_pins`), tasks (`balance_main.c`, `balance_comm.c`) |
| `freertos/cvitek/task/comm/`, `driver/rtos_cmdqu/`, `osdrv/interdrv/rtos_cmdqu/` | `IP_BALANCE` routing |
| `freertos/cvitek/scripts/cv181x_lscript.ld`, `kernel/include/riscv64/FreeRTOSConfig.h` | shm window assertion, stack watermark API |
| `freertos/cvitek/task/CMakeLists.txt`, `task/main/CMakeLists.txt` | balance firmware only for `cv181x` |
| `freertos/cvitek/task/balance_car/test/` | 154 host tests, fakes, simulation, helper tools |
| `freertos/cvitek/task/balance_car/linux_tests/`, `tools/` | 86 Python tests; layout generator, RTOS build check, DTS check |
| `device/generic/rootfs_overlay/duos/mnt/system/bc/` | `bcd`, `bcctl`, `bc_shm`, `bc_camera`, scripts, web UI, generated layout |
| `device/generic/rootfs_overlay/duos/mnt/system/duo-init.sh` | starts the services, no Linux PWM module |
| `build/boards/cv181x/*duos*/dts_*/*.dts` | `&i2c4`, `&spi3` disabled |
| `.github/workflows/balance-car-tests.yml` | CI |


## 10. What is left, split by whether it needs hardware

**Needs hardware (cannot be completed or verified in software):**
all HIL/system/field tests (doc 05 §4–6) and gain tuning from system identification; IIC0 pad routing, GPIO IRQ number and DW GPIO use,
I2C bus-recovery pad mapping, camera chip ids and Linux GPIO numbering; IMX219 sensor (needs a driver/ISP tuning file for the SDK);
hardware e-stop, STBY supervisor, battery sense; the real RTSP binary and encoder reconfiguration for the video profiles; the Wi-Fi/video
jitter test (`PERF-03`); HIL nightly runner (needs a board on a self-hosted runner).

**Not hardware-dependent but deliberately not done:** hardware-timer-paced control and PWM update-in-place (S12) – both change the
real-time core and are only worth doing once measurements on a board show the tick-paced loop or the PWM restart glitch is a problem;
mailbox events RTOS→Linux (the shm ring is used and works); OSD and embedded video in the web page (depend on the media stack);
DMP as the default estimator (decided against; see doc 03 §2.2a).

**Added as an experiment, not verified on hardware:** `est_mode = 2` DMP angle with the MotionApps 6.12 image (doc 03 §3.5) – needs a bench test with the wheels off the ground, and a decision on whether the MPU6500 family accepts the image.
