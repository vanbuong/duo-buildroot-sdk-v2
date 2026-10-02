# Two-wheel balance robot on Milk-V DuoS – design and development plan

Platform: SG2000 – Cortex-A53 running Linux (big core) + C906L running FreeRTOS (little core).
Existing firmware: `freertos/cvitek/task/balance_car` (MPU6050/6500, TB6612FNG, 2× JGB37-520 encoders).

## Documents

| # | Document | Answers |
|---|----------|---------|
| 01 | [System architecture, task schedule, inter-core communication](01-system-architecture.md) | **What does the A53/Linux do, what does the C906L/FreeRTOS do? How are tasks scheduled? How do the cores talk (mailbox, shared memory, heartbeat, safety state machine)?** |
| 02 | [Video streaming over Wi-Fi and teleoperation](02-video-streaming-wifi.md) | **Can it stream video over Wi-Fi?** Yes – on the A53 with the SDK's own camera/encoder/RTSP/AIC8800 stack; pipeline, profiles, protocol, latency budgets, failure handling |
| 03 | [MPU handling and data-processing algorithms](03-imu-and-data-processing.md) | I2C and register configuration, decoding, attitude estimation (gated complementary filter / Kalman), encoder processing, calibration, fault detection |
| 04 | [PID, balance algorithm and movement control](04-pid-and-motion-control.md) | Physics, cascaded controller, gains from first principles, movement commands, sign bring-up, tuning procedure, parameter table |
| 05 | [Test plan, unit tests, CI](05-test-plan-and-ci.md) | Test pyramid, 30 implemented host tests, planned HIL/system/field tests, traceability, CI workflows |
| 06 | [Findings, roadmap, risks, decisions](06-development-roadmap.md) | Review findings on the current code, milestones M0–M8 with estimates, risk register, decision log |

## Architecture at a glance

```
 Phone / PC ── Wi-Fi ──┬── RTSP video  ◄── VENC ◄── ISP ◄── CSI camera           ┐ Cortex-A53 / Linux
                       └── WebSocket ──► bcd ──┐ params, telemetry, heartbeat    ┘
                                               │  mailbox (8 B cmds) + shared memory window
 IMU ── I2C ─┐                                 ▼                                  ┐
 Encoders ───┼─► bc_ctrl 200 Hz: estimator → speed PI → angle PD → mixer → PWM ──►│ C906L / FreeRTOS
 Safety ─────┘   state machine, fault table, heartbeat watchdog                   ┘   TB6612 + motors
```

Rule: **the robot must stay upright with Linux hung or Wi-Fi gone.** Linux only sends setpoints.

## Implementation status

Everything below is **implemented in this branch and verified on GitHub CI** (all 31 checks green on the last pushed head,
including the full SDK image builds for every board) **but has never run on a DuoS board.** Hardware-dependent items are listed
under "Not done".

| Area | Status | Verified by |
|------|--------|-------------|
| Control core: params, gated estimator, cascaded controller, state machine, IMU health, command watchdog | done | 107 host tests / 513 checks (gcc, gcc+ASan/UBSan, clang), coverage 90–100 % per file |
| Closed-loop simulation through the real core + MPU driver | done | `test_sim.c` (assumed plant!) |
| RTOS firmware: blocking 200 Hz control task, IRQ encoders, IMU on I2C0, boot→IDLE, coast on fault, console task | done | real RTOS image builds in CI (cv181x with balance, cv180x without); **not run on hardware** |
| Linux↔RTOS interface: `IP_BALANCE`, 64 KiB shared window (linker-asserted), seqlock rings, parameter block | done | host stress tests + Python↔C interop tests |
| Linux side: `bcd` (teleop, heartbeat, dead-man, parameters, emergency STBY clear), `bcctl`, web UI | done | 57 Python tests incl. WebSocket round trip |
| Two cameras, one at a time (`bc_camera.py`) | done | tests with fake I2C/GPIO; chip ids unverified |
| Resource ownership: PWM module no longer loaded, `&i2c4`/`&spi3` disabled in the four DuoS DTS | done | `dtc` compile in CI |
| CI: unit, coverage, pytest, layout drift, RTOS image, DTS, docs | done | green on GitHub |

### Not done (needs hardware or a decision)

* **Everything on real hardware:** HIL, system and field tests ([05](05-test-plan-and-ci.md) §4–6), system identification
  and gain tuning (the defaults come from an *assumed* plant), the video/Wi-Fi jitter test (`PERF-03`).
* **Unverified assumptions:** the IIC0 pads reach the DuoS header (else set `BC_MPU_I2C_ID 1` and lose J2); GPIO IRQ number 42
  and the DW GPIO register use for the encoders (fallback: `BC_ENC_USE_IRQ 0`); camera chip ids and Linux GPIO numbering;
  the RTSP binary/arguments in `bc-video.sh`; that the PWM block keeps its last state if the RTOS dies (the emergency STBY clear
  is implemented for it).
* **Not implemented:** hardware e-stop/STBY supervisor and battery sense (hardware), Kalman filter, odometry/position hold,
  I2C bus-recovery, spike filter, CPU-load measurement, video profiles/adaptive bitrate/OSD, embedded video in the web page,
  IMX219 sensor driver, `hostapd` in the Buildroot defconfig, mailbox events RTOS→Linux (off by default, ring is used),
  hardware-timer-paced control (tick-paced today), HIL nightly runner.

## Key findings (history; details in docs 03, 04, 06)

1. **The first firmware version's control constants could not balance in simulation** (speed loop positive feedback and ~500× too
   large in counts/s; 0.98 filter corrupted by the car's own acceleration). Replaced by the m/s speed loop and the gated filter;
   a corrected configuration balances across a 10× range of *assumed* plant gains.
2. **Safety gaps** (stale PWM after an IMU error, arming at boot, runaway if the RTOS stops) are closed in software; the last one
   still needs a hardware supervisor for full coverage.
3. **Real-time structure** (busy-wait loop, `printf` in the loop, encoders blind during I2C) replaced by a blocking task, a console
   task and edge interrupts.
4. **Resource conflicts** (Linux owned I2C1, PWM, I2C4/SPI3 pads; IMU pads blocked the 15-pin camera) resolved by moving the IMU to
   I2C0 and releasing the Linux side.
5. **Two cameras, one at a time:** J1 16-pin GC2083 (I2C3, MCLK0) / J2 15-pin OV5647 (I2C2, MCLK1); IMX219 has no SDK driver.

## Conventions in these documents

* **[now]** implemented in this branch · **[plan]** not implemented · **[est]** estimate to be measured · **[sim]** simulation result.
* Numbers marked **[est]** are not measurements. Nothing here was run on a DuoS board.
