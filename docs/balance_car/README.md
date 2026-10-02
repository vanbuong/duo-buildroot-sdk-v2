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

## Key findings (details in docs 03, 04, 06)

1. **The shipped control constants do not balance in simulation.** The speed loop is positive feedback for the
   firmware's angle convention and ~500× too large in counts/s units; the 0.98 complementary filter lets the
   car's own acceleration corrupt the angle. A simulation that runs the real `pid.c`/`mpu60x0.c` shows both. A
   corrected configuration (gated filter α = 0.998, `Kp 15`, `Kd 0.8`, speed loop `−1.94 °/(m/s)`) balances
   across a 10× range of assumed plant gains. *The plant is a model with assumed parameters – confirm on
   hardware.*
2. **Safety gaps:** an IMU read error leaves the motors at their last PWM; boot arms immediately; if the RTOS
   stops the PWM block keeps running. Fixes and tests are specified (S1–S3, SAF-08).
3. **Real-time structure:** busy-wait control loop on the 200 Hz tick, `printf` in the loop, and encoder polling
   that is blind during the 0.4 ms I2C read. Target: timer-driven task, edge-interrupt encoders, console task.
4. **Resource conflicts:** the Linux DTS enables I2C1 and `duo-init.sh` loads the PWM module – both used by the
   RTOS; and the IMU pads (`PAD_MIPIRX4P/N`) are MIPI-RX pads that may collide with the camera connector.
5. **No Linux↔RTOS interface exists yet.** Design: new `IP_BALANCE` mailbox id for 32-bit commands + a 64 KiB
   shared-memory window with seqlock telemetry ring, double-buffered parameters and heartbeats.

## What this branch contains

| Item | Status |
|------|--------|
| Design documents 01–06 | written |
| Host unit tests + closed-loop simulation (`freertos/cvitek/task/balance_car/test`) | **30 tests / 78 checks pass** (gcc, gcc+ASan/UBSan, clang; coverage `pid`/`encoder` 100 %, `mpu60x0` ≈ 88 %) |
| CI workflow `.github/workflows/balance-car-tests.yml` | written, verified locally; first GitHub run pending |
| Firmware behaviour | **unchanged** (findings are documented and pinned by `*_KNOWN_DEFECT_*` tests) |

Run the tests: `make -C freertos/cvitek/task/balance_car/test test`

## Conventions in these documents

* **[now]** exists in the tree · **[plan]** proposed · **[est]** estimate to be measured · **[sim]** simulation result.
* Numbers marked **[est]** are not measurements. Nothing here was run on a DuoS board.
