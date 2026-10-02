# 03 – MPU6050/6500 handling and sensor data processing

Covers: how the IMU is configured and read, how raw data becomes `pitch` / `pitch-rate`, how encoder
data becomes wheel speed / odometry, calibration, fault detection, and the numerical choices behind them.

Legend: **[now]** exists in `freertos/cvitek/task/balance_car`, **[plan]** proposed, **[sim]** result
from the closed-loop simulation in `test/` (see doc 05), **[est]** to be measured.

---

## 1. Data flow overview

```mermaid
flowchart TB
  subgraph HW
    MPU[MPU6050/6500<br/>accel ±2g, gyro ±250 dps<br/>DLPF 42 Hz, 1 kHz internal]
    ENCL[Encoder L A/B]
    ENCR[Encoder R A/B]
  end
  MPU -- "I2C1 400 kHz, 14-byte burst @ 0x3B" --> RAW[raw int16 ax ay az temp gx gy gz]
  RAW --> SCALE[scale to g and deg/s<br/>axis map + sign]
  SCALE --> BIAS[subtract gyro bias]
  BIAS --> GATE{|accel|≈1 g ?}
  GATE -- yes --> FUSE[complementary / Kalman update]
  GATE -- no --> GYRO[gyro-only propagate]
  FUSE --> OUT["θ (deg), ω (deg/s)"]
  GYRO --> OUT
  ENCL --> DEC[quadrature decode ×4]
  ENCR --> DEC
  DEC --> SPD[Δcounts/dt → m/s, low-pass]
  SPD --> ODO[distance, heading]
  OUT --> CTRL[control loops]
  SPD --> CTRL
  ODO --> CTRL
```

## 2. MPU6050 / MPU6500 hardware handling

### 2.1 Wiring and bus

| Item | Value | Where |
|------|-------|-------|
| Bus | **[now]** I2C1 on `PAD_MIPIRX4P` (SCL) / `PAD_MIPIRX4N` (SDA) – **collides with CSI lane pad 4 used by the 15-pin camera connector J2**; **[plan]** move to I2C0 (`IIC0_SCL/SDA`), see doc 01 §2.1 | `board_pins.h`, `board_pins_init()` |
| Address | `0x68` (AD0 low), `0x69` if AD0 high | `BC_MPU_ADDR` |
| Pull-ups | 2.2–4.7 kΩ to 3.3 V on SDA/SCL (breakout boards usually include them – check for 5 V-pulled modules!) | hardware |
| Speed | DesignWare I2C master in **polled** mode, fast mode (`IC_CON_SPEED_FS`) ≈ 400 kHz | `poll_i2c.c` |
| Clocks | **[now]** `clk_apb_i2c` (EN_1 bit 6), `clk_i2c` (EN_3 bit 7), `clk_apb_i2c1` (EN_3 bit 18) for I2C1; for I2C0 the matching `clk_apb_i2c0` gate must be looked up and enabled **[verify]** | `board_pins.c` |
| Level | DuoS GPIO/I2C is **3.3 V only** | datasheet |

Transaction cost of the 14-byte burst: start + addr/W + reg + repeated start + addr/R + 14 data bytes ≈
`(1+9+9+1+9+14×9) ≈ 155` bit times → **≈ 0.4 ms at 400 kHz, ≈ 1.6 ms at 100 kHz [est]**. This is the
largest single contributor to the loop's execution time and it is a busy-wait. Never run the bus at
100 kHz in the control path.

### 2.2 Register configuration

**[now]** (`mpu60x0_init()`):

| Register | Value | Meaning |
|----------|-------|---------|
| `PWR_MGMT_1` (0x6B) | `0x01` | wake up, clock = PLL with X-gyro reference |
| `CONFIG` (0x1A) | `0x03` | DLPF_CFG=3: gyro BW 42 Hz (delay ≈ 4.8 ms), accel BW 44 Hz (≈ 4.9 ms), 1 kHz internal rate |
| `GYRO_CONFIG` (0x1B) | `0x00` | ±250 °/s, 131 LSB/(°/s) |
| `ACCEL_CONFIG` (0x1C) | `0x00` | ±2 g, 16384 LSB/g |
| `SMPLRT_DIV` (0x19) | `0x04` | sample rate = 1 kHz / (1+4) = **200 Hz** |
| `WHO_AM_I` (0x75) | read | 0x68 (6050), 0x70 (6500), 0x71 (9250), 0x98 accepted; other values only warn |

**[plan]** changes, each with the reason:

| Setting | Proposed | Reason |
|---------|----------|--------|
| `GYRO_CONFIG` | `0x08` (±500 °/s, 65.5 LSB/°/s) | A fall or a push can exceed 250 °/s; a clipped gyro makes the estimator lie exactly when it matters. Resolution (0.015 °/s/LSB) is still far below the noise floor. Keep the scale constant in one header and derive `GYRO_LSB`. |
| `SMPLRT_DIV` | `0x00` (1 kHz internal output) | The loop reads asynchronously at 200 Hz; with the sensor's own 200 Hz clock there is a beat pattern (repeated or skipped samples, up to 5 ms extra age). Reading a 1 kHz output makes the age ≤ 1 ms and the DLPF still limits bandwidth. |
| `CONFIG` | `0x02` (98 Hz) *or keep 0x03* | 42 Hz DLPF adds ≈ 5 ms delay to `ω`, which is a substantial fraction of the loop's phase budget (PD crossover ≈ 3 Hz in the model, doc 04). Choose by experiment: 98 Hz if vibration allows. Motor/gear vibration is the deciding factor. |
| MPU6500 only: `ACCEL_CONFIG2` (0x1D) | `0x03` (≈ 44 Hz accel DLPF) | On the 6500 the accel DLPF is a *separate* register that the current code never writes, so accel runs with a much wider bandwidth than the 6050 path. Gate by `WHO_AM_I == 0x70/0x71` (and 0x98 variants). |
| Interrupt | `INT_PIN_CFG`/`INT_ENABLE` data-ready on a spare GPIO (optional) | Lets the ISR timestamp the sample exactly. The simpler alternative is the 1 kHz output above. |
| Temperature | read bytes 6–7 (already in the burst) | Used for gyro bias drift monitoring (§5.3). |

Keep reading the whole 14-byte block: accel (6) + temp (2) + gyro (6) in one transaction guarantees the
samples belong together.

### 2.3 Decoding **[now]**

```c
ax = (int16_t)((buf[0]  << 8) | buf[1]);   // big-endian, signed
ay = ...buf[2..3]; az = ...buf[4..5];
// buf[6..7] = temperature (ignored today)
gx = ...buf[8..9]; gy = ...buf[10..11]; gz = ...buf[12..13];
```

Verified by host unit test `mpu_read_decodes_big_endian_signed` (doc 05).

### 2.4 Axes, signs and mounting

The estimator computes `θ_acc = atan2(−ax, √(ay²+az²))` and integrates `gyro_y`. For a different mounting
only a constant axis map changes; make it explicit **[plan]**:

```c
#define BC_IMU_PITCH_AXIS_ACC   (-ax)          /* which accel axis points along travel */
#define BC_IMU_PITCH_AXIS_GYR   ( gy)          /* gyro axis for pitch rate            */
#define BC_IMU_YAW_AXIS_GYR     ( gz)          /* for turning                         */
#define BC_IMU_SIGN             (+1)           /* flips both angle and rate           */
```

Consistency rule (this is the single most common bring-up bug): **accel-derived angle and gyro rate must
have the same sign**. If they disagree the complementary filter fights itself: static tilt looks fine,
but the angle lags/overshoots whenever the car moves. Check: hold the car still, tilt it by hand slowly –
`pitch_acc` and the integrated `gyro` must move the same way (telemetry fields exist for this).

Control sign requirement is separate and is covered in doc 04 §7.

## 3. Attitude estimation

### 3.1 [now] complementary filter

```
θ[k] = α · (θ[k-1] + ω[k]·dt) + (1-α) · θ_acc[k],   α = 0.98, dt = 5 ms (assumed constant)
time constant τ = α·dt / (1-α) = 0.245 s
```

Host tests prove it converges (`mpu_filter_converges_to_accel_tilt`) and integrates gyro correctly
(`mpu_gyro_integrates_between_accel_corrections`). It has **three weaknesses**:

1. **Accelerometer contamination.** The accelerometer measures *specific force*, i.e. gravity **plus the
   car's own acceleration**. When the motors push at 90 % the wheels accelerate (≈ 4–5 m/s² in the
   assumed model) and the accel-derived angle is off by up to `atan(a/g) ≈ 25°` **[est]**. With
   `1-α = 2 %` this leaks into θ within ≈ 0.25 s, and because the error is *correlated with the motor
   command* it behaves like positive feedback. **[sim]**: with the shipped `α = 0.98` and
   `Kp = 25` the simulated car falls within ≈ 0.35 s (plant gain 0.05 m/s²/%, test
   `sim_KNOWN_DEFECT_shipped_filter_falls_even_without_speed_loop`; the design-time scan below shows the
   same for the other plant gains).
2. **Constant `dt`.** Real period is `tick-phase + printf + I2C`; integration error ≈ `ω·Δt`.
3. **Initial condition `θ = 0` and no bias tracking.** At boot the angle starts at 0 and takes ≈ τ to
   find the real tilt; gyro bias is measured once.

### 3.2 [plan] recommended: gated complementary filter with bias tracking

```
ω   = gyro_y_dps − b                           (b = bias estimate)
θ⁻  = θ[k-1] + ω·dt                            (predict, dt measured)
‖a‖ = √(ax²+ay²+az²)  [g]
if | ‖a‖ − 1 | < G  and  |ω| < W_max :         (accel trustworthy: ≈ static or constant velocity)
        e   = θ_acc − θ⁻
        θ   = θ⁻ + (1-α)·e                     (α = 0.998 → τ = 2.5 s)
        b  += k_b · e · dt                     (slow bias correction, k_b ≈ 0.02 …0.1 /s) [est]
else :  θ   = θ⁻                               (gyro only)
```

Parameters: `α = 0.998`, gate `G = 0.10 g`. **[sim]** results from the design-time parameter scans
(angle loop only, speed loop off, wheel acceleration leaking into the simulated accelerometer, four plant
gains 0.02 / 0.05 / 0.1 / 0.2 m/s² per %, Kd = 0.8). The scans were exploratory and are not kept as tests;
the regression tests pin only the shipped and the recommended configuration.

| Configuration | Result (cases balanced out of 4 plant gains, per Kp) |
|---------------|-------------------------------------------------------|
| `α = 0.98`, no gate | Kp 6: 3/4; Kp 10, 15, 25: **0/4** |
| `α = 0.98` + gate 0.1 g | Kp 6: 4/4; Kp 10–25: 2/4 (only the higher plant gains) |
| `α = 0.995`, no gate | Kp 6: 3/4; Kp 10, 15: 4/4; Kp 25: **0/4** |
| `α = 0.998`, no gate | Kp 6: 3/4; Kp 10, 15, 25: 4/4 |
| **`α = 0.998` + gate 0.1 g, Kp 15** | **4/4**, steady lean ≤ 2.0° (limit cycle from the 6 % motor deadband) |

Take-away: a larger gyro weight is what matters most; the gate adds margin for aggressive gains. The
shipped combination (`α = 0.98`, `Kp = 25`, no gate) is the *worst* corner of this table. The conclusion
depends on the assumed plant (§3.1 item 1) and must be confirmed on the real car (tests `HIL-05`,
`FLD-01`).

The gate is intentionally loose (±0.1 g): at ±0.1 g the accel error is ≤ 5.7° and `1-α` is tiny, so a
short excursion changes θ by < 0.02°.

Why not just trust the gyro? Gyro integration drifts (bias, temperature, scale error). The
accelerometer is the long-term reference; the design goal is to *use it slowly and only when valid*.

Cost: ≈ 25 flops + one `atan2f`, `sqrtf` – a few µs on the C906 FPU **[est]**; negligible next to the I2C read.

### 3.3 [plan, optional] 2-state Kalman filter (angle + gyro bias)

State `x = [θ, b]ᵀ`, input `u = ω_meas`:

```
Predict:   θ ← θ + (u − b)·dt
           P ← F P Fᵀ + Q,   F = [[1, −dt],[0, 1]],  Q = diag(Qθ·dt, Qb·dt)
Update:    S = P00 + R ;  K = [P00/S, P10/S]ᵀ ;  y = θ_acc − θ
           θ ← θ + K0·y ;  b ← b + K1·y
           P ← (I − K H) P ,  H = [1, 0]
```

Typical starting values (to be tuned on logged data): `Qθ = 0.001`, `Qb = 0.003`, `R = 0.03`
(R scales with the accel noise² and can be *inflated* when the norm gate fails, e.g. `R = 10`). Benefits:
automatic bias tracking and covariance-aware weighting. Cost: ≈ 60 flops. Recommendation: implement the
gated complementary filter first (simple, testable); keep the Kalman as an A/B option behind a build/run
flag, selected by data from the tuning logs.

### 3.4 Time base

* Use a free-running 1 MHz (or mtime) counter: `dt = clamp(t_now − t_prev, 2 ms, 10 ms)`; if clamped,
  increment a diagnostic counter.
* All filters take `dt` as an argument (already the case: `mpu60x0_update_angle(imu, dt)`).
* Export `period_us`, `exec_us` in telemetry; the jitter statistics are part of the acceptance tests.

### 3.5 Initialisation

At the end of calibration set `θ = θ_acc` from the averaged accel vector (instead of 0). The robot must
be held still – the boot sequence verifies `‖a‖ ≈ 1 g` and low gyro variance, otherwise the calibration is
rejected (§5.1).

## 4. Encoder processing

### 4.1 Hardware

JGB37-520 motors with Hall quadrature encoders: `11 PPR` at the motor shaft, ×4 decoding, gear ratio `N`
(default 90) → `BC_ENC_COUNTS_PER_REV = 11 × 4 × 90 = 3960` counts per wheel revolution
(`board_pins.h`; set `BC_ENC_GEAR_RATIO` to match your motor).

For a 65 mm wheel (circumference 0.2042 m): **19 390 counts/m**, i.e. 1 count = 0.052 mm.
Resolution of the 5 ms speed estimate: 1 count / 5 ms = **200 counts/s = 0.0103 m/s per LSB** – coarse
compared with the speeds of interest (0.1–0.5 m/s), which is why §4.3 low-passes it.

### 4.2 Decode **[now]**

`encoder_poll()` samples A and B, forms `idx = (prev<<2) | curr`, adds `qdec_lut[idx] ∈ {−1,0,+1}`. Verified by
host tests: 4 counts per cycle, symmetric direction, no count without motion, bounce cancels, illegal
(skipped) transitions count 0, reset.

Sign convention of the shipped LUT (from test `encoder_counts_4x_per_cycle_forward`): the sequence
`00 → 01 → 11 → 10` (**B leads A**, with `curr = A<<1|B`) counts **+**. Which physical direction that is
depends on wiring – calibrate with a sign macro (`BC_ENC_SIGN_L/R`, doc 04 §7).

### 4.3 Limitation and plan: polling → interrupts

**[now]** the control task polls both encoders in a busy loop (`udelay(40)` between polls, so ≈ 20 kHz
sample rate when nothing else runs) **except while the IMU burst is in progress** (≈ 0.4 ms) and while
`printf` blocks. If the motor shaft produces an edge every `T_edge`, a blind window `W` loses
`W / T_edge` edges and, worse, a skipped state produces **no count and a wrong direction ambiguity**.
Example **[est]**: motor shaft at 6 000 rpm, 11 PPR ×4 → 4.4 kHz edge rate → `T_edge = 227 µs`; a 0.4 ms
I2C window misses ≈ 2 edges per control cycle. That systematic loss shows up as a *speed-dependent scale
error*, which is invisible at standstill and wrecks the speed loop at speed.

**[plan]** use edge interrupts on A (both edges) for each wheel (PLIC GPIO interrupt; the current GPIO
helper has no IRQ support → extend `gpio.c`), with an ISR body that reads A and B, applies the LUT
and increments a counter (≤ 2 µs). Alternatives if GPIO IRQs prove awkward: a 10–20 kHz timer ISR that
polls (deterministic, still loses nothing as long as the sample rate ≥ 4× max edge rate), or
reading the I2C sensor from a lower-priority context. Acceptance: **no count loss** in the bench test with
a signal generator at the maximum edge rate (test `HIL-02`).

### 4.4 Wheel speed and odometry [plan]

```
Δc_L = cL[k] − cL[k−1]              (int32 subtraction is wrap-safe)
v_L  = s_L · Δc_L / dt / counts_per_m   [m/s]       (s_L = ±1 sign macro)
v    = ½ (v_L + v_R)                forward speed
ω_enc= (v_R − v_L) / track          yaw rate from wheels
v_f[k] = v_f[k−1] + β (v − v_f[k−1])    β ≈ 0.3  (≈ 4-sample / 20 ms time constant) [est]
x += v·dt    (distance, for position hold / drive-distance commands)
ψ += ω_gyro_z·dt            (heading from gyro Z; encoders only as a slow correction)
```

Units: the firmware currently feeds **counts/s** into the speed PID, which is why its gains look
unusual (`SPEED_KP 0.05 °` per count/s). Converting to m/s makes gains physically meaningful (doc 04 §3).

## 5. Calibration

### 5.1 Gyro bias at boot **[now, to harden]**

`mpu60x0_calibrate_gyro(imu, 200)` averages 200 samples × 5 ms = **1 s** of `gy` only (host test
`mpu_gyro_bias_calibration_removes_offset`: 1.5 °/s offset removed, residual angle drift < 0.05°).
Hardening **[plan]**:

* calibrate all three gyro axes (yaw rate is used for turning);
* **reject** if variance of any axis exceeds a threshold (≈ 0.5 °/s RMS **[est]**) or `‖a‖` is not
  within 1 ± 0.05 g – robot was moving; retry up to 3× then `FAULT`;
* store result in the shm calibration block so Linux can display and persist it;
* run again on the `BC_CMD_CALIBRATE` command while DISARMED.

### 5.2 Level trim (centre-of-gravity offset)

The mechanical balance point is rarely at `θ = 0` (battery position, IMU mounting angle). Procedure
**[plan]**:

1. Hold the car at the angle where it feels balanced (hands off briefly, or on the stand), send
   `BC_CMD_CALIBRATE(2)`; the RTOS averages `θ` for 1 s and stores `trim_deg`.
2. The angle loop uses `θ_set = trim + speed-loop output`.
3. The speed-loop integral (doc 04) removes small residual errors automatically.

### 5.3 Drift monitoring

Log `temp`, `gyro_bias` and `trim` every minute; warn when bias moves > 1 °/s from the boot value (sensor
warm-up). Optional online bias tracking is part of the filter in §3.2/3.3.

### 5.4 Accelerometer offsets

Not required for balancing (the trim absorbs a constant offset). A 6-position calibration is only needed
if the accel norm gate regularly rejects a still robot; then store `offset[3]` and `scale[3]`.

## 6. Fault detection and recovery

| Check | Rule | Action |
|-------|------|--------|
| WHO_AM_I | not in {0x68, 0x70, 0x71, 0x98} | boot warning (today); `FAULT` unless `ALLOW_UNKNOWN_IMU` |
| I2C error | `poll_i2c_read` ≠ 0 | reuse last sample ≤ 2 cycles, then coast + `FAULT_IMU_BUS` **[now: `continue` – no coast, keeps last PWM!]** |
| Stuck data | 14-byte frame identical ×20 | `FAULT_IMU_STUCK` |
| Saturation | `|raw gyro| ≥ 32 700` for > 3 cycles | `FAULT_IMU_RANGE` |
| Accel norm | `‖a‖ ∉ [0.5, 1.5] g` for > 100 ms | warn, gyro-only; fault after 500 ms |
| Spikes | `|θ[k] − θ[k−1]| > 10°` in one cycle | reject sample, count |
| Bus recovery | after 3 consecutive errors: toggle SCL 9× as GPIO, issue STOP, re-init I2C + sensor | automatic once; then `FAULT` |
| Re-init | on `BC_CMD_DISARM` + `BC_CMD_CALIBRATE` | clears the fault if sensor answers |

Note on **[now]** behaviour: on a failed read the loop executes `continue` *before* the motor update,
so the motors keep the **previous duty** with a stale angle. This is the highest-priority safety fix in
the roadmap (item S1 in doc 06).

## 7. Numerical and implementation notes

* Use `float` (the C906L has the D extension); keep `atan2f`/`sqrtf` out of ISRs.
* `mpu60x0_t` holds both raw ints and the angle; split into `imu_raw_t`, `imu_cal_t`, `attitude_t`
  **[plan]** so the pure math (estimator, calibration statistics) has no I2C dependency and runs on the host.
* Keep scale constants (`ACCEL_LSB`, `GYRO_LSB`) derived from the configured range in one place; the host
  test for `GYRO_CONFIG` should assert both the register value and the LSB constant (prevents a silent
  factor-of-two when the range is changed).
* Telemetry records both `pitch` and `pitch_acc` so a log shows immediately whether the gate/filter is
  doing its job.

## 8. Tests that cover this document (details in doc 05)

| Behaviour | Test |
|-----------|------|
| Register init values, WHO_AM_I, burst decode, I2C error propagation | `mpu_init_accepts_known_who_am_i`, `mpu_read_decodes_big_endian_signed`, `mpu_read_propagates_i2c_error` |
| Filter convergence and gyro integration | `mpu_filter_converges_to_accel_tilt`, `mpu_gyro_integrates_between_accel_corrections` |
| Gyro bias calibration and failure path | `mpu_gyro_bias_calibration_removes_offset`, `mpu_calibration_fails_cleanly_on_bus_error` |
| Quadrature decode | six `encoder_*` tests |
| Contaminated-accel behaviour, recommended estimator | closed-loop `sim_*` tests (the reference estimator is in `test_sim.c`; move it to `src/` when implemented) |
| Not yet covered (planned): gate logic in `src/`, Kalman, stuck-data and range faults, bus recovery | unit tests `EST-*`, `FLT-*` in doc 05 |
