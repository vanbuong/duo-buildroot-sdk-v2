# FreeRTOS balance-car firmware (C906L little core)

Two-wheel self-balancing robot control for **Milk-V DuoS** (SG2000):

| Part | Role |
|------|------|
| MPU6050 / MPU6500 | Pitch via I2C0 (`IIC0_SCL/SDA`; I2C1 on `PAD_MIPIRX4P/N` selectable but blocks the 15-pin camera) |
| TB6612FNG | Dual H-bridge + HW PWM |
| 2× JGB37-520 | Geared 12 V motors with quadrature encoders |

Runs as a CVIRTOS task on the little core alongside the mailbox (`CMDQU`) path used by Linux remoteproc.

## Default DuoS pin map

Edit `include/board_pins.h` if your wiring differs. Remux is applied in `board_pins_init()`.

| Signal | Pad | Function |
|--------|-----|----------|
| MPU SCL/SDA | IIC0_SCL / IIC0_SDA (`BC_MPU_I2C_ID 0`) | I2C0 |
| TB6612 PWMA | VIVO_D10 | PWM_1 (pwm0 ch1) |
| TB6612 PWMB | VIVO_D9 | PWM_2 (pwm0 ch2) |
| AIN1 / AIN2 | VIVO_D1 / VIVO_D2 | XGPIOB_20 / 19 |
| BIN1 / BIN2 | VIVO_D4 / VIVO_D5 | XGPIOB_17 / 16 |
| STBY | VIVO_D6 | XGPIOB_15 |
| Enc L A/B | VIVO_D7 / VIVO_D8 | XGPIOB_14 / 13 |
| Enc R A/B | VIVO_D0 / VIVO_D3 | XGPIOB_21 / 18 |

**Power:** TB6612 VM = 12 V for the JGB37-520 motors. Logic (VCC) = 3.3 V. DuoS GPIO is **3.3 V only** — do not drive 5 V into the pads.

**Linux side:** `duo-init.sh` no longer loads the PWM module; `&i2c4` and `&spi3` (pads `VIVO_D0..D8`) are disabled in the four DuoS DTS; `&i2c0` is already disabled. I2C2/I2C3 are the camera buses – never use them here.

## Build

Built automatically with CVIRTOS (`freertos/cvitek/build_cv181x.sh`) when `BALANCE_CAR` is enabled (default for this tree). Output firmware remains `cvirtos.bin` / `cvirtos.elf` (or load via remoteproc as `arduino.elf` / `cvirtos.elf`).

```bash
# From SDK top after envsetup / defconfig for DuoS:
./freertos/cvitek/build_cv181x.sh
# Install path: freertos/cvitek/install/bin/cvirtos.{elf,bin}
```

Copy `cvirtos.elf` to the SD root (or `/lib/firmware/`) as expected by your remoteproc `firmware-name`.

## Architecture (see docs/balance_car/01, 04)

* `bc_ctrl` task, 200 Hz, blocks in `vTaskDelayUntil`: IMU burst read → `bc_core_step()` (health, gated complementary
  estimator, state machine, speed PI → angle PD → turn PI → mixer) → TB6612 + 20 kHz HW PWM.
* Encoders: GPIO both-edge interrupts (`BC_ENC_USE_IRQ`), 4× decode.
* States: BOOT → CALIBRATING → IDLE → ARMING → BALANCING (→ FALLEN / FAULT / ESTOP). Motors only run in BALANCING; every other
  state coasts with `STBY = 0`. Boot never arms by itself.
* Linux interface: `IP_BALANCE` mailbox commands (arm, disarm, target, parameters, calibrate, heartbeat) + a 64 KiB shared window
  at the top of the carve-out (telemetry/event rings, parameter block). Linux side: `/mnt/system/bc/` (`bcd`, `bcctl`, web UI).
* Parameters (gains, limits, signs) are a run-time set; defaults in `bc_params.h`, board sign overrides `BC_BOARD_*_SIGN`.
* Hold the robot still and upright during the 1 s IMU calibration at boot.

Tests: `make -C test test` (C) and `make -C test pytest` (Linux side); CI also builds the real RTOS image.

## MPU address

Default `0x68` (AD0 low). Set `BC_MPU_ADDR` to `BC_MPU_ADDR_AD0_HIGH` (`0x69`) if AD0 is pulled high.

## Gear ratio

Set `BC_ENC_GEAR_RATIO` in `board_pins.h` (it seeds the `enc_cpr` parameter) to match your JGB37-520 variant (common values 30/45/60/90/150).
