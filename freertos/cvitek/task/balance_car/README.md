# FreeRTOS balance-car firmware (C906L little core)

Two-wheel self-balancing robot control for **Milk-V DuoS** (SG2000):

| Part | Role |
|------|------|
| MPU6050 / MPU6500 | Pitch via I2C1 |
| TB6612FNG | Dual H-bridge + HW PWM |
| 2× JGB37-520 | Geared 12 V motors with quadrature encoders |

Runs as a CVIRTOS task on the little core alongside the mailbox (`CMDQU`) path used by Linux remoteproc.

## Default DuoS pin map

Edit `include/board_pins.h` if your wiring differs. Remux is applied in `board_pins_init()`.

| Signal | Pad | Function |
|--------|-----|----------|
| MPU SCL/SDA | PAD_MIPIRX4P / PAD_MIPIRX4N | I2C1 |
| TB6612 PWMA | VIVO_D10 | PWM_1 (pwm0 ch1) |
| TB6612 PWMB | VIVO_D9 | PWM_2 (pwm0 ch2) |
| AIN1 / AIN2 | VIVO_D1 / VIVO_D2 | XGPIOB_20 / 19 |
| BIN1 / BIN2 | VIVO_D4 / VIVO_D5 | XGPIOB_17 / 16 |
| STBY | VIVO_D6 | XGPIOB_15 |
| Enc L A/B | VIVO_D7 / VIVO_D8 | XGPIOB_14 / 13 |
| Enc R A/B | VIVO_D0 / VIVO_D3 | XGPIOB_21 / 18 |

**Power:** TB6612 VM = 12 V for the JGB37-520 motors. Logic (VCC) = 3.3 V. DuoS GPIO is **3.3 V only** — do not drive 5 V into the pads.

**Linux conflict:** Disable or leave unused the same PWM / GPIO / I2C1 nodes in the Linux DTS so the little core owns them.

## Build

Built automatically with CVIRTOS (`freertos/cvitek/build_cv181x.sh`) when `BALANCE_CAR` is enabled (default for this tree). Output firmware remains `cvirtos.bin` / `cvirtos.elf` (or load via remoteproc as `arduino.elf` / `cvirtos.elf`).

```bash
# From SDK top after envsetup / defconfig for DuoS:
./freertos/cvitek/build_cv181x.sh
# Install path: freertos/cvitek/install/bin/cvirtos.{elf,bin}
```

Copy `cvirtos.elf` to the SD root (or `/lib/firmware/`) as expected by your remoteproc `firmware-name`.

## Control architecture

1. **IMU** — MPU6050/6500 at 200 Hz, complementary filter → pitch (deg)
2. **Encoders** — 4× quadrature decode, polled in the control task
3. **Cascaded PID** — speed loop → angle setpoint; angle loop → motor %
4. **TB6612** — direction GPIOs + hardware PWM @ 20 kHz
5. **Safety** — disarm and coast if `|pitch| > 45°`

Starting gains in `balance_main.c` (`ANGLE_KP/KD`, `SPEED_KP/KI`) need chassis-specific tuning. Hold the robot upright during gyro bias calibration at boot.

## MPU address

Default `0x68` (AD0 low). Set `BC_MPU_ADDR` to `BC_MPU_ADDR_AD0_HIGH` (`0x69`) if AD0 is pulled high.

## Gear ratio

Set `BC_ENC_GEAR_RATIO` in `board_pins.h` to match your JGB37-520 variant (common values 30/45/60/90/150).
