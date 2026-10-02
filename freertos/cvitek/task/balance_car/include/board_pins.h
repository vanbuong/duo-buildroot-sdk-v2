/*
 * Milk-V DuoS (SG2000 / cv181x) default wiring for a two-wheel balance car.
 *
 * Edit this file to match your harness. Pads are remuxed in board_pins_init().
 * Keep these lines exclusive to FreeRTOS (disable the same pins in the Linux
 * DTS if Linux also claims them).
 *
 * PWMA/PWMB use hardware PWM (VIVO_D10 / VIVO_D9 → PWM_1 / PWM_2 on pwm0).
 * Direction, STBY and encoders use XGPIO on the remaining VIVO pads.
 * MPU6050/6500 default bus: I2C0 (IIC0_SCL/SDA pads). I2C1 on PAD_MIPIRX4P/N
 * is selectable with BC_MPU_I2C_ID 1, but those pads are CSI lane pad 4 of the
 * 15-pin camera connector, so it blocks that camera (docs/balance_car/01 2.1).
 * I2C2/I2C3 belong to the cameras and must not be used here.
 */

#ifndef BALANCE_BOARD_PINS_H
#define BALANCE_BOARD_PINS_H

#include "gpio.h"

/* MPU6050 / MPU6500 ------------------------------------------------------- */
#define BC_MPU_I2C_ID		0	/* 0: IIC0 pads (default), 1: PAD_MIPIRX4P/N */
#define BC_MPU_ADDR_AD0_LOW	0x68
#define BC_MPU_ADDR_AD0_HIGH	0x69
#define BC_MPU_ADDR		BC_MPU_ADDR_AD0_LOW

/*
 * TB6612FNG – motor A = left, motor B = right
 * PWM channels: global PWM index = bank*4 + ch  (pwm0 @ 0x03060000).
 */
#define BC_PWMA_CHIP		0
#define BC_PWMA_CH		1	/* PWM_1 on VIVO_D10 */
#define BC_PWMB_CHIP		0
#define BC_PWMB_CH		2	/* PWM_2 on VIVO_D9 */
#define BC_PWM_PERIOD_NS	50000	/* 20 kHz */

#define BC_PIN_AIN1		GPIOB_PIN(20)	/* VIVO_D1  / XGPIOB_20 */
#define BC_PIN_AIN2		GPIOB_PIN(19)	/* VIVO_D2  / XGPIOB_19 */
#define BC_PIN_BIN1		GPIOB_PIN(17)	/* VIVO_D4  / XGPIOB_17 */
#define BC_PIN_BIN2		GPIOB_PIN(16)	/* VIVO_D5  / XGPIOB_16 */
#define BC_PIN_STBY		GPIOB_PIN(15)	/* VIVO_D6  / XGPIOB_15 */

/* JGB37-520 quadrature encoders (A/B per motor) --------------------------- */
#define BC_PIN_ENC_L_A		GPIOB_PIN(14)	/* VIVO_D7  / XGPIOB_14 */
#define BC_PIN_ENC_L_B		GPIOB_PIN(13)	/* VIVO_D8  / XGPIOB_13 */
#define BC_PIN_ENC_R_A		GPIOB_PIN(21)	/* VIVO_D0  / XGPIOB_21 */
#define BC_PIN_ENC_R_B		GPIOB_PIN(18)	/* VIVO_D3  / XGPIOB_18 */

/* Control loop rate (Hz). Must equal the FreeRTOS tick (200 Hz): the control
 * task blocks in vTaskDelayUntil(1 tick). */
#define BC_CTRL_HZ		200

/*
 * Encoder decoding: 1 = GPIO edge interrupts (default, no blind windows during
 * the I2C read), 0 = polling at the start/end of every control cycle (fallback
 * if the GPIO interrupt path misbehaves on a board).
 */
#ifndef BC_ENC_USE_IRQ
#define BC_ENC_USE_IRQ		1
#endif
#define BC_ENC_IRQ_NUM		42	/* GPIO1_INTR_FLAG: GPIOB bank on the C906L PLIC */

/* Board-level overrides of the run-time parameter defaults (docs/04 section 7:
 * flip these when the bring-up check shows a wrong polarity). The Linux side
 * (bcd) can still change every value at run time. */
#define BC_BOARD_IMU_SIGN	(+1)
#define BC_BOARD_MOTOR_SIGN_L	(+1)
#define BC_BOARD_MOTOR_SIGN_R	(+1)
#define BC_BOARD_ENC_SIGN_L	(+1)
#define BC_BOARD_ENC_SIGN_R	(+1)

/* JGB37-520: ~11 PPR motor-side; set gear ratio to match your motors. */
#define BC_ENC_PPR_MOTOR	11
#define BC_ENC_GEAR_RATIO	90
#define BC_ENC_COUNTS_PER_REV	(BC_ENC_PPR_MOTOR * 4 * BC_ENC_GEAR_RATIO)

void board_pins_init(void);

#endif /* BALANCE_BOARD_PINS_H */
