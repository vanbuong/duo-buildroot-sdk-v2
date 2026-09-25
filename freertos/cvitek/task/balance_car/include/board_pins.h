/*
 * Milk-V DuoS (SG2000 / cv181x) default wiring for a two-wheel balance car.
 *
 * Edit this file to match your harness. Pads are remuxed in board_pins_init().
 * Keep these lines exclusive to FreeRTOS (disable the same pins in the Linux
 * DTS if Linux also claims them).
 *
 * PWMA/PWMB use hardware PWM (VIVO_D10 / VIVO_D9 → PWM_1 / PWM_2 on pwm0).
 * Direction, STBY and encoders use XGPIO on the remaining VIVO pads.
 * MPU6050/6500 is on I2C1 (PAD_MIPIRX4P/N).
 */

#ifndef BALANCE_BOARD_PINS_H
#define BALANCE_BOARD_PINS_H

#include "gpio.h"

/* MPU6050 / MPU6500 ------------------------------------------------------- */
#define BC_MPU_I2C_ID		1	/* I2C1 */
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

/* Control loop rate (Hz). FreeRTOS tick is 200 Hz. */
#define BC_CTRL_HZ		200

/* JGB37-520: ~11 PPR motor-side; set gear ratio to match your motors. */
#define BC_ENC_PPR_MOTOR	11
#define BC_ENC_GEAR_RATIO	90
#define BC_ENC_COUNTS_PER_REV	(BC_ENC_PPR_MOTOR * 4 * BC_ENC_GEAR_RATIO)

void board_pins_init(void);

#endif /* BALANCE_BOARD_PINS_H */
