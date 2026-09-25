#ifndef BALANCE_HW_PWM_H
#define BALANCE_HW_PWM_H

#include <stdint.h>

#define CVI_PWM0_BASE	0x03060000UL
#define CVI_PWM1_BASE	0x03061000UL
#define CVI_PWM2_BASE	0x03062000UL
#define CVI_PWM3_BASE	0x03063000UL

/* clkgen: enable bit for clk_pwm (REG_CLK_EN_1 bit 8). */
#define CVI_CLKGEN_BASE		0x03002000UL
#define CVI_CLK_EN_1		0x004
#define CVI_CLK_PWM_BIT		8

int hw_pwm_init(void);
int hw_pwm_config(uint8_t chip, uint8_t channel, uint32_t period_ns,
		  uint32_t duty_ns);
int hw_pwm_enable(uint8_t chip, uint8_t channel, int enable);
/* duty_pct: 0 .. 100 */
int hw_pwm_set_percent(uint8_t chip, uint8_t channel, uint32_t period_ns,
		       uint8_t duty_pct);

#endif /* BALANCE_HW_PWM_H */
