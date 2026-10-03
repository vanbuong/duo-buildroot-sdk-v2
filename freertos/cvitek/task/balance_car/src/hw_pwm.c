#include "hw_pwm.h"
#include "mmio.h"
#include "printf.h"

struct cvi_pwm_regs {
	volatile uint32_t hlperiod0;	/* 0x00 */
	volatile uint32_t period0;	/* 0x04 */
	volatile uint32_t hlperiod1;	/* 0x08 */
	volatile uint32_t period1;	/* 0x0c */
	volatile uint32_t hlperiod2;	/* 0x10 */
	volatile uint32_t period2;	/* 0x14 */
	volatile uint32_t hlperiod3;	/* 0x18 */
	volatile uint32_t period3;	/* 0x1c */
	volatile uint32_t reserved_1[8];
	volatile uint32_t polarity;	/* 0x40 */
	volatile uint32_t pwmstart;	/* 0x44 */
	volatile uint32_t pwmdone;	/* 0x48 */
	volatile uint32_t pwmupdate;	/* 0x4c */
	volatile uint32_t reserved_2[32];
	volatile uint32_t pwm_oe;	/* 0xd0 */
};

/* U-Boot driver assumes 100 MHz PWM clock. */
#define CVI_PWM_CLK_MHZ	100

static struct cvi_pwm_regs *pwm_base(uint8_t chip)
{
	switch (chip) {
	case 0:
		return (struct cvi_pwm_regs *)CVI_PWM0_BASE;
	case 1:
		return (struct cvi_pwm_regs *)CVI_PWM1_BASE;
	case 2:
		return (struct cvi_pwm_regs *)CVI_PWM2_BASE;
	case 3:
		return (struct cvi_pwm_regs *)CVI_PWM3_BASE;
	default:
		return 0;
	}
}

int hw_pwm_init(void)
{
	/* Ensure clk_pwm gate is on (Linux marks it CRITICAL; FSBL may too). */
	mmio_setbits_32(CVI_CLKGEN_BASE + CVI_CLK_EN_1, 1U << CVI_CLK_PWM_BIT);
	printf("[balance] HW PWM clocks gated on\n");
	return 0;
}

int hw_pwm_config(uint8_t chip, uint8_t channel, uint32_t period_ns,
		  uint32_t duty_ns)
{
	struct cvi_pwm_regs *regs = pwm_base(chip);
	uint32_t period_val, hlperiod_val;

	if (!regs || channel > 3)
		return -1;
	if (duty_ns >= period_ns)
		duty_ns = period_ns ? period_ns - 1 : 0;
	if (period_ns == 0)
		return -1;

	period_val = CVI_PWM_CLK_MHZ * period_ns / 1000;
	hlperiod_val = CVI_PWM_CLK_MHZ * (period_ns - duty_ns) / 1000;
	if (period_val == 0)
		period_val = 1;

	switch (channel) {
	case 0:
		regs->hlperiod0 = hlperiod_val;
		regs->period0 = period_val;
		break;
	case 1:
		regs->hlperiod1 = hlperiod_val;
		regs->period1 = period_val;
		break;
	case 2:
		regs->hlperiod2 = hlperiod_val;
		regs->period2 = period_val;
		break;
	case 3:
		regs->hlperiod3 = hlperiod_val;
		regs->period3 = period_val;
		break;
	}
	return 0;
}

int hw_pwm_enable(uint8_t chip, uint8_t channel, int enable)
{
	struct cvi_pwm_regs *regs = pwm_base(chip);
	uint32_t value;

	if (!regs || channel > 3)
		return -1;

	value = regs->pwm_oe;
	regs->pwm_oe = value | (1U << channel);

	value = regs->pwmstart;
	regs->pwmstart = value & ~(1U << channel);
	if (enable)
		regs->pwmstart = value | (1U << channel);
	return 0;
}

int hw_pwm_set_percent(uint8_t chip, uint8_t channel, uint32_t period_ns,
		       uint8_t duty_pct)
{
	uint32_t duty_ns;

	if (duty_pct > 100)
		duty_pct = 100;
	duty_ns = (uint32_t)((uint64_t)period_ns * duty_pct / 100);
	if (hw_pwm_config(chip, channel, period_ns, duty_ns))
		return -1;
	return hw_pwm_enable(chip, channel, duty_pct > 0);
}
