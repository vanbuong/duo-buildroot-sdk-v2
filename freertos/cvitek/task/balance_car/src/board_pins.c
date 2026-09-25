#include "board_pins.h"
#include "hal_pinmux.h"
#include "cv181x_pinmux.h"
#include "pinctrl.h"
#include "mmio.h"
#include "printf.h"

#define CVI_CLKGEN_BASE	0x03002000UL
#define CVI_CLK_EN_1	0x004
#define CVI_CLK_EN_3	0x00C

void board_pins_init(void)
{
	/* clk_apb_i2c (EN_1 bit6), clk_i2c (EN_3 bit7), clk_apb_i2c1 (EN_3 bit18) */
	mmio_setbits_32(CVI_CLKGEN_BASE + CVI_CLK_EN_1, (1U << 6));
	mmio_setbits_32(CVI_CLKGEN_BASE + CVI_CLK_EN_3, (1U << 7) | (1U << 18));

	/* I2C1 for MPU6050/6500 on PAD_MIPIRX4P/N */
	hal_pinmux_config(PINMUX_I2C1);

	/* Hardware PWM for TB6612 PWMA / PWMB */
	PINMUX_CONFIG(VIVO_D10, PWM_1);
	PINMUX_CONFIG(VIVO_D9, PWM_2);

	/* Direction / STBY / encoders as GPIO */
	PINMUX_CONFIG(VIVO_D0, XGPIOB_21);
	PINMUX_CONFIG(VIVO_D1, XGPIOB_20);
	PINMUX_CONFIG(VIVO_D2, XGPIOB_19);
	PINMUX_CONFIG(VIVO_D3, XGPIOB_18);
	PINMUX_CONFIG(VIVO_D4, XGPIOB_17);
	PINMUX_CONFIG(VIVO_D5, XGPIOB_16);
	PINMUX_CONFIG(VIVO_D6, XGPIOB_15);
	PINMUX_CONFIG(VIVO_D7, XGPIOB_14);
	PINMUX_CONFIG(VIVO_D8, XGPIOB_13);

	printf("[balance] DuoS pins remuxed (PWM1/2, VIVO→GPIO, MIPIRX4→I2C1)\n");
}
