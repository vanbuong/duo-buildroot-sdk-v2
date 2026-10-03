#include "board_pins.h"
#include "hal_pinmux.h"
#include "cv181x_pinmux.h"
#include "pinctrl.h"
#include "mmio.h"
#include "printf.h"
#include "delay.h"
#include "i2c_recover.h"

#define CVI_CLKGEN_BASE	0x03002000UL
#define CVI_CLK_EN_1	0x004
#define CVI_CLK_EN_3	0x00C

void board_pins_init(void)
{
	/* clk_apb_i2c (EN_1 bit6), clk_i2c (EN_3 bit7) shared by all I2C blocks */
	mmio_setbits_32(CVI_CLKGEN_BASE + CVI_CLK_EN_1, (1U << 6));
	mmio_setbits_32(CVI_CLKGEN_BASE + CVI_CLK_EN_3, (1U << 7));
#if BC_MPU_I2C_ID == 0
	/* clk_apb_i2c0 (EN_3 bit17); IIC0_SCL/SDA pads (U-Boot parks them as GPIO) */
	mmio_setbits_32(CVI_CLKGEN_BASE + CVI_CLK_EN_3, (1U << 17));
	hal_pinmux_config(PINMUX_I2C0);
#elif BC_MPU_I2C_ID == 1
	/* clk_apb_i2c1 (EN_3 bit18); PAD_MIPIRX4P/N: collides with the 15-pin CSI connector */
	mmio_setbits_32(CVI_CLKGEN_BASE + CVI_CLK_EN_3, (1U << 18));
	hal_pinmux_config(PINMUX_I2C1);
#else
#error "BC_MPU_I2C_ID must be 0 or 1 (I2C2/I2C3 are camera buses)"
#endif

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

	printf("[balance] DuoS pins remuxed (PWM1/2, VIVO->GPIO, IMU on I2C%d)\n",
	       BC_MPU_I2C_ID);
}

/*
 * I2C bus recovery: temporarily mux the IMU bus pads to GPIO, bit-bang up to 9
 * clocks (open drain emulated with direction: output-low = 0, input = release;
 * the module's pull-ups provide the high level), then give the pads back to the
 * I2C block. Not yet verified on hardware.
 */
#if BC_MPU_I2C_ID == 0
#define REC_SCL		GPIOA_PIN(28)
#define REC_SDA		GPIOA_PIN(29)
#else
#define REC_SCL		GPIOC_PIN(3)
#define REC_SDA		GPIOC_PIN(2)
#endif

static void rec_drive(int pin, int level)
{
	if (level) {
		gpio_direction_input(pin);
	} else {
		gpio_set_value(pin, 0);
		gpio_direction_output(pin, 0);
	}
}
static void rec_scl(void *c, int l) { (void)c; rec_drive(REC_SCL, l); }
static void rec_sda(void *c, int l) { (void)c; rec_drive(REC_SDA, l); }
static int rec_sda_read(void *c) { (void)c; return gpio_get_value(REC_SDA) ? 1 : 0; }
static void rec_delay(void *c, unsigned us) { (void)c; udelay(us); }

int board_i2c_bus_recover(void)
{
	bc_i2c_recover_ops_t ops = { 0, rec_scl, rec_sda, rec_sda_read, rec_delay };
	int clocks = 0, rc;

#if BC_MPU_I2C_ID == 0
	PINMUX_CONFIG(IIC0_SCL, XGPIOA_28);
	PINMUX_CONFIG(IIC0_SDA, XGPIOA_29);
#else
	PINMUX_CONFIG(PAD_MIPIRX4P, XGPIOC_3);
	PINMUX_CONFIG(PAD_MIPIRX4N, XGPIOC_2);
#endif
	rc = bc_i2c_recover(&ops, &clocks);
#if BC_MPU_I2C_ID == 0
	hal_pinmux_config(PINMUX_I2C0);
#else
	hal_pinmux_config(PINMUX_I2C1);
#endif
	printf("[balance] I2C%d bus recovery: %d clocks, %s\n", BC_MPU_I2C_ID, clocks,
	       rc == 0 ? "SDA released" : "SDA STILL LOW");
	return rc;
}
