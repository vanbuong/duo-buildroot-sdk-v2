/*
 * Minimal DesignWare I2C master (polling) for MPU6050/6500 on C906L.
 * Avoids the unfinished riscv64 hali2c IRQ path.
 */
#include "poll_i2c.h"
#include "mmio.h"
#include "delay.h"
#include "top_reg.h"
#include "printf.h"

#define IC_CON			0x00
#define IC_TAR			0x04
#define IC_DATA_CMD		0x10
#define IC_SS_SCL_HCNT		0x14
#define IC_SS_SCL_LCNT		0x18
#define IC_FS_SCL_HCNT		0x1c
#define IC_FS_SCL_LCNT		0x20
#define IC_INTR_MASK		0x30
#define IC_RAW_INTR_STAT	0x34
#define IC_RX_TL		0x38
#define IC_TX_TL		0x3c
#define IC_CLR_INTR		0x40
#define IC_CLR_TX_ABRT		0x54
#define IC_ENABLE		0x6c
#define IC_STATUS		0x70
#define IC_TX_ABRT_SOURCE	0x80
#define IC_ENABLE_STATUS	0x9c

#define IC_CON_MASTER		(1U << 0)
#define IC_CON_SPEED_FS		(2U << 1)
#define IC_CON_RESTART_EN	(1U << 5)
#define IC_CON_SLAVE_DISABLE	(1U << 6)

#define IC_STATUS_RFNE		(1U << 3)
#define IC_STATUS_TFNF		(1U << 1)
#define IC_STATUS_ACTIVITY	(1U << 0)

#define IC_DATA_CMD_READ	(1U << 8)
#define IC_DATA_CMD_STOP	(1U << 9)

#define IC_INTR_TX_ABRT		(1U << 6)

static uintptr_t i2c_base(uint8_t bus_id)
{
	switch (bus_id) {
	case 0: return (uintptr_t)I2C0_BASE;
	case 1: return (uintptr_t)I2C1_BASE;
	case 2: return (uintptr_t)I2C2_BASE;
	case 3: return (uintptr_t)I2C3_BASE;
	case 4: return (uintptr_t)I2C4_BASE;
	default: return 0;
	}
}

static void i2c_enable(uintptr_t base, int on)
{
	int t = 100;

	do {
		mmio_write_32(base + IC_ENABLE, on ? 1 : 0);
		if ((mmio_read_32(base + IC_ENABLE_STATUS) & 1) == (on ? 1U : 0U))
			return;
		udelay(25);
	} while (--t);
}

static int wait_tx_abrt(uintptr_t base)
{
	if (mmio_read_32(base + IC_RAW_INTR_STAT) & IC_INTR_TX_ABRT) {
		(void)mmio_read_32(base + IC_TX_ABRT_SOURCE);
		(void)mmio_read_32(base + IC_CLR_TX_ABRT);
		return -1;
	}
	return 0;
}

int poll_i2c_init(uint8_t bus_id)
{
	uintptr_t base = i2c_base(bus_id);

	if (!base)
		return -1;

	i2c_enable(base, 0);
	mmio_write_32(base + IC_CON,
		      IC_CON_MASTER | IC_CON_SPEED_FS |
		      IC_CON_RESTART_EN | IC_CON_SLAVE_DISABLE);
	/* ~100 MHz IC_CLK approx; FS ~400 kHz ballpark counts from U-Boot. */
	mmio_write_32(base + IC_FS_SCL_HCNT, 0x36);
	mmio_write_32(base + IC_FS_SCL_LCNT, 0x67);
	mmio_write_32(base + IC_SS_SCL_HCNT, 0x1a4);
	mmio_write_32(base + IC_SS_SCL_LCNT, 0x1f0);
	mmio_write_32(base + IC_RX_TL, 0);
	mmio_write_32(base + IC_TX_TL, 0);
	mmio_write_32(base + IC_INTR_MASK, 0);
	i2c_enable(base, 1);

	printf("[balance] poll I2C%d init @%lx\n", bus_id, (unsigned long)base);
	return 0;
}

static int set_tar(uintptr_t base, uint8_t addr)
{
	int t = 1000;

	i2c_enable(base, 0);
	mmio_write_32(base + IC_TAR, addr);
	i2c_enable(base, 1);

	while (t--) {
		if (!(mmio_read_32(base + IC_STATUS) & IC_STATUS_ACTIVITY))
			break;
		udelay(1);
	}
	return 0;
}

int poll_i2c_write(uint8_t bus_id, uint8_t addr, uint8_t reg,
		   const uint8_t *data, uint16_t len)
{
	uintptr_t base = i2c_base(bus_id);
	uint16_t i;
	int t;

	if (!base)
		return -1;
	set_tar(base, addr);

	/* Write register address */
	t = 10000;
	while (!(mmio_read_32(base + IC_STATUS) & IC_STATUS_TFNF) && --t)
		udelay(1);
	if (!t)
		return -1;
	mmio_write_32(base + IC_DATA_CMD, reg);

	for (i = 0; i < len; i++) {
		t = 10000;
		while (!(mmio_read_32(base + IC_STATUS) & IC_STATUS_TFNF) && --t)
			udelay(1);
		if (!t)
			return -1;
		mmio_write_32(base + IC_DATA_CMD,
			      data[i] | ((i + 1 == len) ? IC_DATA_CMD_STOP : 0));
		if (wait_tx_abrt(base))
			return -1;
	}
	return 0;
}

int poll_i2c_read(uint8_t bus_id, uint8_t addr, uint8_t reg,
		  uint8_t *data, uint16_t len)
{
	uintptr_t base = i2c_base(bus_id);
	uint16_t i;
	int t;

	if (!base || !len)
		return -1;
	set_tar(base, addr);

	/* Write register, then repeated-start reads */
	t = 10000;
	while (!(mmio_read_32(base + IC_STATUS) & IC_STATUS_TFNF) && --t)
		udelay(1);
	if (!t)
		return -1;
	mmio_write_32(base + IC_DATA_CMD, reg);

	for (i = 0; i < len; i++) {
		t = 10000;
		while (!(mmio_read_32(base + IC_STATUS) & IC_STATUS_TFNF) && --t)
			udelay(1);
		if (!t)
			return -1;
		mmio_write_32(base + IC_DATA_CMD,
			      IC_DATA_CMD_READ |
			      ((i + 1 == len) ? IC_DATA_CMD_STOP : 0));

		t = 10000;
		while (!(mmio_read_32(base + IC_STATUS) & IC_STATUS_RFNE) && --t) {
			if (wait_tx_abrt(base))
				return -1;
			udelay(1);
		}
		if (!t)
			return -1;
		data[i] = (uint8_t)mmio_read_32(base + IC_DATA_CMD);
	}
	return 0;
}
