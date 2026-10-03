/*
 * I2C bus recovery (pure logic, host-testable).
 *
 * A slave (the MPU) that was reset mid-byte can hold SDA low forever. The
 * standard fix is to clock SCL up to 9 times until the slave releases SDA and
 * then generate a STOP condition.
 */
#ifndef BC_I2C_RECOVER_H
#define BC_I2C_RECOVER_H

typedef struct {
	void *ctx;
	void (*scl)(void *ctx, int level);	/* drive SCL (open drain: 1 = release) */
	void (*sda)(void *ctx, int level);	/* drive SDA (open drain: 1 = release) */
	int (*sda_read)(void *ctx);
	void (*delay_us)(void *ctx, unsigned us);
} bc_i2c_recover_ops_t;

#define BC_I2C_RECOVER_MAX_CLOCKS 9

/*
 * Returns 0 when SDA is released (high) at the end and a STOP was generated,
 * -1 when SDA is still stuck low after 9 clocks (hardware problem).
 * *clocks_used (optional) receives the number of SCL pulses issued.
 */
int bc_i2c_recover(const bc_i2c_recover_ops_t *ops, int *clocks_used);

#endif
