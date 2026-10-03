#include "i2c_recover.h"

#define HALF_PERIOD_US 5	/* ~100 kHz */

int bc_i2c_recover(const bc_i2c_recover_ops_t *o, int *clocks_used)
{
	int n = 0;

	o->sda(o->ctx, 1);
	o->scl(o->ctx, 1);
	o->delay_us(o->ctx, HALF_PERIOD_US);
	while (!o->sda_read(o->ctx) && n < BC_I2C_RECOVER_MAX_CLOCKS) {
		o->scl(o->ctx, 0);
		o->delay_us(o->ctx, HALF_PERIOD_US);
		o->scl(o->ctx, 1);
		o->delay_us(o->ctx, HALF_PERIOD_US);
		n++;
	}
	if (clocks_used)
		*clocks_used = n;
	if (!o->sda_read(o->ctx))
		return -1;
	/* STOP: SDA low -> high while SCL is high */
	o->scl(o->ctx, 0);
	o->delay_us(o->ctx, HALF_PERIOD_US);
	o->sda(o->ctx, 0);
	o->delay_us(o->ctx, HALF_PERIOD_US);
	o->scl(o->ctx, 1);
	o->delay_us(o->ctx, HALF_PERIOD_US);
	o->sda(o->ctx, 1);
	o->delay_us(o->ctx, HALF_PERIOD_US);
	return 0;
}
