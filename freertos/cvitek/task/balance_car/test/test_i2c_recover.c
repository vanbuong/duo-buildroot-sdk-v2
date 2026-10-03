#include "unity_lite.h"
#include "i2c_recover.h"

typedef struct {
	int scl, sda_master;
	int stuck_clocks;	/* slave releases SDA after this many SCL pulses (-1 = never) */
	int pulses, stop_seen;
	int prev_sda, last_scl;
	unsigned total_us;
} bus_t;

static void b_scl(void *c, int l)
{
	bus_t *b = c;

	if (l && !b->scl)
		b->pulses++;
	b->scl = l;
	b->last_scl = l;
}
static void b_sda(void *c, int l)
{
	bus_t *b = c;

	if (l && !b->sda_master && b->scl)
		b->stop_seen = 1;	/* SDA rising while SCL high */
	b->sda_master = l;
}
static int b_read(void *c)
{
	bus_t *b = c;
	int slave_low = b->stuck_clocks < 0 || b->pulses - 1 < b->stuck_clocks;

	if (b->pulses == 0 && b->stuck_clocks == 0)
		slave_low = 0;
	return b->sda_master && !slave_low;
}
static void b_delay(void *c, unsigned us) { ((bus_t *)c)->total_us += us; }

static int run(bus_t *b, int *clocks)
{
	bc_i2c_recover_ops_t ops = { b, b_scl, b_sda, b_read, b_delay };

	b->sda_master = 1;
	return bc_i2c_recover(&ops, clocks);
}

static void i2c_recover_free_bus_needs_no_clocks_but_sends_stop(void)
{
	bus_t b = { 0 };
	int n = -1;

	b.stuck_clocks = 0;
	b.scl = 1;
	CHECK_EQ(run(&b, &n), 0);
	CHECK_EQ(n, 0);
	CHECK(b.stop_seen);
}

static void i2c_recover_clocks_until_slave_releases(void)
{
	bus_t b = { 0 };
	int n = -1;

	b.stuck_clocks = 4;
	b.scl = 1;
	CHECK_EQ(run(&b, &n), 0);
	CHECK(n >= 4 && n <= 5);
	CHECK(n < BC_I2C_RECOVER_MAX_CLOCKS);
	CHECK(b.stop_seen);
}

static void i2c_recover_gives_up_after_nine_clocks(void)
{
	bus_t b = { 0 };
	int n = -1;

	b.stuck_clocks = -1;
	b.scl = 1;
	CHECK_EQ(run(&b, &n), -1);
	CHECK_EQ(n, 9);
	CHECK(!b.stop_seen);
	CHECK(b.total_us < 1000);	/* bounded: never blocks the control loop for long */
}

static void i2c_recover_accepts_null_counter(void)
{
	bus_t b = { 0 };

	b.stuck_clocks = 0;
	b.scl = 1;
	CHECK_EQ(run(&b, 0), 0);
}

void suite_i2c_recover(void)
{
	printf("suite i2c_recover\n");
	RUN(i2c_recover_free_bus_needs_no_clocks_but_sends_stop);
	RUN(i2c_recover_clocks_until_slave_releases);
	RUN(i2c_recover_gives_up_after_nine_clocks);
	RUN(i2c_recover_accepts_null_counter);
}
