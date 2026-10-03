#include <math.h>
#include <string.h>
#include "unity_lite.h"
#include "dmp612.h"
#include "bc_core.h"
#include "bc_shm.h"
#include "fake_hw.h"

static uint8_t g_image[DMP612_CODE_SIZE];

static void put_be16(uint8_t *p, int16_t v) { p[0] = (uint8_t)((uint16_t)v >> 8); p[1] = (uint8_t)v; }

/* 28-byte packet: quaternion words are the high halves of 32-bit fields */
static void make_pkt(uint8_t *p, float w, float x, float y, float z)
{
	memset(p, 0, DMP612_PACKET_SIZE);
	put_be16(p + 0, (int16_t)lrintf(w * 16384.0f));
	put_be16(p + 4, (int16_t)lrintf(x * 16384.0f));
	put_be16(p + 8, (int16_t)lrintf(y * 16384.0f));
	put_be16(p + 12, (int16_t)lrintf(z * 16384.0f));
	put_be16(p + 16, 100); put_be16(p + 18, -200); put_be16(p + 20, 16384);
	put_be16(p + 22, 164); put_be16(p + 24, -328); put_be16(p + 26, 16);
}

static void setup(void)
{
	unsigned i;

	fake_i2c_reset();
	for (i = 0; i < sizeof(g_image); i++)
		g_image[i] = (uint8_t)(i * 7 + 3);
}

static void dmp_load_writes_image_and_configures_chip(void)
{
	mpu60x0_t imu;
	unsigned i;

	setup();
	memset(&imu, 0, sizeof(imu));
	CHECK_EQ(dmp612_load(&imu, g_image, sizeof(g_image)), 0);
	for (i = 0; i < sizeof(g_image); i++)
		if (fake_dmp_mem_get(i) != g_image[i]) {
			CHECK(0);
			break;
		}
	CHECK_EQ(fake_i2c_get_reg(0x70), 0x04);		/* DMP program start 0x0400 */
	CHECK_EQ(fake_i2c_get_reg(0x71), 0x00);
	CHECK_EQ(fake_i2c_get_reg(0x1B), 0x18);		/* +-2000 dps required by the DMP */
	CHECK_EQ(fake_i2c_get_reg(0x19), 0x01);
	CHECK_EQ(fake_i2c_get_reg(0x1A), 0x01);
	CHECK_EQ(fake_i2c_get_reg(0x6A) & 0xC0, 0xC0);	/* FIFO + DMP enabled */
	CHECK_EQ(fake_i2c_get_reg(0x38), 0x02);
	CHECK_NEAR(imu.gyro_lsb, 16.4, 1e-6);
}

static void dmp_load_rejects_wrong_size_and_null(void)
{
	mpu60x0_t imu;

	setup();
	memset(&imu, 0, sizeof(imu));
	CHECK_EQ(dmp612_load(&imu, g_image, 100), -3);
	CHECK_EQ(dmp612_load(&imu, 0, DMP612_CODE_SIZE), -3);
	CHECK_EQ(fake_i2c_write_count(), 0);		/* nothing touched */
}

static void dmp_load_detects_memory_that_does_not_stick(void)
{
	mpu60x0_t imu;

	setup();
	memset(&imu, 0, sizeof(imu));
	fake_dmp_mem_stuck(1500);
	CHECK_EQ(dmp612_load(&imu, g_image, sizeof(g_image)), -2);
	CHECK(imu.gyro_lsb == 0.0f);			/* not switched to the DMP scale */
}

static void dmp_parse_decodes_quaternion_gyro_accel(void)
{
	uint8_t p[DMP612_PACKET_SIZE];
	dmp_sample_t s;

	make_pkt(p, 0.5f, 0.5f, 0.5f, 0.5f);
	CHECK_EQ(dmp612_parse(p, &s), 0);
	CHECK_NEAR(s.qw, 0.5, 1e-4);
	CHECK_NEAR(s.qz, 0.5, 1e-4);
	CHECK_EQ(s.ax, 100); CHECK_EQ(s.ay, -200); CHECK_EQ(s.az, 16384);
	CHECK_EQ(s.gx, 164); CHECK_EQ(s.gy, -328); CHECK_EQ(s.gz, 16);
}

static void dmp_parse_rejects_non_unit_quaternion(void)
{
	uint8_t p[DMP612_PACKET_SIZE];
	dmp_sample_t s;

	make_pkt(p, 0.2f, 0.2f, 0.2f, 0.2f);		/* norm 0.4 */
	CHECK_EQ(dmp612_parse(p, &s), -1);
	memset(p, 0, sizeof(p));
	CHECK_EQ(dmp612_parse(p, &s), -1);
	make_pkt(p, 1.0f, 0.5f, 0.5f, 0.5f);	/* norm 1.32 */
	CHECK_EQ(dmp612_parse(p, &s), -1);
}

static void dmp_gravity_gives_pitch_like_the_accelerometer(void)
{
	/* a rotation of +10 deg about Y: the accelerometer reads ax = -sin(10), so the
	 * accelerometer pitch convention gives +10 deg - the same sign from gravity */
	bc_params_t p;
	bc_est_t e;
	uint8_t pk[DMP612_PACKET_SIZE];
	dmp_sample_t s;
	float g[3], half = 5.0f * 3.14159265f / 180.0f;

	bc_params_default(&p);
	p.est_mode = 2;
	make_pkt(pk, cosf(half), 0.0f, sinf(half), 0.0f);
	CHECK_EQ(dmp612_parse(pk, &s), 0);
	dmp612_gravity(&s, g);
	CHECK_NEAR(g[0], -sinf(2 * half), 2e-3);
	CHECK_NEAR(g[2], cosf(2 * half), 2e-3);

	bc_est_reset(&e);
	bc_est_init_accel(&e, &p, -sinf(2 * half), 0.0f, cosf(2 * half));	/* accel at +10 deg */
	bc_est_set_dmp(&e, 1, g);
	bc_est_update(&e, &p, -sinf(2 * half), 0.0f, cosf(2 * half), 0.0f, 0.005f);
	CHECK_NEAR(e.theta_dmp, 10.0, 0.2);
	CHECK_NEAR(e.theta_acc, 10.0, 0.01);
}

static void dmp_poll_returns_latest_packet_and_leaves_partial(void)
{
	mpu60x0_t imu;
	dmp_sample_t s;
	dmp_stats_t st;
	uint8_t a[DMP612_PACKET_SIZE], b[DMP612_PACKET_SIZE];

	setup();
	memset(&imu, 0, sizeof(imu));
	memset(&st, 0, sizeof(st));
	CHECK_EQ(dmp612_poll(&imu, &s, &st), 1);		/* empty */
	make_pkt(a, 1.0f, 0.0f, 0.0f, 0.0f);
	make_pkt(b, 0.9999f, 0.0f, 0.0f, 0.0f);
	put_be16(b + 0, 16000);				/* distinguishable */
	fake_fifo_push(a, sizeof(a));
	fake_fifo_push(b, sizeof(b));
	fake_fifo_push(a, 10);				/* partial third packet */
	CHECK_EQ(dmp612_poll(&imu, &s, &st), 0);
	CHECK_NEAR(s.qw, 16000.0 / 16384.0, 1e-4);	/* newest of the two complete packets */
	CHECK_EQ(st.packets, 2);
	CHECK_EQ(fake_fifo_len(), 10);			/* partial data kept for next time */
	CHECK_EQ(dmp612_poll(&imu, &s, &st), 1);	/* still incomplete */
}

static void dmp_poll_counts_bad_packets_and_keeps_good_one(void)
{
	mpu60x0_t imu;
	dmp_sample_t s;
	dmp_stats_t st;
	uint8_t good[DMP612_PACKET_SIZE], bad[DMP612_PACKET_SIZE];

	setup();
	memset(&imu, 0, sizeof(imu));
	memset(&st, 0, sizeof(st));
	make_pkt(good, 1.0f, 0.0f, 0.0f, 0.0f);
	memset(bad, 0, sizeof(bad));
	fake_fifo_push(good, sizeof(good));
	fake_fifo_push(bad, sizeof(bad));
	CHECK_EQ(dmp612_poll(&imu, &s, &st), 0);	/* the garbled newest one does not hide the good one */
	CHECK_EQ(st.bad_packets, 1);
	fake_fifo_push(bad, sizeof(bad));
	CHECK_EQ(dmp612_poll(&imu, &s, &st), 1);	/* only garbage: nothing usable */
}

static void dmp_poll_resets_fifo_on_overflow_or_lag(void)
{
	mpu60x0_t imu;
	dmp_sample_t s;
	dmp_stats_t st;
	uint8_t p[DMP612_PACKET_SIZE];
	int i, resets;

	setup();
	memset(&imu, 0, sizeof(imu));
	memset(&st, 0, sizeof(st));
	make_pkt(p, 1.0f, 0.0f, 0.0f, 0.0f);
	resets = fake_fifo_resets();
	for (i = 0; i < DMP612_MAX_PACKETS + 2; i++)		/* too many queued: stale */
		fake_fifo_push(p, sizeof(p));
	CHECK_EQ(dmp612_poll(&imu, &s, &st), -2);
	CHECK_EQ(fake_fifo_resets(), resets + 1);
	CHECK_EQ(fake_fifo_len(), 0);
	CHECK_EQ(st.overflows, 1);
	for (i = 0; i < 40; i++)				/* ~1120 B: hardware FIFO full */
		fake_fifo_push(p, sizeof(p));
	CHECK_EQ(dmp612_poll(&imu, &s, &st), -2);
	CHECK_EQ(st.overflows, 2);
}

static void dmp_poll_propagates_bus_error(void)
{
	mpu60x0_t imu;
	dmp_sample_t s;
	dmp_stats_t st;

	setup();
	memset(&imu, 0, sizeof(imu));
	memset(&st, 0, sizeof(st));
	fake_i2c_fail_reads(1);
	CHECK_EQ(dmp612_poll(&imu, &s, &st), -1);
}

static void dmp_scale_uses_dmp_gyro_range_after_load(void)
{
	mpu60x0_t imu;
	mpu60x0_scaled_t sc;

	setup();
	memset(&imu, 0, sizeof(imu));
	imu.gy = 164;
	mpu60x0_scale(&imu, &sc);
	CHECK_NEAR(sc.gy, 164.0 / MPU_GYRO_LSB_PER_DPS, 1e-4);	/* default range */
	CHECK_EQ(dmp612_load(&imu, g_image, sizeof(g_image)), 0);
	mpu60x0_scale(&imu, &sc);
	CHECK_NEAR(sc.gy, 10.0, 1e-3);				/* 164 LSB at 16.4 LSB/dps */
}

/* ---- estimator mode 2 ----------------------------------------------------- */
static void feed(bc_est_t *e, const bc_params_t *p, float theta_deg, int dmp_valid, float dmp_deg)
{
	float r = theta_deg * 3.14159265f / 180.0f, d = dmp_deg * 3.14159265f / 180.0f;
	float g[3] = { -sinf(d), 0.0f, cosf(d) };

	bc_est_set_dmp(e, dmp_valid, g);
	bc_est_update(e, p, -sinf(r), 0.0f, cosf(r), 0.0f, 0.005f);
}

static void est_mode2_uses_dmp_angle_when_it_agrees(void)
{
	bc_params_t p;
	bc_est_t e;

	bc_params_default(&p);
	p.est_mode = 2;
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	feed(&e, &p, 0.0f, 1, 2.0f);			/* filter 0 deg, DMP 2 deg: within 8 deg */
	CHECK(e.dmp_used);
	CHECK_NEAR(e.theta_out, 2.0, 0.05);
	CHECK(fabsf(e.theta) < 0.5f);			/* the filter itself is untouched */
	CHECK_EQ(e.dmp_fallbacks, 0);
}

static void est_mode2_falls_back_when_dmp_disagrees_or_is_missing(void)
{
	bc_params_t p;
	bc_est_t e;

	bc_params_default(&p);
	p.est_mode = 2;
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	feed(&e, &p, 0.0f, 1, 25.0f);			/* DMP far from the filter */
	CHECK(!e.dmp_used);
	CHECK_NEAR(e.theta_out, e.theta, 1e-6);
	feed(&e, &p, 0.0f, 0, 0.0f);			/* no fresh packet */
	CHECK(!e.dmp_used);
	CHECK_EQ(e.dmp_fallbacks, 2);
}

static void est_mode2_does_not_pin_the_filter_to_the_dmp(void)
{
	/* a DMP that drifts away must be noticed even though it was used before */
	bc_params_t p;
	bc_est_t e;
	int i, used_at_end = 1;

	bc_params_default(&p);
	p.est_mode = 2;
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	for (i = 0; i < 2000; i++) {			/* DMP drifts 0.05 deg per cycle */
		feed(&e, &p, 0.0f, 1, 0.05f * (float)i);
		used_at_end = e.dmp_used;
	}
	CHECK(!used_at_end);
	CHECK(fabsf(e.theta_out) < 1.0f);		/* fell back to the filter, not the drifted DMP */
}

static void est_modes_0_and_1_ignore_dmp_input(void)
{
	bc_params_t p;
	bc_est_t e;

	bc_params_default(&p);
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	feed(&e, &p, 0.0f, 1, 5.0f);
	CHECK(!e.dmp_used);
	CHECK_NEAR(e.theta_out, e.theta, 1e-6);
	p.est_mode = 1;
	feed(&e, &p, 0.0f, 1, 5.0f);
	CHECK(!e.dmp_used);
	CHECK_EQ(e.dmp_fallbacks, 0);
}

static void core_passes_dmp_to_estimator_and_flags_use(void)
{
	bc_core_t c;
	bc_core_in_t in;
	bc_core_out_t out;
	bc_params_t p;
	int i;

	bc_params_default(&p);
	p.est_mode = 2;
	bc_core_init(&c, &p);
	memset(&in, 0, sizeof(in));
	in.imu_ok = 1; in.az = 1.0f; in.dt = 0.005f;
	for (i = 0; i < 5; i++)
		bc_core_step(&c, &in, &out);
	in.dmp_valid = 1; in.dmp_g[0] = -0.0349f; in.dmp_g[1] = 0.0f; in.dmp_g[2] = 0.9994f;	/* 2 deg */
	bc_core_step(&c, &in, &out);
	CHECK(out.dmp_used);
	CHECK_NEAR(out.theta, 2.0, 0.05);
	in.imu_ok = 0;					/* a failed IMU read invalidates the DMP sample too */
	bc_core_step(&c, &in, &out);
	CHECK(!out.dmp_used);
}

/* ---- shared-memory image block --------------------------------------------- */
static void shm_dmp_block_roundtrip_and_corruption(void)
{
	static bc_shm_t shm;
	static uint8_t out[BC_DMP_DATA_MAX];
	uint32_t size = 0, seq;

	setup();
	memset(&shm, 0, sizeof(shm));
	CHECK_EQ(bc_dmp_blk_read(&shm.dmp, BC_DMP_KIND_612, out, sizeof(out), &size), -1);	/* none yet */
	seq = bc_dmp_blk_write(&shm.dmp, BC_DMP_KIND_612, g_image, sizeof(g_image));
	CHECK(seq != 0 && !(seq & 1u));
	CHECK_EQ(bc_dmp_blk_read(&shm.dmp, BC_DMP_KIND_612, out, sizeof(out), &size), 0);
	CHECK_EQ(size, DMP612_CODE_SIZE);
	CHECK(memcmp(out, g_image, size) == 0);
	CHECK_EQ(bc_dmp_blk_read(&shm.dmp, 99, out, sizeof(out), &size), -2);		/* wrong kind */
	CHECK_EQ(bc_dmp_blk_read(&shm.dmp, BC_DMP_KIND_612, out, 100, &size), -2);	/* buffer too small */
	shm.dmp.data[500] ^= 0x01;							/* bit flip */
	CHECK_EQ(bc_dmp_blk_read(&shm.dmp, BC_DMP_KIND_612, out, sizeof(out), &size), -2);
	CHECK_EQ(bc_dmp_blk_write(&shm.dmp, BC_DMP_KIND_612, g_image, BC_DMP_DATA_MAX + 1), 0);
	CHECK_EQ(offsetof(bc_shm_t, dmp), 0x6000);
}

void suite_dmp(void)
{
	printf("suite dmp\n");
	RUN(dmp_load_writes_image_and_configures_chip);
	RUN(dmp_load_rejects_wrong_size_and_null);
	RUN(dmp_load_detects_memory_that_does_not_stick);
	RUN(dmp_parse_decodes_quaternion_gyro_accel);
	RUN(dmp_parse_rejects_non_unit_quaternion);
	RUN(dmp_gravity_gives_pitch_like_the_accelerometer);
	RUN(dmp_poll_returns_latest_packet_and_leaves_partial);
	RUN(dmp_poll_counts_bad_packets_and_keeps_good_one);
	RUN(dmp_poll_resets_fifo_on_overflow_or_lag);
	RUN(dmp_poll_propagates_bus_error);
	RUN(dmp_scale_uses_dmp_gyro_range_after_load);
	RUN(est_mode2_uses_dmp_angle_when_it_agrees);
	RUN(est_mode2_falls_back_when_dmp_disagrees_or_is_missing);
	RUN(est_mode2_does_not_pin_the_filter_to_the_dmp);
	RUN(est_modes_0_and_1_ignore_dmp_input);
	RUN(core_passes_dmp_to_estimator_and_flags_use);
	RUN(shm_dmp_block_roundtrip_and_corruption);
}
