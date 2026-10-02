#include "unity_lite.h"
#include "mpu60x0.h"
#include "fake_hw.h"

#define RAD2DEG 57.2957795f

/* Feed a static tilt of `deg` (accel only, gyro zero) for `n` samples. */
static float settle_at(mpu60x0_t *imu, float deg, int n)
{
	float s = sinf(deg / RAD2DEG), c = cosf(deg / RAD2DEG);
	int i;

	/* firmware: pitch = atan2(-ax, hypot(ay, az)) */
	fake_i2c_set_accel_gyro((int16_t)(-s * 16384.0f), 0,
				(int16_t)(c * 16384.0f), 0, 0, 0);
	for (i = 0; i < n; i++) {
		mpu60x0_read(imu);
		mpu60x0_update_angle(imu, 0.005f);
	}
	return imu->angle_deg;
}

static void mpu_init_accepts_known_who_am_i(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_reg(0x75, 0x68);
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);
	CHECK_EQ(fake_i2c_get_reg(0x6B), 0x01);	/* wake, PLL x-gyro */
	CHECK_EQ(fake_i2c_get_reg(0x1A), 0x03);	/* DLPF 42 Hz */
	CHECK_EQ(fake_i2c_get_reg(0x1B), 0x00);	/* +-250 dps */
	CHECK_EQ(fake_i2c_get_reg(0x1C), 0x00);	/* +-2 g */
}

static void mpu_read_decodes_big_endian_signed(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_accel_gyro(1, -2, 16384, -131, 262, -32768);
	CHECK_EQ(mpu60x0_read(&imu), 0);
	CHECK_EQ(imu.ax, 1);
	CHECK_EQ(imu.ay, -2);
	CHECK_EQ(imu.az, 16384);
	CHECK_EQ(imu.gx, -131);
	CHECK_EQ(imu.gy, 262);
	CHECK_EQ(imu.gz, -32768);
}

static void mpu_read_propagates_i2c_error(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_fail_reads(1);
	CHECK(mpu60x0_read(&imu) != 0);
}

static void mpu_filter_converges_to_accel_tilt(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	/* tau = alpha*dt/(1-alpha) = 0.245 s -> 3 s is >12 tau */
	CHECK_NEAR(settle_at(&imu, 10.0f, 600), 10.0f, 0.1);
	CHECK_NEAR(settle_at(&imu, -25.0f, 600), -25.0f, 0.1);
	CHECK_NEAR(settle_at(&imu, 0.0f, 600), 0.0f, 0.1);
}

static void mpu_gyro_integrates_between_accel_corrections(void)
{
	mpu60x0_t imu = { 0 };
	int i;

	fake_i2c_reset();
	/* accel says level, gyro says +100 dps: the filter reaches a steady
	 * lag of about gyro * tau = 100 * 0.245 = 24.5 deg. */
	fake_i2c_set_accel_gyro(0, 0, 16384, 0, (int16_t)(100 * 131), 0);
	for (i = 0; i < 1000; i++) {
		mpu60x0_read(&imu);
		mpu60x0_update_angle(&imu, 0.005f);
	}
	CHECK_NEAR(imu.angle_deg, 24.5, 1.0);
}

static void mpu_gyro_bias_calibration_removes_offset(void)
{
	mpu60x0_t imu = { 0 };
	int i;

	fake_i2c_reset();
	/* 1.5 dps constant offset, robot upright and still. */
	fake_i2c_set_accel_gyro(0, 0, 16384, 0, (int16_t)(1.5f * 131), 0);
	CHECK_EQ(mpu60x0_calibrate_gyro(&imu, 200), 0);
	CHECK_NEAR(imu.gyro_bias_y, 1.5, 0.02);
	for (i = 0; i < 2000; i++) {
		mpu60x0_read(&imu);
		mpu60x0_update_angle(&imu, 0.005f);
	}
	CHECK_NEAR(imu.angle_deg, 0.0, 0.05);
}

static void mpu_calibration_fails_cleanly_on_bus_error(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	fake_i2c_fail_reads(1);
	CHECK(mpu60x0_calibrate_gyro(&imu, 50) != 0);
}

void suite_mpu(void)
{
	printf("suite mpu60x0\n");
	RUN(mpu_init_accepts_known_who_am_i);
	RUN(mpu_read_decodes_big_endian_signed);
	RUN(mpu_read_propagates_i2c_error);
	RUN(mpu_filter_converges_to_accel_tilt);
	RUN(mpu_gyro_integrates_between_accel_corrections);
	RUN(mpu_gyro_bias_calibration_removes_offset);
	RUN(mpu_calibration_fails_cleanly_on_bus_error);
}
