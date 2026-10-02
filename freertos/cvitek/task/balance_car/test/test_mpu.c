#include "unity_lite.h"
#include "mpu60x0.h"
#include "fake_hw.h"

#define G_LSB MPU_ACCEL_LSB_PER_G
#define W_LSB MPU_GYRO_LSB_PER_DPS

static void mpu_init_6050_register_setup(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_reg(0x75, 0x68);
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);
	CHECK_EQ(fake_i2c_get_reg(0x6B), 0x01);	/* wake, PLL x-gyro */
	CHECK_EQ(fake_i2c_get_reg(0x1A), 0x03);	/* DLPF 42 Hz */
	CHECK_EQ(fake_i2c_get_reg(0x1B), 0x08);	/* +-500 dps (S10) */
	CHECK_EQ(fake_i2c_get_reg(0x1C), 0x00);	/* +-2 g */
	CHECK_EQ(fake_i2c_get_reg(0x19), 0x00);	/* 1 kHz output (S10) */
	CHECK_EQ(fake_i2c_get_reg(0x1D), 0x00);	/* ACCEL_CONFIG2 untouched on 6050 */
	CHECK_EQ(imu.who_am_i, 0x68);
}

static void mpu_init_6500_sets_accel_dlpf(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_reg(0x75, 0x70);
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);
	CHECK_EQ(fake_i2c_get_reg(0x1D), 0x03);	/* S10: accel DLPF on the 6500 */
}

static void mpu_scale_constants_match_register_setup(void)
{
	/* guards a silent factor-of-two if the range register changes */
	CHECK_NEAR(MPU_GYRO_LSB_PER_DPS, 65.5, 1e-6);	/* GYRO_CONFIG 0x08 = +-500 */
	CHECK_NEAR(MPU_ACCEL_LSB_PER_G, 16384.0, 1e-6);	/* ACCEL_CONFIG 0x00 = +-2 g */
}

static void mpu_read_decodes_big_endian_signed(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	fake_i2c_set_accel_gyro(1, -2, 16384, -131, 262, -32768);
	CHECK_EQ(mpu60x0_read(&imu), 0);
	CHECK_EQ(imu.ax, 1);
	CHECK_EQ(imu.ay, -2);
	CHECK_EQ(imu.az, 16384);
	CHECK_EQ(imu.gx, -131);
	CHECK_EQ(imu.gy, 262);
	CHECK_EQ(imu.gz, -32768);
	CHECK_EQ(imu.raw[0], 0);
	CHECK_EQ(imu.raw[5], 0x00);
}

static void mpu_read_propagates_i2c_error(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	fake_i2c_fail_reads(1);
	CHECK(mpu60x0_read(&imu) != 0);
}

static void mpu_scale_applies_bias_and_units(void)
{
	mpu60x0_t imu = { 0 };
	mpu60x0_scaled_t s;

	imu.ax = 8192; imu.az = 16384; imu.gy = (int16_t)(10 * W_LSB);
	imu.gyro_bias[1] = 1.5f;
	mpu60x0_scale(&imu, &s);
	CHECK_NEAR(s.ax, 0.5, 1e-4);
	CHECK_NEAR(s.az, 1.0, 1e-4);
	CHECK_NEAR(s.gy, 8.5, 0.02);
}

static void mpu_calibration_measures_bias_all_axes(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	fake_i2c_set_accel_gyro(0, 0, (int16_t)G_LSB, (int16_t)(0.5f * W_LSB),
				(int16_t)(1.5f * W_LSB), (int16_t)(-2.0f * W_LSB));
	CHECK_EQ(mpu60x0_calibrate(&imu, 200), 0);
	CHECK_NEAR(imu.gyro_bias[0], 0.5, 0.03);
	CHECK_NEAR(imu.gyro_bias[1], 1.5, 0.03);
	CHECK_NEAR(imu.gyro_bias[2], -2.0, 0.03);
	CHECK_NEAR(imu.accel_mean[2], 1.0, 0.01);
	CHECK(imu.calibrated);
}

static int g_tick;
static void shake_hook(void)
{
	/* gyro y swings +-20 dps: robot is being moved */
	int16_t v = (int16_t)(((g_tick++ & 1) ? 20 : -20) * W_LSB);

	fake_i2c_set_accel_gyro(0, 0, (int16_t)G_LSB, 0, v, 0);
}

static void mpu_calibration_rejects_motion(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	g_tick = 0;
	fake_i2c_set_read_hook(shake_hook);
	CHECK_EQ(mpu60x0_calibrate(&imu, 100), -2);
	CHECK(!imu.calibrated);
	CHECK_NEAR(imu.gyro_bias[1], 0.0, 1e-9);	/* no partial result kept */
	fake_i2c_set_read_hook(0);
}

static void mpu_calibration_rejects_bad_gravity(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	fake_i2c_set_accel_gyro(0, 0, (int16_t)(1.4f * G_LSB), 0, 0, 0);
	CHECK_EQ(mpu60x0_calibrate(&imu, 50), -2);
	CHECK(!imu.calibrated);
}

static void mpu_calibration_fails_cleanly_on_bus_error(void)
{
	mpu60x0_t imu = { 0 };

	fake_i2c_reset();
	fake_i2c_fail_reads(1);
	CHECK_EQ(mpu60x0_calibrate(&imu, 50), -1);
}

void suite_mpu(void)
{
	printf("suite mpu60x0\n");
	RUN(mpu_init_6050_register_setup);
	RUN(mpu_init_6500_sets_accel_dlpf);
	RUN(mpu_scale_constants_match_register_setup);
	RUN(mpu_read_decodes_big_endian_signed);
	RUN(mpu_read_propagates_i2c_error);
	RUN(mpu_scale_applies_bias_and_units);
	RUN(mpu_calibration_measures_bias_all_axes);
	RUN(mpu_calibration_rejects_motion);
	RUN(mpu_calibration_rejects_bad_gravity);
	RUN(mpu_calibration_fails_cleanly_on_bus_error);
}
