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

static void mpu_variant_table(void)
{
	CHECK_EQ(mpu60x0_variant_from_id(0x68), MPU_VARIANT_6050);
	CHECK_EQ(mpu60x0_variant_from_id(0x70), MPU_VARIANT_6500);	/* MPU6500 */
	CHECK_EQ(mpu60x0_variant_from_id(0x71), MPU_VARIANT_6500);	/* MPU9250 */
	CHECK_EQ(mpu60x0_variant_from_id(0x73), MPU_VARIANT_6500);	/* MPU9255 */
	CHECK_EQ(mpu60x0_variant_from_id(0x98), MPU_VARIANT_6500);	/* ICM-20689 */
	CHECK_EQ(mpu60x0_variant_from_id(0x12), MPU_VARIANT_6500);	/* ICM-20602 */
	CHECK_EQ(mpu60x0_variant_from_id(0xAF), MPU_VARIANT_6500);	/* ICM-20608 */
	CHECK_EQ(mpu60x0_variant_from_id(0x00), MPU_VARIANT_UNKNOWN);
	CHECK_EQ(mpu60x0_variant_from_id(0xFF), MPU_VARIANT_UNKNOWN);
}

static void mpu_init_detects_each_6500_family_id(void)
{
	const uint8_t ids[] = { 0x70, 0x71, 0x73, 0x98 };
	size_t i;

	for (i = 0; i < sizeof(ids); i++) {
		mpu60x0_t imu;

		fake_i2c_reset();
		fake_i2c_set_reg(0x75, ids[i]);
		CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);
		CHECK_EQ(imu.variant, MPU_VARIANT_6500);
		CHECK_EQ(fake_i2c_get_reg(0x1D), 0x03);
		CHECK_EQ(fake_i2c_get_reg(0x1B), 0x08);		/* common setup identical */
		CHECK_EQ(fake_i2c_get_reg(0x1A), 0x03);
	}
}

static void mpu_init_6050_does_not_touch_accel_config2(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_reg(0x75, 0x68);
	fake_i2c_set_reg(0x1D, 0xA5);			/* undefined on the 6050 */
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);
	CHECK_EQ(imu.variant, MPU_VARIANT_6050);
	CHECK_EQ(fake_i2c_get_reg(0x1D), 0xA5);
}

static void mpu_init_unknown_id_falls_back_to_6050_map(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_reg(0x75, 0x42);
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);
	CHECK_EQ(imu.variant, MPU_VARIANT_6050);
	CHECK_EQ(imu.who_am_i, 0x42);
	CHECK_EQ(fake_i2c_get_reg(0x1D), 0x00);
}

static void mpu_init_resets_device_and_signal_paths_then_wakes(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_reg(0x75, 0x70);
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);
	CHECK_EQ(fake_i2c_get_reg(0x68), 0x07);		/* SIGNAL_PATH_RESET written */
	CHECK_EQ(fake_i2c_get_reg(0x6B) & 0x87, 0x01);	/* awake, reset bit clear, PLL clock */
}

static void mpu_init_fails_when_a_critical_register_does_not_stick(void)
{
	const uint8_t regs[] = { 0x1A, 0x1B, 0x1C, 0x19 };
	size_t i;

	for (i = 0; i < sizeof(regs); i++) {
		mpu60x0_t imu;

		fake_i2c_reset();
		fake_i2c_set_reg(0x75, 0x68);
		/* preload a different value and make the register read-only */
		fake_i2c_set_reg(regs[i], 0x55);
		fake_i2c_ignore_writes(regs[i], 1);
		CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), -2);
	}
}

static void mpu_init_tolerates_clone_ignoring_accel_config2(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_set_reg(0x75, 0x70);			/* claims 6500 */
	fake_i2c_ignore_writes(0x1D, 1);		/* but behaves like a 6050 */
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), 0);	/* warning only */
	CHECK_EQ(imu.variant, MPU_VARIANT_6500);
}

static void mpu_init_reports_bus_error_on_who_am_i(void)
{
	mpu60x0_t imu;

	fake_i2c_reset();
	fake_i2c_fail_reads(1);
	CHECK_EQ(mpu60x0_init(&imu, 1, 0x68), -1);
}

static void mpu_temperature_conversion_per_variant(void)
{
	mpu60x0_t imu = { 0 };
	mpu60x0_scaled_t s;

	imu.temp = 0;
	imu.variant = MPU_VARIANT_6050;
	mpu60x0_scale(&imu, &s);
	CHECK_NEAR(s.temp_c, 36.53, 0.01);		/* 6050: raw 0 = 36.53 C */
	imu.variant = MPU_VARIANT_6500;
	mpu60x0_scale(&imu, &s);
	CHECK_NEAR(s.temp_c, 21.0, 0.01);		/* 6500: raw 0 = 21 C   */
	imu.temp = 3339;
	mpu60x0_scale(&imu, &s);
	CHECK_NEAR(s.temp_c, 31.0, 0.05);
	imu.temp = 3400;
	imu.variant = MPU_VARIANT_6050;
	mpu60x0_scale(&imu, &s);
	CHECK_NEAR(s.temp_c, 46.53, 0.01);
}

void suite_mpu(void)
{
	printf("suite mpu60x0\n");
	RUN(mpu_init_6050_register_setup);
	RUN(mpu_init_6500_sets_accel_dlpf);
	RUN(mpu_variant_table);
	RUN(mpu_init_detects_each_6500_family_id);
	RUN(mpu_init_6050_does_not_touch_accel_config2);
	RUN(mpu_init_unknown_id_falls_back_to_6050_map);
	RUN(mpu_init_resets_device_and_signal_paths_then_wakes);
	RUN(mpu_init_fails_when_a_critical_register_does_not_stick);
	RUN(mpu_init_tolerates_clone_ignoring_accel_config2);
	RUN(mpu_init_reports_bus_error_on_who_am_i);
	RUN(mpu_temperature_conversion_per_variant);
	RUN(mpu_scale_constants_match_register_setup);
	RUN(mpu_read_decodes_big_endian_signed);
	RUN(mpu_read_propagates_i2c_error);
	RUN(mpu_scale_applies_bias_and_units);
	RUN(mpu_calibration_measures_bias_all_axes);
	RUN(mpu_calibration_rejects_motion);
	RUN(mpu_calibration_rejects_bad_gravity);
	RUN(mpu_calibration_fails_cleanly_on_bus_error);
}
