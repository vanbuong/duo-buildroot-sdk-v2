#include <math.h>
#include "unity_lite.h"
#include "estimator.h"

#define R2D 57.2957795

static void acc_for(double deg, float *ax, float *az)
{
	*ax = (float)(-sin(deg / R2D));
	*az = (float)cos(deg / R2D);
}

static void est_init_from_accel_matches_tilt(void)
{
	bc_params_t p;
	bc_est_t e;
	float ax, az;

	bc_params_default(&p);
	acc_for(7.5, &ax, &az);
	bc_est_init_accel(&e, &p, ax, 0.0f, az);	/* EST-04 */
	CHECK_NEAR(e.theta, 7.5, 0.05);
	CHECK(e.initialized);
}

static void est_converges_to_static_tilt(void)
{
	bc_params_t p;
	bc_est_t e;
	float ax, az;
	int i;

	bc_params_default(&p);
	bc_est_reset(&e);
	acc_for(10.0, &ax, &az);
	for (i = 0; i < 4000; i++)			/* 20 s, tau = 2.5 s */
		bc_est_update(&e, &p, ax, 0.0f, az, 0.0f, 0.005f);
	/* first call self-initialises from the accelerometer */
	CHECK_NEAR(e.theta, 10.0, 0.05);
}

static void est_integrates_gyro_over_short_windows(void)
{
	bc_params_t p;
	bc_est_t e;
	int i;

	bc_params_default(&p);
	p.est_bias_gain = 0.0f;
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	/* accel still says level (low weight), gyro 100 dps for 0.1 s */
	for (i = 0; i < 20; i++)
		bc_est_update(&e, &p, 0.0f, 0.0f, 1.0f, 100.0f, 0.005f);
	CHECK_NEAR(e.theta, 10.0 - 0.5, 0.6);		/* ~10 deg, accel pulls back a bit */
	CHECK_NEAR(e.omega, 100.0, 1e-3);
}

static void est_gate_rejects_accelerating_frames(void)
{
	bc_params_t p;
	bc_est_t e;
	int i;

	bc_params_default(&p);
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	/* car accelerates: |a| = 1.5 g and misleading direction */
	for (i = 0; i < 200; i++)
		bc_est_update(&e, &p, -1.1f, 0.0f, 1.0f, 0.0f, 0.005f);
	CHECK(e.gated);					/* EST-01 */
	CHECK_NEAR(e.theta, 0.0, 1e-4);			/* untouched, gyro only */
	/* with the gate off the same data drags the angle */
	p.est_gate_g = 0.0f;
	for (i = 0; i < 200; i++)
		bc_est_update(&e, &p, -1.1f, 0.0f, 1.0f, 0.0f, 0.005f);
	CHECK(e.theta > 2.0);
}

static void est_zero_vector_is_finite(void)
{
	bc_params_t p;
	bc_est_t e;

	bc_params_default(&p);
	bc_est_reset(&e);
	bc_est_update(&e, &p, 0.0f, 0.0f, 0.0f, 0.0f, 0.005f);
	CHECK(isfinite(e.theta) && isfinite(e.norm));
}

static void est_tracks_gyro_bias_while_still(void)
{
	bc_params_t p;
	bc_est_t e;
	int i;

	bc_params_default(&p);
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	for (i = 0; i < 24000; i++)			/* 120 s, 2 dps offset */
		bc_est_update(&e, &p, 0.0f, 0.0f, 1.0f, 2.0f, 0.005f);
	CHECK_NEAR(e.bias, 2.0, 0.15);			/* EST-02 */
	CHECK_NEAR(e.omega, 0.0, 0.15);
	CHECK_NEAR(e.theta, 0.0, 0.2);
}

static void est_bias_frozen_while_gate_closed(void)
{
	bc_params_t p;
	bc_est_t e;
	int i;

	bc_params_default(&p);
	bc_est_init_accel(&e, &p, 0.0f, 0.0f, 1.0f);
	for (i = 0; i < 4000; i++)
		bc_est_update(&e, &p, 0.0f, 0.0f, 1.6f, 2.0f, 0.005f);	/* |a|=1.6 */
	CHECK_NEAR(e.bias, 0.0, 1e-6);
}

static void est_imu_sign_flips_angle_and_rate(void)
{
	bc_params_t p;
	bc_est_t e;
	float ax, az;

	bc_params_default(&p);
	p.imu_sign = -1;
	acc_for(8.0, &ax, &az);
	bc_est_init_accel(&e, &p, ax, 0.0f, az);
	CHECK_NEAR(e.theta, -8.0, 0.05);
	bc_est_update(&e, &p, ax, 0.0f, az, 10.0f, 0.005f);
	CHECK_NEAR(e.omega, -10.0, 1e-3);
}

void suite_estimator(void)
{
	printf("suite estimator\n");
	RUN(est_init_from_accel_matches_tilt);
	RUN(est_converges_to_static_tilt);
	RUN(est_integrates_gyro_over_short_windows);
	RUN(est_gate_rejects_accelerating_frames);
	RUN(est_zero_vector_is_finite);
	RUN(est_tracks_gyro_bias_while_still);
	RUN(est_bias_frozen_while_gate_closed);
	RUN(est_imu_sign_flips_angle_and_rate);
}
