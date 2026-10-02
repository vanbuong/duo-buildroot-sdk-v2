/*
 * Closed-loop tests: the real pid.c + mpu60x0.c run against a simulated
 * wheeled inverted pendulum (sim_plant.c). The loop below mirrors
 * balance_ctrl_task() in src/balance_main.c (200 Hz, same data flow).
 */
#include <stdint.h>
#include <stdlib.h>
#include "unity_lite.h"
#include "pid.h"
#include "mpu60x0.h"
#include "fake_hw.h"
#include "sim_plant.h"

#define RAD2DEG 57.2957795

typedef struct {
	double angle_kp, angle_kd;
	double speed_kp, speed_ki, speed_clamp;
	double ku;			/* plant gain override */
	double theta0_deg;		/* initial lean */
	double accel_noise_g;		/* uniform +-, per axis */
	double gyro_noise_dps;		/* uniform +- */
	double push_at_s, push_rad_s;	/* impulse on pitch rate */
	double seconds;
	/* estimator: 0 = shipped mpu60x0_update_angle() (alpha 0.98) */
	double est_alpha;		/* >0: reference estimator, see est_update() */
	double est_gate_g;		/* skip accel when | |a|-1g | > gate (0=off) */
} sim_cfg_t;

typedef struct {
	double max_theta_deg;
	double tail_theta_deg;		/* max |theta| over the last second */
	double max_x_m;
	double tail_speed_mps;		/* max |v| over the last second */
	int fell;			/* firmware would have disarmed (>45 deg) */
} sim_res_t;

static uint32_t g_rng = 12345;
static double urand(void)	/* deterministic, in [-1, 1] */
{
	g_rng = g_rng * 1664525u + 1013904223u;
	return ((g_rng >> 8) & 0xFFFF) / 32768.0 - 1.0;
}

/*
 * Reference estimator for the planned firmware change (docs/balance_car/03):
 * complementary filter with a larger gyro weight plus an accel-norm gate that
 * ignores the accelerometer while the car itself is accelerating hard.
 */
static void est_update(mpu60x0_t *m, double alpha, double gate_g, double dt)
{
	double ax = m->ax / 16384.0, ay = m->ay / 16384.0, az = m->az / 16384.0;
	double gy = m->gy / 131.0 - m->gyro_bias_y;
	double norm = sqrt(ax * ax + ay * ay + az * az);
	double acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD2DEG;
	double pred = m->angle_deg + gy * dt;

	if (gate_g > 0 && fabs(norm - 1.0) > gate_g)
		m->angle_deg = (float)pred;	/* gyro only */
	else
		m->angle_deg = (float)(alpha * pred + (1.0 - alpha) * acc_pitch);
}

static sim_cfg_t cfg_default(void)
{
	sim_cfg_t c = { 0 };

	c.angle_kp = 25.0; c.angle_kd = 0.8;
	c.speed_kp = 0.0; c.speed_ki = 0.0; c.speed_clamp = 15.0;
	c.ku = 0.05; c.theta0_deg = 5.0; c.seconds = 10.0;
	c.push_at_s = -1.0;
	return c;
}

static sim_res_t run_sim(const sim_cfg_t *c)
{
	const double dt = 0.005;
	const int n = (int)(c->seconds / dt);
	bc_pid_t pa, ps;
	mpu60x0_t imu = { 0 };
	plant_t pl;
	sim_res_t r = { 0 };
	double prev_cnt = 0;
	int i, pushed = 0;

	g_rng = 12345;
	fake_i2c_reset();
	plant_init(&pl);
	pl.ku = c->ku;
	pl.theta = c->theta0_deg / RAD2DEG;
	imu.angle_deg = (float)c->theta0_deg;	/* robot held at start angle */
	pid_init(&pa, (float)c->angle_kp, 0.0f, (float)c->angle_kd, -90.0f, 90.0f);
	pid_init(&ps, (float)c->speed_kp, (float)c->speed_ki, 0.0f,
		 (float)-c->speed_clamp, (float)c->speed_clamp);

	for (i = 0; i < n; i++) {
		double t = i * dt, s = sin(pl.theta), co = cos(pl.theta);
		double cnt, speed, aset, motor, lin_a_g;

		if (!pushed && c->push_at_s >= 0 && t >= c->push_at_s) {
			pl.omega += c->push_rad_s;
			pushed = 1;
		}
		/* Accelerometer also sees the wheel acceleration (ax). */
		lin_a_g = pl.ku * pl.u_applied / 9.81;
		fake_i2c_set_accel_gyro(
			(int16_t)((-s + lin_a_g * co + c->accel_noise_g * urand()) * 16384.0),
			(int16_t)(c->accel_noise_g * urand() * 16384.0),
			(int16_t)((co + lin_a_g * s + c->accel_noise_g * urand()) * 16384.0),
			0,
			(int16_t)((pl.omega * RAD2DEG + c->gyro_noise_dps * urand()) * 131.0),
			0);
		mpu60x0_read(&imu);
		if (c->est_alpha > 0)
			est_update(&imu, c->est_alpha, c->est_gate_g, dt);
		else
			mpu60x0_update_angle(&imu, (float)dt);

		cnt = plant_encoder_counts(&pl);
		speed = (cnt - prev_cnt) / dt;	/* counts/s, as in firmware */
		prev_cnt = cnt;

		aset = pid_update(&ps, 0.0f, (float)speed, (float)dt);
		motor = pid_update(&pa, (float)aset, imu.angle_deg, (float)dt);
		if (fabs(imu.angle_deg) > 45.0 && !r.fell)
			r.fell = 1;	/* firmware would coast here */
		plant_step(&pl, r.fell ? 0.0 : motor, dt);

		if (fabs(plant_theta_deg(&pl)) > r.max_theta_deg)
			r.max_theta_deg = fabs(plant_theta_deg(&pl));
		if (fabs(pl.x) > r.max_x_m)
			r.max_x_m = fabs(pl.x);
		if (t >= c->seconds - 1.0) {
			if (fabs(plant_theta_deg(&pl)) > r.tail_theta_deg)
				r.tail_theta_deg = fabs(plant_theta_deg(&pl));
			if (fabs(pl.v) > r.tail_speed_mps)
				r.tail_speed_mps = fabs(pl.v);
		}
		if (fabs(pl.theta) > 3.0)
			break;		/* diverged, stop wasting cycles */
	}
	if (fabs(pl.theta) > 3.0)
		r.max_theta_deg = 1e6;
	return r;
}

/* ---- tests ------------------------------------------------------------- */

/* Shipped configuration: src/balance_main.c + mpu60x0.c as of this commit. */
static sim_cfg_t cfg_shipped(void)
{
	sim_cfg_t c = cfg_default();

	c.angle_kp = 25.0; c.angle_kd = 0.8;
	c.speed_kp = 0.05; c.speed_ki = 0.01; c.speed_clamp = 15.0;
	return c;
}

/* Recommended configuration from docs/balance_car/03 and 04. */
static sim_cfg_t cfg_recommended(void)
{
	sim_cfg_t c = cfg_default();

	c.est_alpha = 0.998; c.est_gate_g = 0.1;
	c.angle_kp = 15.0; c.angle_kd = 0.8;
	c.speed_kp = -1.0e-4; c.speed_ki = -1.0e-4; c.speed_clamp = 8.0;
	c.seconds = 15.0;
	return c;
}

static void sim_recommended_balances_across_plant_gains(void)
{
	/* 0.4x .. 4x around the assumed motor/chassis gain. */
	const double kus[] = { 0.02, 0.05, 0.1, 0.2 };
	size_t i;

	for (i = 0; i < sizeof(kus) / sizeof(kus[0]); i++) {
		sim_cfg_t c = cfg_recommended();
		sim_res_t r;

		c.ku = kus[i];
		r = run_sim(&c);
		CHECK(!r.fell);
		CHECK(r.max_theta_deg < 12.0);
		CHECK(r.tail_theta_deg < 3.0);
		CHECK(r.max_x_m < 1.5);
		CHECK(r.tail_speed_mps < 0.15);
	}
}

static void sim_recommended_tolerates_sensor_noise(void)
{
	sim_cfg_t c = cfg_recommended();
	sim_res_t r;

	c.accel_noise_g = 0.03; c.gyro_noise_dps = 0.5;
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
}

static void sim_recommended_recovers_from_push(void)
{
	sim_cfg_t c = cfg_recommended();
	sim_res_t r;

	c.theta0_deg = 0.0; c.push_at_s = 2.0; c.push_rad_s = 0.6;	/* ~34 dps */
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
}

static void sim_recommended_starts_from_larger_lean(void)
{
	sim_cfg_t c = cfg_recommended();
	sim_res_t r;

	c.theta0_deg = 12.0;
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
}

/*
 * Guards the sign calibration step (docs/balance_car/04 section 7): with the
 * firmware's angle convention the speed loop needs a NEGATIVE gain. A wrong
 * encoder/motor polarity turns it into positive feedback: the car falls or
 * runs away.
 */
static void sim_wrong_speed_sign_diverges(void)
{
	sim_cfg_t c = cfg_recommended();
	sim_res_t r;

	c.speed_kp = +5.0e-4; c.speed_ki = +1.0e-4;
	r = run_sim(&c);
	/* either it falls, or it runs away instead of holding position */
	CHECK(r.fell || r.max_x_m > 5.0);
}

/*
 * KNOWN DEFECTS in the shipped firmware (docs/balance_car/04 and 05).
 * These pin current behaviour so the fix is visible in review: when
 * balance_main.c / mpu60x0.c are updated, invert or delete them.
 *   a) complementary alpha 0.98 trusts the accelerometer, which is
 *      contaminated by the car's own acceleration (positive feedback).
 *   b) speed gains +0.05 / +-15 deg are the wrong sign and ~500x too large
 *      for counts/s units.
 */
static void sim_KNOWN_DEFECT_shipped_config_falls(void)
{
	sim_cfg_t c = cfg_shipped();
	sim_res_t r = run_sim(&c);

	CHECK(r.fell || r.max_theta_deg > 45.0);
}

static void sim_KNOWN_DEFECT_shipped_speed_gains_fall_even_with_good_estimator(void)
{
	sim_cfg_t c = cfg_recommended();
	sim_res_t r;

	c.speed_kp = 0.05; c.speed_ki = 0.01; c.speed_clamp = 15.0;
	r = run_sim(&c);
	CHECK(r.fell || r.max_theta_deg > 45.0);
}

/* Flipping only the sign of the shipped gains is not enough: too large. */
static void sim_speed_gain_magnitude_matters_not_just_sign(void)
{
	sim_cfg_t c = cfg_recommended();
	sim_res_t r;

	c.speed_kp = -0.05; c.speed_ki = -0.01; c.speed_clamp = 15.0;
	r = run_sim(&c);
	CHECK(r.fell || r.max_theta_deg > 45.0);
}

/* Upper stability limit of the speed loop in the model (docs 04 section 3.2). */
static void sim_speed_loop_stable_up_to_4_deg_per_mps(void)
{
	sim_cfg_t c = cfg_recommended();
	sim_res_t r;
	double kcps = 3.9 / 19390.0;	/* 3.9 deg/(m/s) on the reference chassis */

	c.speed_kp = -kcps; c.speed_ki = -kcps;
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
	CHECK(r.max_x_m < 1.0);
}

static void sim_KNOWN_DEFECT_shipped_filter_falls_even_without_speed_loop(void)
{
	sim_cfg_t c = cfg_shipped();
	sim_res_t r;

	c.speed_kp = 0.0; c.speed_ki = 0.0;
	r = run_sim(&c);
	CHECK(r.fell || r.max_theta_deg > 45.0);
}

void suite_sim(void)
{
	printf("suite closed-loop simulation\n");
	RUN(sim_recommended_balances_across_plant_gains);
	RUN(sim_recommended_tolerates_sensor_noise);
	RUN(sim_recommended_recovers_from_push);
	RUN(sim_recommended_starts_from_larger_lean);
	RUN(sim_wrong_speed_sign_diverges);
	RUN(sim_KNOWN_DEFECT_shipped_config_falls);
	RUN(sim_speed_loop_stable_up_to_4_deg_per_mps);
	RUN(sim_speed_gain_magnitude_matters_not_just_sign);
	RUN(sim_KNOWN_DEFECT_shipped_speed_gains_fall_even_with_good_estimator);
	RUN(sim_KNOWN_DEFECT_shipped_filter_falls_even_without_speed_loop);
}
