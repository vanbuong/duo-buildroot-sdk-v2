#include <math.h>
#include "estimator.h"

#define RAD2DEG 57.2957795f

static float accel_pitch(float ax, float ay, float az)
{
	return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

#define SPIKE_RUN_MAX	200	/* 1 s at 200 Hz: if every sample is "wrong", the estimate is */

void bc_est_reset(bc_est_t *e)
{
	e->theta = e->omega = e->theta_acc = e->bias = 0.0f;
	e->norm = 1.0f;
	e->gated = 0;
	e->initialized = 0;
	e->spike = 0;
	e->spike_run = 0;
	e->spikes = 0;
	e->P[0][0] = 0.5f; e->P[0][1] = 0.0f;
	e->P[1][0] = 0.0f; e->P[1][1] = 1.0f;
}

void bc_est_init_accel(bc_est_t *e, const bc_params_t *p,
		       float ax, float ay, float az)
{
	bc_est_reset(e);
	e->norm = sqrtf(ax * ax + ay * ay + az * az);
	e->theta_acc = (float)p->imu_sign * accel_pitch(ax, ay, az);
	e->theta = e->theta_acc;
	e->initialized = 1;
}

/* Is the accelerometer usable this cycle? Sets gated/spike. */
static int accel_usable(bc_est_t *e, const bc_params_t *p, float pred)
{
	e->spike = 0;
	if (p->est_gate_g > 0.0f && fabsf(e->norm - 1.0f) > p->est_gate_g) {
		e->gated = 1;			/* the car itself is accelerating */
		return 0;
	}
	e->gated = 0;
	if (p->est_spike_deg > 0.0f && fabsf(e->theta_acc - pred) > p->est_spike_deg) {
		if (e->spike_run < SPIKE_RUN_MAX) {
			e->spike = 1;		/* knock/impact or glitch: ignore this sample */
			e->spike_run++;
			e->spikes++;
			return 0;
		}
		return 1;			/* rejected for 1 s straight: the estimate is the wrong one */
	}
	e->spike_run = 0;
	return 1;
}

static void update_complementary(bc_est_t *e, const bc_params_t *p, float rate, float dt)
{
	float pred = e->theta + rate * dt, err;

	if (!accel_usable(e, p, pred)) {
		e->theta = pred;
		return;
	}
	err = e->theta_acc - pred;
	e->theta = p->est_alpha * pred + (1.0f - p->est_alpha) * e->theta_acc;
	if (p->est_bias_gain > 0.0f && fabsf(rate) < 15.0f)
		e->bias -= p->est_bias_gain * err * dt;
}

static void update_kalman(bc_est_t *e, const bc_params_t *p, float rate_raw, float dt)
{
	float (*P)[2] = e->P;
	float rate = rate_raw - e->bias, pred, S, K0, K1, y, r, p00, p01;

	/* predict */
	e->theta += rate * dt;
	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + p->kf_q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += p->kf_q_bias * dt;
	pred = e->theta;
	e->omega = rate;

	if (!accel_usable(e, p, pred))
		return;				/* covariance keeps growing: gyro-only */

	/* update with the accelerometer angle; a large innovation means the car is
	 * accelerating sideways (direction twisted, |a| still ~1 g): inflate R */
	y = e->theta_acc - e->theta;
	r = p->kf_r;
	if (p->kf_robust_deg > 0.0f) {
		float z = y / p->kf_robust_deg;

		r *= 1.0f + z * z;
	}
	S = P[0][0] + r;
	K0 = P[0][0] / S;
	K1 = P[1][0] / S;
	e->theta += K0 * y;
	e->bias += K1 * y;
	p00 = P[0][0];
	p01 = P[0][1];
	P[0][0] -= K0 * p00;
	P[0][1] -= K0 * p01;
	P[1][0] -= K1 * p00;
	P[1][1] -= K1 * p01;
	e->omega = rate_raw - e->bias;
}

void bc_est_update(bc_est_t *e, const bc_params_t *p,
		   float ax, float ay, float az, float gy, float dt)
{
	float rate_raw = (float)p->imu_sign * gy;

	if (dt <= 0.0f)
		dt = 0.005f;
	e->norm = sqrtf(ax * ax + ay * ay + az * az);
	e->theta_acc = (float)p->imu_sign * accel_pitch(ax, ay, az);

	if (!e->initialized) {
		e->theta = e->theta_acc;
		e->initialized = 1;
	}
	if (p->est_mode == 1) {
		update_kalman(e, p, rate_raw, dt);
	} else {
		e->omega = rate_raw - e->bias;
		update_complementary(e, p, e->omega, dt);
	}
}
