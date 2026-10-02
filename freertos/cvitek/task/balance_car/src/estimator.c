#include <math.h>
#include "estimator.h"

#define RAD2DEG 57.2957795f

static float accel_pitch(float ax, float ay, float az)
{
	return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

void bc_est_reset(bc_est_t *e)
{
	e->theta = e->omega = e->theta_acc = e->bias = 0.0f;
	e->norm = 1.0f;
	e->gated = 0;
	e->initialized = 0;
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

void bc_est_update(bc_est_t *e, const bc_params_t *p,
		   float ax, float ay, float az, float gy, float dt)
{
	float rate = (float)p->imu_sign * gy - e->bias;
	float pred, err;

	if (dt <= 0.0f)
		dt = 0.005f;
	e->norm = sqrtf(ax * ax + ay * ay + az * az);
	e->theta_acc = (float)p->imu_sign * accel_pitch(ax, ay, az);
	e->omega = rate;

	if (!e->initialized) {
		e->theta = e->theta_acc;
		e->initialized = 1;
	}
	pred = e->theta + rate * dt;

	if (p->est_gate_g > 0.0f && fabsf(e->norm - 1.0f) > p->est_gate_g) {
		/* the car itself is accelerating: trust the gyro only */
		e->gated = 1;
		e->theta = pred;
		return;
	}
	e->gated = 0;
	err = e->theta_acc - pred;
	e->theta = p->est_alpha * pred + (1.0f - p->est_alpha) * e->theta_acc;
	if (p->est_bias_gain > 0.0f && fabsf(rate) < 15.0f)
		e->bias -= p->est_bias_gain * err * dt;
}
