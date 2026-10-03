#include <math.h>
#include "control.h"

#define PI_F 3.14159265f
#define DEG2RAD 0.01745329252f

static float clampf(float x, float lo, float hi)
{
	return x < lo ? lo : (x > hi ? hi : x);
}

float bc_slew(float cur, float target, float max_delta)
{
	float d = target - cur;

	if (d > max_delta)
		return cur + max_delta;
	if (d < -max_delta)
		return cur - max_delta;
	return target;
}

float bc_deadband_comp(float x, float dz, float max)
{
	float a = fabsf(x), m;

	if (a < 0.5f)
		return 0.0f;
	m = dz + a * (100.0f - dz) / 100.0f;
	if (m > max)
		m = max;
	return x < 0 ? -m : m;
}

float bc_counts_to_mps(int32_t dcounts, float dt, const bc_params_t *p)
{
	float counts_per_m = p->enc_cpr / (PI_F * p->wheel_diam_m);

	if (dt <= 0.0f)
		return 0.0f;
	return (float)dcounts / dt / counts_per_m;
}

static void configure(bc_ctrl_t *c, const bc_params_t *p)
{
	/* Speed loop gains are positive magnitudes in the parameter set; with
	 * the convention of docs/04 section 7 the loop needs negative gains. */
	pid_init(&c->pa, p->a_kp, p->a_ki, p->a_kd,
		 -p->a_out_max, p->a_out_max);
	pid_set_i_limit(&c->pa, p->a_out_max * 0.3f);
	pid_init(&c->pv, -p->v_kp, -p->v_ki, 0.0f, -p->v_max_deg, p->v_max_deg);
	pid_set_i_limit(&c->pv, p->v_max_deg);
	pid_init(&c->pt, p->t_kp, p->t_ki, 0.0f, -p->t_max, p->t_max);
	pid_set_i_limit(&c->pt, p->t_max);
}

void bc_ctrl_init(bc_ctrl_t *c, const bc_params_t *p)
{
	configure(c, p);
	bc_ctrl_reset(c);
}

void bc_ctrl_reset(bc_ctrl_t *c)
{
	pid_reset(&c->pa);
	pid_reset(&c->pv);
	pid_reset(&c->pt);
	c->v_t = c->w_t = c->v_f = c->theta_cmd = 0.0f;
	c->have_prev = 0;
	c->x_m = c->psi_rad = c->hold_x = c->still_s = 0.0f;
	c->hold_active = 0;
}

void bc_ctrl_step(bc_ctrl_t *c, const bc_params_t *p,
		  const bc_ctrl_in_t *in, bc_ctrl_out_t *out)
{
	float dt = in->dt > 0.0f ? in->dt : 0.005f;
	float v_l, v_r, v, theta_set, u, turn, scale, tmax, w_meas, v_corr = 0.0f;

	/* wheel speeds from encoder deltas */
	if (!c->have_prev) {
		c->prev_cl = in->enc_l;
		c->prev_cr = in->enc_r;
		c->have_prev = 1;
	}
	v_l = (float)p->enc_sign_l * bc_counts_to_mps(in->enc_l - c->prev_cl, dt, p);
	v_r = (float)p->enc_sign_r * bc_counts_to_mps(in->enc_r - c->prev_cr, dt, p);
	c->prev_cl = in->enc_l;
	c->prev_cr = in->enc_r;
	v = 0.5f * (v_l + v_r);
	c->v_f += p->v_filter * (v - c->v_f);

	/* target shaping */
	c->v_t = bc_slew(c->v_t, clampf(in->v_target, -p->v_max, p->v_max),
			 p->acc_max * dt);
	c->w_t = bc_slew(c->w_t, clampf(in->w_target, -p->w_max, p->w_max),
			 p->alpha_max * dt);

	/* odometry (distance from the unfiltered wheel speeds, heading from the gyro) */
	c->x_m += v * dt;
	w_meas = in->omega_z * DEG2RAD * (float)p->imu_sign;
	c->psi_rad += w_meas * dt;

	/* position hold: once commanded to stand still and (nearly) stopped, pull the car
	 * back to where it stopped. Adds a bounded speed correction to the speed target. */
	if (p->x_kp > 0.0f && fabsf(in->v_target) < 0.01f) {
		c->still_s += dt;
		if (!c->hold_active && c->still_s > 0.5f && fabsf(c->v_f) < 0.05f) {
			c->hold_x = c->x_m;
			c->hold_active = 1;
		}
	} else {
		c->still_s = 0.0f;
		c->hold_active = 0;
	}
	if (c->hold_active)
		v_corr = clampf(p->x_kp * (c->hold_x - c->x_m), -p->x_vmax, p->x_vmax);

	/* outer loop: speed -> lean command (deg) */
	c->theta_cmd = pid_update(&c->pv, c->v_t + v_corr, c->v_f, dt);

	/* inner loop: angle PD with derivative on the gyro rate */
	theta_set = p->trim_deg + c->theta_cmd;
	u = pid_update_rate(&c->pa, theta_set, in->theta, in->omega, dt, 0);
	out->saturated = fabsf(u) >= p->a_out_max - 0.5f;

	/* turn loop on yaw rate; authority shrinks with speed */
	scale = 1.0f / (1.0f + fabsf(c->v_f) / p->t_speed_scale);
	tmax = p->t_max * scale;
	c->pt.out_min = -tmax;
	c->pt.out_max = tmax;
	turn = pid_update(&c->pt, c->w_t, w_meas, dt);

	/* mixer: keep the balance drive, give the turn what is left */
	{
		float lim = p->motor_max_pct;
		float room = lim - fabsf(u);
		float l, r;

		if (room < 0.0f)
			room = 0.0f;
		turn = clampf(turn, -room, room);
		l = bc_deadband_comp(u + turn, p->deadband_pct, lim);
		r = bc_deadband_comp(u - turn, p->deadband_pct, lim);
		out->out_l = l;
		out->out_r = r;
	}
	out->u = u;
	out->turn = turn;
	out->theta_cmd = c->theta_cmd;
	out->v_l = v_l;
	out->v_r = v_r;
	out->v_f = c->v_f;
	out->x_m = c->x_m;
	out->psi_rad = c->psi_rad;
	out->hold_active = c->hold_active;
}
