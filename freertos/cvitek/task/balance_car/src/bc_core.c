#include <math.h>
#include "bc_core.h"

void bc_core_init(bc_core_t *c, const bc_params_t *p)
{
	c->p = *p;
	bc_est_reset(&c->est);
	bc_ctrl_init(&c->ctl, &c->p);
	bc_state_init(&c->sm);
	bc_imu_health_init(&c->health);
	c->ax = c->ay = 0.0f;
	c->az = 1.0f;
	c->gy = c->gz = 0.0f;
	c->stale_cycles = 0;
	c->last_sat = 0;
}

void bc_core_set_params(bc_core_t *c, const bc_params_t *p)
{
	bc_pid_t pa = c->ctl.pa, pv = c->ctl.pv, pt = c->ctl.pt;

	c->p = *p;
	bc_ctrl_init(&c->ctl, &c->p);
	/* keep integrator state across a parameter change */
	c->ctl.pa.integral = pa.integral;
	c->ctl.pv.integral = pv.integral;
	c->ctl.pt.integral = pt.integral;
}

void bc_core_step(bc_core_t *c, const bc_core_in_t *in, bc_core_out_t *out)
{
	const bc_params_t *p = &c->p;
	float ax = in->ax, ay = in->ay, az = in->az, gy = in->gy, gz = in->gz;
	float dt = in->dt > 0.0f ? in->dt : 0.005f;
	bc_state_in_t si;
	bc_ctrl_in_t ci;

	if (in->imu_ok) {
		c->ax = ax; c->ay = ay; c->az = az; c->gy = gy; c->gz = gz;
		c->stale_cycles = 0;
	} else {
		/* reuse the last good sample; the health monitor escalates */
		ax = c->ax; ay = c->ay; az = c->az; gy = c->gy; gz = c->gz;
		c->stale_cycles++;
	}
	bc_imu_health_update(&c->health, in->imu_ok, in->raw, in->raw_gx,
			     in->raw_gy, in->raw_gz,
			     sqrtf(ax * ax + ay * ay + az * az), dt * 1000.0f);

	if (!c->est.initialized)
		bc_est_init_accel(&c->est, p, ax, ay, az);
	else
		bc_est_update(&c->est, p, ax, ay, az, gy, dt);

	si.theta = c->est.theta;
	si.trim = p->trim_deg;
	si.omega = c->est.omega;
	si.v_f = c->ctl.v_f;
	si.dt_ms = dt * 1000.0f;
	si.saturated = c->last_sat;		/* from the previous cycle */
	si.imu_bus_fail = c->health.bus_fail;
	si.imu_stuck = c->health.stuck;
	si.imu_range = c->health.range;
	si.timing_fault = in->timing_fault;
	si.events = in->events;
	bc_state_step(&c->sm, &si, p);

	out->state = c->sm.state;
	out->fault = c->sm.fault;
	out->motor_enable = bc_state_is_balancing(&c->sm);
	out->theta = c->est.theta;
	out->theta_acc = c->est.theta_acc;
	out->omega = c->est.omega;
	out->norm = c->est.norm;
	out->gated = c->est.gated;

	if (out->motor_enable) {
		ci.theta = c->est.theta;
		ci.omega = c->est.omega;
		ci.omega_z = gz;
		ci.enc_l = in->enc_l;
		ci.enc_r = in->enc_r;
		ci.v_target = in->v_target;
		ci.w_target = in->w_target;
		ci.dt = dt;
		bc_ctrl_step(&c->ctl, p, &ci, &out->ctl);
		out->out_l = (float)p->motor_sign_l * out->ctl.out_l;
		out->out_r = (float)p->motor_sign_r * out->ctl.out_r;
		c->last_sat = out->ctl.saturated;
	} else {
		c->last_sat = 0;
		bc_ctrl_reset(&c->ctl);
		out->ctl.saturated = 0;
		out->ctl.out_l = out->ctl.out_r = 0.0f;
		out->ctl.u = out->ctl.turn = out->ctl.theta_cmd = 0.0f;
		out->ctl.v_f = out->ctl.v_l = out->ctl.v_r = 0.0f;
		out->out_l = out->out_r = 0.0f;
	}
}
