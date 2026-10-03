#include <math.h>
#include "sim_plant.h"

void plant_init(plant_t *p)
{
	p->g = 9.81;
	p->l = 0.10;
	p->ku = 0.05;
	p->deadband_pct = 6.0;
	p->wheel_circ_m = 0.065 * 3.14159265358979;
	p->counts_per_rev = 11.0 * 4.0 * 90.0;
	p->actuator_tau_s = 0.02;
	p->theta = p->omega = p->x = p->v = p->u_applied = 0.0;
}

void plant_step(plant_t *p, double u_cmd, double dt)
{
	double u, a, alpha;
	const int sub = 10;
	int i;

	for (i = 0; i < sub; i++) {
		double h = dt / sub;

		p->u_applied += (u_cmd - p->u_applied) * (h / p->actuator_tau_s);
		u = p->u_applied;
		if (fabs(u) < p->deadband_pct)
			u = 0.0;
		else
			u -= (u > 0 ? p->deadband_pct : -p->deadband_pct);

		a = p->ku * u;				/* wheel accel */
		alpha = (p->g / p->l) * sin(p->theta) +
			(a / p->l) * cos(p->theta);	/* body pitch accel */
		p->omega += alpha * h;
		p->theta += p->omega * h;
		p->v += a * h;
		p->x += p->v * h;
	}
}

double plant_theta_deg(const plant_t *p) { return p->theta * 57.2957795; }

double plant_encoder_counts(const plant_t *p)
{
	return p->x / p->wheel_circ_m * p->counts_per_rev;
}
