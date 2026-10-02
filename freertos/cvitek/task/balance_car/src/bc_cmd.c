#include "bc_cmd.h"

void bc_cmd_init(bc_cmd_t *c, uint32_t now_ms)
{
	c->speed_mmps = c->turn_mradps = 0;
	c->target_ms = now_ms;
	c->hb_counter = 0;
	c->hb_changed_ms = now_ms;
	c->seen_hb = 0;
}

void bc_cmd_set_target(bc_cmd_t *c, int32_t speed_mmps, int32_t turn_mradps,
		       uint32_t now_ms)
{
	c->speed_mmps = speed_mmps;
	c->turn_mradps = turn_mradps;
	c->target_ms = now_ms;
}

void bc_cmd_heartbeat(bc_cmd_t *c, uint32_t counter, uint32_t now_ms)
{
	if (!c->seen_hb || counter != c->hb_counter) {
		c->hb_counter = counter;
		c->hb_changed_ms = now_ms;
		c->seen_hb = 1;
	}
}

static float clampf(float x, float lim)
{
	return x > lim ? lim : (x < -lim ? -lim : x);
}

void bc_cmd_effective(const bc_cmd_t *c, const bc_params_t *p, uint32_t now_ms,
		      float *v_mps, float *w_rads, int *zeroed, int *hb_lost)
{
	/* unsigned subtraction is wrap-safe */
	uint32_t t_age = now_ms - c->target_ms;
	uint32_t h_age = now_ms - c->hb_changed_ms;
	int stale = t_age > (uint32_t)p->cmd_timeout_ms ||
		    h_age > (uint32_t)p->cmd_timeout_ms;

	*zeroed = stale;
	if (hb_lost)
		*hb_lost = p->hb_disarm_ms > 0.0f && h_age > (uint32_t)p->hb_disarm_ms;
	if (stale) {
		*v_mps = 0.0f;
		*w_rads = 0.0f;
		return;
	}
	*v_mps = clampf((float)c->speed_mmps * 0.001f, p->v_max);
	*w_rads = clampf((float)c->turn_mradps * 0.001f, p->w_max);
}
