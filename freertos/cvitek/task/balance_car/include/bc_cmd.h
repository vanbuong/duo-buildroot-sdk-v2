/* Command snapshot + heartbeat watchdog - docs/balance_car/01 section 7.2. */
#ifndef BC_CMD_H
#define BC_CMD_H

#include <stdint.h>
#include "bc_params.h"

typedef struct {
	int32_t speed_mmps, turn_mradps;
	uint32_t target_ms;		/* time the last SET_TARGET arrived   */
	uint32_t hb_counter;
	uint32_t hb_changed_ms;		/* last time the counter changed      */
	int seen_hb;
} bc_cmd_t;

void bc_cmd_init(bc_cmd_t *c, uint32_t now_ms);
void bc_cmd_set_target(bc_cmd_t *c, int32_t speed_mmps, int32_t turn_mradps,
		       uint32_t now_ms);
void bc_cmd_heartbeat(bc_cmd_t *c, uint32_t counter, uint32_t now_ms);
/*
 * Effective targets: clamped to v_max/w_max, forced to zero when the target
 * or the heartbeat is older than cmd_timeout_ms. *hb_lost = 1 when Linux has
 * been silent for hb_disarm_ms (and that threshold is non-zero).
 */
void bc_cmd_effective(const bc_cmd_t *c, const bc_params_t *p, uint32_t now_ms,
		      float *v_mps, float *w_rads, int *zeroed, int *hb_lost);

#endif
