#include <math.h>
#include <string.h>
#include "imu_health.h"

#define STUCK_FRAMES	20
#define BUS_FAIL_MAX	3
#define SAT_RAW		32700
#define SAT_CYCLES	3

void bc_imu_health_init(bc_imu_health_t *h)
{
	memset(h, 0, sizeof(*h));
}

static int is_sat(int16_t g)
{
	return g >= SAT_RAW || g <= -SAT_RAW;
}

void bc_imu_health_update(bc_imu_health_t *h, int ok, const uint8_t *frame,
			  int16_t gx, int16_t gy, int16_t gz, float norm,
			  float dt_ms)
{
	if (!ok) {
		h->total_errors++;
		if (++h->bus_fail_streak >= BUS_FAIL_MAX)
			h->bus_fail = 1;
		return;
	}
	h->bus_fail_streak = 0;
	h->bus_fail = 0;

	if (frame) {
		if (h->have_last && memcmp(h->last, frame, sizeof(h->last)) == 0) {
			if (++h->same_count >= STUCK_FRAMES)
				h->stuck = 1;
		} else {
			h->same_count = 0;
			h->stuck = 0;
		}
		memcpy(h->last, frame, sizeof(h->last));
		h->have_last = 1;
	}

	if (is_sat(gx) || is_sat(gy) || is_sat(gz)) {
		if (++h->sat_count > SAT_CYCLES)
			h->range = 1;
	} else {
		h->sat_count = 0;
	}

	if (norm < 0.5f || norm > 1.5f) {
		h->norm_bad_ms += dt_ms;
		if (h->norm_bad_ms > 100.0f)
			h->norm_warn = 1;
		if (h->norm_bad_ms > 500.0f)
			h->range = 1;
	} else {
		h->norm_bad_ms = 0.0f;
		h->norm_warn = 0;
	}
}
