/* IMU data sanity checks - docs/balance_car/03 section 6. Pure logic. */
#ifndef BC_IMU_HEALTH_H
#define BC_IMU_HEALTH_H

#include <stdint.h>

typedef struct {
	uint8_t last[14];
	int have_last;
	int same_count;
	int sat_count;
	int bus_fail_streak;
	float norm_bad_ms;
	/* results */
	int bus_fail;		/* >= 3 consecutive read errors      */
	int stuck;		/* >= 20 identical frames            */
	int range;		/* gyro clipped >3 cycles or accel norm out of window > 500 ms */
	int norm_warn;		/* accel norm out of window > 100 ms */
	unsigned total_errors;
} bc_imu_health_t;

void bc_imu_health_init(bc_imu_health_t *h);
/* ok = read succeeded; frame = the 14 raw bytes (may be NULL); gx..gz raw
 * int16; norm in g; dt_ms cycle time. */
void bc_imu_health_update(bc_imu_health_t *h, int ok, const uint8_t *frame,
			  int16_t gx, int16_t gy, int16_t gz, float norm,
			  float dt_ms);

#endif
