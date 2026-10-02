/*
 * Attitude estimator (pitch) - docs/balance_car/03 section 3.2.
 * Gated complementary filter with online gyro-bias tracking. Pure math.
 */
#ifndef BC_ESTIMATOR_H
#define BC_ESTIMATOR_H

#include "bc_params.h"

typedef struct {
	float theta;		/* fused pitch [deg], sign per imu_sign       */
	float omega;		/* bias-corrected pitch rate [deg/s]          */
	float theta_acc;	/* accelerometer-only pitch [deg]             */
	float norm;		/* |accel| [g]                                */
	float bias;		/* residual gyro bias estimate [deg/s]        */
	int gated;		/* 1 = accel rejected this cycle              */
	int initialized;
} bc_est_t;

void bc_est_reset(bc_est_t *e);
/* Start from the accelerometer angle (robot held still). */
void bc_est_init_accel(bc_est_t *e, const bc_params_t *p,
		       float ax, float ay, float az);
/* ax..az in g, gy in deg/s (boot bias already removed), dt in seconds. */
void bc_est_update(bc_est_t *e, const bc_params_t *p,
		   float ax, float ay, float az, float gy, float dt);

#endif
