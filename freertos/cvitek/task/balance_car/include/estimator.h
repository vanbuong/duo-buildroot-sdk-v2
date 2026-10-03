/*
 * Attitude estimator (pitch) - docs/balance_car/03 section 3.
 * est_mode 0: gated complementary filter with online gyro-bias tracking.
 * est_mode 1: 2-state Kalman filter (angle, gyro bias), same gate.
 * est_mode 2 (experimental): the complementary filter keeps running; the angle used by
 *   the controller (theta_out) is the DMP quaternion's pitch while it is fresh and within
 *   est_dmp_tol_deg of the filter, otherwise the filter's angle.
 * Both reject accelerometer samples that jump away from the prediction
 * (est_spike_deg). Pure math.
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
	int spike;		/* 1 = accel sample rejected as a spike       */
	int spike_run;		/* consecutive rejected samples               */
	unsigned spikes;	/* total rejected samples                     */
	float P[2][2];		/* Kalman covariance (mode 1)                 */
	float theta_out;	/* angle for the controller (= theta unless DMP is used) */
	int dmp_valid;		/* fresh DMP gravity vector supplied this cycle */
	float dmp_g[3];		/* gravity direction from the DMP quaternion [g] */
	float theta_dmp;	/* pitch from the DMP [deg], sign per imu_sign */
	int dmp_used;		/* 1 = theta_out is the DMP angle this cycle  */
	unsigned dmp_fallbacks;	/* mode 2 cycles that used the filter instead */
	int dmp_hold;		/* cycles the filter stays in charge after a disagreement */
} bc_est_t;

void bc_est_reset(bc_est_t *e);
/* Start from the accelerometer angle (robot held still). */
void bc_est_init_accel(bc_est_t *e, const bc_params_t *p,
		       float ax, float ay, float az);
/* Supply the DMP gravity vector for the next update (valid = a fresh packet). */
void bc_est_set_dmp(bc_est_t *e, int valid, const float g[3]);
/* ax..az in g, gy in deg/s (boot bias already removed), dt in seconds. */
void bc_est_update(bc_est_t *e, const bc_params_t *p,
		   float ax, float ay, float az, float gy, float dt);

#endif
