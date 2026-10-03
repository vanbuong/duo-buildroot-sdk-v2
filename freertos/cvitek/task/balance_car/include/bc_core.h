/*
 * One control cycle of the balance robot, hardware independent:
 *   health checks -> estimator -> state machine -> controller -> motor demand.
 * Used by balance_main.c on the C906L and by the host closed-loop tests.
 */
#ifndef BC_CORE_H
#define BC_CORE_H

#include <stdint.h>
#include "bc_params.h"
#include "estimator.h"
#include "control.h"
#include "bc_state.h"
#include "imu_health.h"

typedef struct {
	bc_params_t p;
	bc_est_t est;
	bc_ctrl_t ctl;
	bc_state_t sm;
	bc_imu_health_t health;
	/* last good sample, reused for up to 2 failed reads */
	float ax, ay, az, gy, gz;
	int stale_cycles;
	int last_sat;
} bc_core_t;

typedef struct {
	int imu_ok;			/* read succeeded                      */
	const uint8_t *raw;		/* 14-byte frame (stuck detection) or NULL */
	float ax, ay, az;		/* g                                   */
	float gy, gz;			/* deg/s, boot bias removed            */
	int16_t raw_gx, raw_gy, raw_gz;	/* raw gyro for range check            */
	int32_t enc_l, enc_r;
	float v_target, w_target;	/* effective targets [m/s], [rad/s]    */
	float dt;			/* s, measured                         */
	uint32_t events;		/* BC_EV_*                             */
	int timing_fault;
	int dmp_valid;			/* fresh DMP packet this cycle (est_mode 2) */
	float dmp_g[3];			/* gravity direction from the DMP [g]       */
} bc_core_in_t;

typedef struct {
	bc_state_id_t state;
	bc_fault_t fault;
	int motor_enable;		/* STBY / drive allowed                */
	float out_l, out_r;		/* % after motor_sign (hardware direction) */
	bc_ctrl_out_t ctl;
	float theta, theta_acc, omega, norm;
	int gated;
	int dmp_used;			/* theta is the DMP angle this cycle   */
} bc_core_out_t;

void bc_core_init(bc_core_t *c, const bc_params_t *p);
/* Apply a validated parameter set; controller state is kept. */
void bc_core_set_params(bc_core_t *c, const bc_params_t *p);
void bc_core_step(bc_core_t *c, const bc_core_in_t *in, bc_core_out_t *out);

#endif
