/*
 * Cascaded balance controller (pure math) - docs/balance_car/04.
 * angle PD (inner) <- speed PI (outer, m/s) ; turn PI on yaw rate ; mixer.
 */
#ifndef BC_CONTROL_H
#define BC_CONTROL_H

#include "bc_params.h"
#include "pid.h"

typedef struct {
	bc_pid_t pa;		/* angle loop  */
	bc_pid_t pv;		/* speed loop  */
	bc_pid_t pt;		/* turn loop   */
	float v_t, w_t;		/* slew-limited targets [m/s], [rad/s] */
	float v_f;		/* low-passed forward speed [m/s]      */
	float theta_cmd;	/* lean command from the speed loop [deg] */
	int32_t prev_cl, prev_cr;
	int have_prev;
	float x_m;		/* odometry: distance travelled since BALANCING began [m] */
	float psi_rad;		/* heading from the yaw gyro [rad]                         */
	float hold_x;		/* position-hold anchor [m]                                */
	float still_s;		/* time the car has been commanded to stand still [s]      */
	int hold_active;
} bc_ctrl_t;

typedef struct {
	float theta;		/* deg  */
	float omega;		/* deg/s pitch rate */
	float omega_z;		/* deg/s yaw rate   */
	int32_t enc_l, enc_r;	/* raw counts       */
	float v_target, w_target;	/* requested [m/s], [rad/s] */
	float dt;		/* s */
} bc_ctrl_in_t;

typedef struct {
	float u;		/* common drive [%]            */
	float turn;		/* differential drive [%]      */
	float theta_cmd;	/* [deg]                       */
	float v_l, v_r, v_f;	/* wheel speeds [m/s]          */
	float out_l, out_r;	/* motor demand [%], +/-motor_max, deadband-compensated,
				   BEFORE motor_sign                  */
	int saturated;		/* angle loop at its limit     */
	float x_m, psi_rad;	/* odometry                    */
	int hold_active;	/* position hold engaged       */
} bc_ctrl_out_t;

void bc_ctrl_init(bc_ctrl_t *c, const bc_params_t *p);
/* Clear integrators/filters (call when not balancing). */
void bc_ctrl_reset(bc_ctrl_t *c);
void bc_ctrl_step(bc_ctrl_t *c, const bc_params_t *p,
		  const bc_ctrl_in_t *in, bc_ctrl_out_t *out);

/* helpers (exposed for tests) */
float bc_slew(float cur, float target, float max_delta);
float bc_deadband_comp(float x, float dz, float max);
float bc_counts_to_mps(int32_t dcounts, float dt, const bc_params_t *p);

#endif
